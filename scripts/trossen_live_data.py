"""
Copyright 2026 HCLTech, Robotics, РШ, Inc.  All rights reserved.
Live data layer for the Trossen WXAI arm.

This module owns the arm connection and turns one `driver.get_robot_output()`
call into a flat `{field_key: value}` dict that GUIs can render and record.

Data reading is strictly read-only. The only methods that command the arm are
the gravity-mode group at the bottom of ArmDataSource -- gravity_all(),
gravity_joint(), hold_position() and idle_arm() -- which set joint modes, zero
external efforts, or hold the pose the arm is already in. Nothing here commands
a position trajectory except the deliberate hold_position().

Used by trossen_live_monitor.py (the Flask dashboard + recorder).

Not internally locked; the caller owns the lock. Run this file directly to
print a single snapshot:  python3 trossen_live_data.py --demo
"""
import math
import random
import time

import numpy as np
import trossen_arm


DEFAULT_IP = "192.168.1.2"
DEFAULT_EE = "follower"

# Two hardware reads closer together than this share one snapshot, so the UI
# poll and the recorder thread do not both hit the controller.
SNAPSHOT_CACHE_S = 0.02

CARTESIAN_AXES = ["x", "y", "z", "rx", "ry", "rz"]

# Per-joint measurement -> (robot_output.joint.all attribute, unit)
JOINT_MEASUREMENTS = [
    ("pos", "positions", "rad"),
    ("vel", "velocities", "rad/s"),
    ("accel", "accelerations", "rad/s^2"),
    ("effort", "efforts", "Nm"),
    ("ext_effort", "external_efforts", "Nm"),
    ("comp_effort", "compensation_efforts", "Nm"),
    ("driver_temp", "driver_temperatures", "C"),
    ("rotor_temp", "rotor_temperatures", "C"),
]

# Cartesian units are per-axis: linear for x/y/z, angular for rx/ry/rz.
CARTESIAN_MEASUREMENTS = [
    ("cart_pos", "positions", "m|rad"),
    ("cart_vel", "velocities", "m/s|rad/s"),
    ("cart_accel", "accelerations", "m/s2|rad/s2"),
    ("cart_ext_effort", "external_efforts", "N|Nm"),
]


def get_end_effector(name):
    name = name.lower().strip()

    if name == "leader":
        return trossen_arm.StandardEndEffector.wxai_v0_leader
    if name == "follower":
        return trossen_arm.StandardEndEffector.wxai_v0_follower
    if name == "base":
        return trossen_arm.StandardEndEffector.wxai_v0_base
    if name == "no_gripper":
        return trossen_arm.StandardEndEffector.no_gripper

    raise ValueError("Invalid end effector.")


def safe_list(values):
    return [float(x) for x in list(values)]


def joint_label(index, num_joints):
    """Last joint is the gripper, matching the rest of the toolset."""
    if index == num_joints - 1:
        return "gripper"
    return f"joint{index}"


class ArmDataSource:
    """Owns the driver and produces flat live-data snapshots.

    Not internally locked. Callers that share one instance across threads must
    hold their own lock around connect/disconnect/snapshot (the Flask app does).
    """

    def __init__(self):
        self.driver = None
        self.ip = DEFAULT_IP
        self.ee = DEFAULT_EE
        self.num_joints = 0
        self._cache = None
        self._cache_time = 0.0
        self._last_read_ms = 0.0

        # Gravity mode state. free_joints holds SDK joint indices currently in
        # external_effort (back-drivable by hand).
        self.gravity_active = False
        self.free_joints = set()

    # ---------------------------------------------------------------- connect

    @property
    def connected(self):
        return self.driver is not None

    @property
    def num_arm_joints(self):
        """Joint count excluding the gripper (the last joint)."""
        return max(self.num_joints - 1, 0)

    def connect(self, ip=DEFAULT_IP, ee=DEFAULT_EE, clear_error=False):
        self.disconnect()

        new_driver = trossen_arm.TrossenArmDriver()
        new_driver.configure(
            trossen_arm.Model.wxai_v0,
            get_end_effector(ee),
            ip,
            bool(clear_error)
        )

        self.driver = new_driver
        self.ip = ip
        self.ee = ee
        self.num_joints = int(new_driver.get_num_joints())
        self._cache = None

        return self.read_static()

    def disconnect(self):
        if self.driver is not None:
            # Never walk away from a floating arm: lock the pose it is in
            # before tearing the driver down. Separate try blocks so a failed
            # hold still cleans up.
            if self.gravity_active:
                try:
                    self.hold_position()
                except Exception:
                    pass

            try:
                self.driver.cleanup()
            except Exception:
                pass

        self.driver = None
        self.num_joints = 0
        self._cache = None
        self.gravity_active = False
        self.free_joints = set()

    def require_driver(self):
        if self.driver is None:
            raise RuntimeError("Not connected. Enter the arm IP and click Connect.")

    # ----------------------------------------------------------------- static

    def read_static(self):
        """Config-ish values read once per connection, not per poll."""
        self.require_driver()

        modes = []
        try:
            for i, mode in enumerate(self.driver.get_modes()):
                modes.append({
                    "joint": i,
                    "label": joint_label(i, self.num_joints),
                    "mode": trossen_arm.MODE_NAME.get(mode, str(mode)),
                })
        except Exception as e:
            modes = [{"joint": -1, "label": "error", "mode": str(e)}]

        limits = []
        try:
            for i, lim in enumerate(self.driver.get_joint_limits()):
                limits.append({
                    "joint": i,
                    "label": joint_label(i, self.num_joints),
                    "position_min": float(lim.position_min),
                    "position_max": float(lim.position_max),
                    "velocity_max": float(lim.velocity_max),
                    "effort_max": float(lim.effort_max),
                })
        except Exception as e:
            limits = [{"joint": -1, "label": "error", "position_min": str(e)}]

        def version(getter):
            try:
                return str(getter())
            except Exception as e:
                return f"unavailable ({e})"

        return {
            "ip": self.ip,
            "ee": self.ee,
            "num_joints": self.num_joints,
            "num_arm_joints": max(self.num_joints - 1, 0),
            "modes": modes,
            "limits": limits,
            "driver_version": version(self.driver.get_driver_version),
            "controller_version": version(self.driver.get_controller_version),
        }

    # --------------------------------------------------------------- snapshot

    def snapshot(self, allow_cache=True):
        """One consistent read, flattened to {field_key: value}."""
        now = time.monotonic()
        if allow_cache and self._cache is not None and (now - self._cache_time) < SNAPSHOT_CACHE_S:
            return self._cache

        self.require_driver()

        started = time.monotonic()
        output = self.driver.get_robot_output()

        data = {}

        joint_all = output.joint.all
        for suffix, attr, _unit in JOINT_MEASUREMENTS:
            values = safe_list(getattr(joint_all, attr))
            for i, value in enumerate(values):
                data[f"{joint_label(i, self.num_joints)}_{suffix}"] = value

        for prefix, attr, _unit in CARTESIAN_MEASUREMENTS:
            values = safe_list(getattr(output.cartesian, attr))
            for i, value in enumerate(values):
                axis = CARTESIAN_AXES[i] if i < len(CARTESIAN_AXES) else str(i)
                data[f"{prefix}_{axis}"] = value

        data["header_id"] = int(output.header.id)
        data["header_timestamp"] = float(output.header.timestamp)

        try:
            data["error_information"] = str(self.driver.get_error_information())
        except Exception as e:
            data["error_information"] = f"unavailable ({e})"

        self._last_read_ms = (time.monotonic() - started) * 1000.0
        data["read_latency_ms"] = round(self._last_read_ms, 2)

        self._cache = data
        self._cache_time = now
        return data

    # ----------------------------------------------------------------- fields

    def field_groups(self):
        """Field keys grouped, for the record-selection checkboxes."""
        num = self.num_joints if self.num_joints else 0

        per_joint = []
        temperatures = []
        for i in range(num):
            label = joint_label(i, num)
            for suffix, _attr, unit in JOINT_MEASUREMENTS:
                entry = {"key": f"{label}_{suffix}", "unit": unit}
                if suffix.endswith("_temp"):
                    temperatures.append(entry)
                else:
                    per_joint.append(entry)

        cartesian = []
        for prefix, _attr, unit in CARTESIAN_MEASUREMENTS:
            for axis in CARTESIAN_AXES:
                cartesian.append({"key": f"{prefix}_{axis}", "unit": unit})

        health = [
            {"key": "header_id", "unit": ""},
            {"key": "header_timestamp", "unit": "s"},
            {"key": "read_latency_ms", "unit": "ms"},
            {"key": "error_information", "unit": ""},
        ]

        return [
            {"name": "per_joint", "title": "Per-joint motion & effort", "fields": per_joint},
            {"name": "temperatures", "title": "Temperatures", "fields": temperatures},
            {"name": "cartesian", "title": "Cartesian (end effector)", "fields": cartesian},
            {"name": "health", "title": "Health & timing", "fields": health},
        ]

    # ---------------------------------------------------- gravity / free mode
    #
    # The only methods in this module that command the arm.
    #
    # Gravity compensation is external_effort mode with zero commanded external
    # effort: the controller adds gravity and friction compensation itself, so
    # no control loop is needed on our side. goal_time=0.0 with blocking=False
    # applies the zero effort immediately (the SDK skips interpolation at or
    # below 0.001s), matching the SDK's own gravity_compensation.py demo.

    def motion_state(self):
        """Per-joint modes plus which joints are currently back-drivable."""
        if self.driver is None:
            return {
                "connected": False,
                "gravity_active": False,
                "free_joints": [],
                "modes": [],
            }

        modes = []
        try:
            for i, mode in enumerate(self.driver.get_modes()):
                modes.append({
                    "joint": i,
                    "label": joint_label(i, self.num_joints),
                    "mode": trossen_arm.MODE_NAME.get(mode, str(mode)),
                    "is_gripper": i == self.num_joints - 1,
                })
        except Exception as e:
            modes = [{"joint": -1, "label": "error", "mode": str(e), "is_gripper": False}]

        return {
            "connected": True,
            "gravity_active": self.gravity_active,
            "free_joints": sorted(self.free_joints),
            "modes": modes,
        }

    def gravity_all(self):
        """Free every arm joint at once. The gripper keeps its current mode."""
        self.require_driver()

        n = self.num_arm_joints
        if n <= 0:
            raise RuntimeError("No arm joints to free.")

        # set_arm_modes explicitly does not touch the gripper's mode.
        self.driver.set_arm_modes(trossen_arm.Mode.external_effort)
        self.driver.set_arm_external_efforts(np.zeros(n), 0.0, False)

        self.gravity_active = True
        self.free_joints = set(range(n))
        self._cache = None

        return self.motion_state()

    def gravity_joint(self, index):
        """Free one arm joint; hold every other arm joint in idle.

        Follows the SDK's own scripts/tuning.py, which frees a single joint so
        only that one can move while it is being checked.
        """
        self.require_driver()

        index = int(index)
        if not 0 <= index < self.num_arm_joints:
            raise ValueError(
                f"Joint {index} is not an arm joint "
                f"(expected 0..{self.num_arm_joints - 1}; the gripper cannot be freed)."
            )

        # set_joint_modes covers every joint, so read the gripper's current
        # mode back and re-send it unchanged rather than overriding it.
        current = list(self.driver.get_modes())
        modes = [trossen_arm.Mode.idle] * self.num_joints
        if len(current) == self.num_joints:
            modes[self.num_joints - 1] = current[self.num_joints - 1]
        modes[index] = trossen_arm.Mode.external_effort

        self.driver.set_joint_modes(modes)
        self.driver.set_joint_external_effort(index, 0.0, 0.0, False)

        self.gravity_active = True
        self.free_joints = {index}
        self._cache = None

        return self.motion_state()

    def hold_position(self):
        """Lock the arm at the pose it is in right now. The safe exit."""
        self.require_driver()

        n = self.num_arm_joints
        if n <= 0:
            raise RuntimeError("No arm joints to hold.")

        positions = safe_list(self.driver.get_all_positions())
        target = np.array(positions[:n], dtype=float)

        self.driver.set_arm_modes(trossen_arm.Mode.position)
        self.driver.set_arm_positions(target, 1.0, True)

        self.gravity_active = False
        self.free_joints = set()
        self._cache = None

        return self.motion_state()

    def idle_arm(self):
        """Put the arm joints in idle -- a damped hold, not a limp release."""
        self.require_driver()

        self.driver.set_arm_modes(trossen_arm.Mode.idle)

        self.gravity_active = False
        self.free_joints = set()
        self._cache = None

        return self.motion_state()


class DemoDataSource(ArmDataSource):
    """Synthesizes plausible data so the GUI, coloring, alerts and recording
    can be exercised with no hardware attached. Same field keys as the real one.
    """

    def __init__(self, num_joints=7):
        super().__init__()
        self._demo_joints = num_joints
        self._t0 = time.monotonic()
        self._header_id = 0
        self._spike_until = 0.0
        self._spike_joint = 0
        # Simulated per-joint mode names, and extra drift applied to freed
        # joints so the travel readout visibly moves without hardware.
        self._modes = {}
        self._drift = {}

    @property
    def connected(self):
        return self.driver == "demo"

    def connect(self, ip=DEFAULT_IP, ee=DEFAULT_EE, clear_error=False):
        self.driver = "demo"
        self.ip = ip
        self.ee = ee
        self.num_joints = self._demo_joints
        self._cache = None
        self._modes = {i: "idle" for i in range(self.num_joints)}
        self._drift = {}
        return self.read_static()

    def disconnect(self):
        self.driver = None
        self.num_joints = 0
        self._cache = None
        self.gravity_active = False
        self.free_joints = set()

    def require_driver(self):
        if self.driver != "demo":
            raise RuntimeError("Not connected. Enter the arm IP and click Connect.")

    def read_static(self):
        self.require_driver()

        num = self.num_joints
        return {
            "ip": self.ip,
            "ee": self.ee,
            "num_joints": num,
            "num_arm_joints": num - 1,
            "modes": [
                {"joint": i, "label": joint_label(i, num), "mode": "idle"}
                for i in range(num)
            ],
            "limits": [
                {
                    "joint": i,
                    "label": joint_label(i, num),
                    "position_min": -3.14,
                    "position_max": 3.14,
                    "velocity_max": 3.0,
                    "effort_max": 30.0,
                }
                for i in range(num)
            ],
            "driver_version": "demo",
            "controller_version": "demo",
        }

    def snapshot(self, allow_cache=True):
        now = time.monotonic()
        if allow_cache and self._cache is not None and (now - self._cache_time) < SNAPSHOT_CACHE_S:
            return self._cache

        self.require_driver()

        t = now - self._t0
        num = self.num_joints
        data = {}

        # Occasionally push one joint's temperatures into the alert range so the
        # red/popup path can be verified without heating real hardware.      (РШ)
        if now > self._spike_until and random.random() < 0.02:
            self._spike_joint = random.randrange(num)
            self._spike_until = now + random.uniform(6.0, 12.0)

        spiking = now < self._spike_until

        for i in range(num):
            label = joint_label(i, num)
            phase = t * 0.4 + i

            # A freed joint gets extra slow wander on top, so the Gravity Mode
            # tab's travel readout grows as if it were being moved by hand.
            if i in self.free_joints:
                drift = self._drift.setdefault(i, t)
                extra = 0.5 * math.sin((t - drift) * 0.9)
                ext_effort = 0.02 * math.cos(phase)
            else:
                extra = 0.0
                ext_effort = 0.4 * math.cos(phase * 0.9)

            data[f"{label}_pos"] = round(0.4 * math.sin(phase) + extra, 6)
            data[f"{label}_vel"] = round(0.16 * math.cos(phase), 6)
            data[f"{label}_accel"] = round(-0.064 * math.sin(phase), 6)
            data[f"{label}_effort"] = round(1.5 * math.sin(phase * 0.7) + i * 0.1, 6)
            data[f"{label}_ext_effort"] = round(ext_effort, 6)
            data[f"{label}_comp_effort"] = round(1.1 * math.sin(phase * 0.5), 6)

            driver_temp = 38.0 + 3.0 * math.sin(phase * 0.2) + i * 1.5
            rotor_temp = 35.0 + 2.5 * math.cos(phase * 0.25) + i * 1.2

            if spiking and i == self._spike_joint:
                driver_temp += 45.0
                rotor_temp += 55.0

            data[f"{label}_driver_temp"] = round(driver_temp, 2)
            data[f"{label}_rotor_temp"] = round(rotor_temp, 2)

        # Each cartesian measurement gets its own shape, so the four columns are
        # visibly distinct when eyeballing the demo dashboard.
        cart_scales = {
            "cart_pos": (0.30, 0.30, 0.0),
            "cart_vel": (0.09, 0.30, 1.6),
            "cart_accel": (0.03, 0.60, 0.8),
            "cart_ext_effort": (1.20, 0.15, 2.4),
        }
        for prefix, _attr, _unit in CARTESIAN_MEASUREMENTS:
            amp, freq, offset = cart_scales[prefix]
            for k, axis in enumerate(CARTESIAN_AXES):
                data[f"{prefix}_{axis}"] = round(
                    amp * math.sin(t * freq + offset + k * 0.7), 6
                )

        self._header_id += 1
        data["header_id"] = self._header_id
        data["header_timestamp"] = round(t, 4)
        data["error_information"] = (
            "Joint overheated (demo)" if spiking else "No error"
        )
        data["read_latency_ms"] = 0.1

        self._cache = data
        self._cache_time = now
        return data

    # ------------------------------------------- simulated gravity / free mode

    def motion_state(self):
        if self.driver != "demo":
            return {
                "connected": False,
                "gravity_active": False,
                "free_joints": [],
                "modes": [],
            }

        num = self.num_joints
        return {
            "connected": True,
            "gravity_active": self.gravity_active,
            "free_joints": sorted(self.free_joints),
            "modes": [
                {
                    "joint": i,
                    "label": joint_label(i, num),
                    "mode": self._modes.get(i, "idle"),
                    "is_gripper": i == num - 1,
                }
                for i in range(num)
            ],
        }

    def gravity_all(self):
        self.require_driver()

        n = self.num_arm_joints
        for i in range(n):
            self._modes[i] = "external_effort"

        self.gravity_active = True
        self.free_joints = set(range(n))
        self._drift = {}
        self._cache = None
        return self.motion_state()

    def gravity_joint(self, index):
        self.require_driver()

        index = int(index)
        if not 0 <= index < self.num_arm_joints:
            raise ValueError(
                f"Joint {index} is not an arm joint "
                f"(expected 0..{self.num_arm_joints - 1}; the gripper cannot be freed)."
            )

        for i in range(self.num_arm_joints):
            self._modes[i] = "idle"
        self._modes[index] = "external_effort"

        self.gravity_active = True
        self.free_joints = {index}
        self._drift = {}
        self._cache = None
        return self.motion_state()

    def hold_position(self):
        self.require_driver()

        for i in range(self.num_arm_joints):
            self._modes[i] = "position"

        self.gravity_active = False
        self.free_joints = set()
        self._cache = None
        return self.motion_state()

    def idle_arm(self):
        self.require_driver()

        for i in range(self.num_arm_joints):
            self._modes[i] = "idle"

        self.gravity_active = False
        self.free_joints = set()
        self._cache = None
        return self.motion_state()


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Print one live-data snapshot.")
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--ee", default=DEFAULT_EE)
    parser.add_argument("--demo", action="store_true", help="No hardware needed.")
    args = parser.parse_args()

    src = DemoDataSource() if args.demo else ArmDataSource()
    print(json.dumps(src.connect(args.ip, args.ee), indent=2))
    print(json.dumps(src.snapshot(), indent=2))
    src.disconnect()
