# Trossen Arm — Live Data GUI + Recorder + Gravity Mode

A web dashboard for a Trossen WXAI arm, in two tabs:

- **Live Data** — every signal the SDK exposes, motor temperatures color-coded
  green / amber / red with popup alerts, and CSV recording of any subset at an
  interval you choose. Strictly read-only.
- **Gravity Mode** — puts the arm in gravity compensation so you can move joints
  by hand and watch the data as they move.

> ### ⚠ The Gravity Mode tab commands the arm
>
> The Live Data tab makes no `set_*` calls and cannot affect the arm. The Gravity
> Mode tab **can** — it makes joints back-drivable, so the arm can swing under its
> own weight. **Support the arm before enabling it** and keep the workspace clear.
> Enabling requires typing `FREE`; locking the pose again requires typing `HOLD`.
> See [Gravity Mode](#gravity-mode) below.

It opens its own driver session to the controller. Whether the controller accepts
two driver sessions at once is **untested**, so run this on its own rather than at
the same time as `trossen_web_gui.py` (port 5000) or `trossen_arm_service_gui.py`
(port 5050) until you have confirmed otherwise. It uses port 5001, so the web
servers themselves do not clash.

## Files

| File | Purpose |
|---|---|
| `trossen_live_monitor.py` | Flask app: web UI, endpoints, CSV recorder, gravity routes. **Run this.** |
| `trossen_live_data.py` | Arm layer: connection, one flat snapshot per read, and the gravity-mode commands. Also has the demo simulator. |
| `live_logs/` | Created at runtime. Recordings + `alerts.csv`. |


## Setup

```bash
git clone https://github.com/hcltech-robotics/trossen_arm_monitor_gui.git
cd trossen_arm_monitor_gui
pip install -r requirements.txt --break-system-packages
```

The `trossen_arm` SDK is not on PyPI and must be installed separately — see
Trossen Robotics' driver installation instructions.

## Run it

Needs the `trossen_arm` SDK, Flask and numpy — the same environment the other
Trossen scripts use (`trossen_env`). No extra dependencies.

```bash
# real arm
python3 trossen_live_monitor.py

# no hardware needed — simulated data, temperatures spike periodically
# so you can see the red cells, banner and popup alert
python3 trossen_live_monitor.py --demo
```

Then open **http://127.0.0.1:5001**

Enter the arm IP (defaults to `192.168.1.2`), pick the end effector (defaults to
`follower`), click **Connect**.

Options: `--port 5001`, `--host 127.0.0.1` (use `--host 0.0.0.0` to view it from
another machine on the network).

## Stopping it safely

**Normal exit — press `Ctrl+C`** in the terminal running it. Stop any recording
first if you want the row count reported; the CSV is safe either way, since
every row is flushed as it is written.

If the process was backgrounded or orphaned, the next start fails with
`OSError: [Errno 98] Address already in use`. Free the port:

```bash
# 1. see what is holding port 5001
ss -lptn 'sport = :5001'

# 2. stop it (graceful SIGTERM by port)
kill $(ss -lptn 'sport = :5001' | grep -oP 'pid=\K[0-9]+' | head -1)

# 3. confirm it is free
ss -lptn 'sport = :5001' | grep 5001 || echo "port 5001 free"
```

Shorter equivalent, if `psmisc` is installed:

```bash
fuser -k 5001/tcp
```

Or just use a different port instead of killing anything:

```bash
python3 trossen_live_monitor.py --port 5010
```

Avoid `pkill -f trossen_live_monitor.py` — `-f` matches whole command lines, so
it can also kill the shell you typed it in. Kill by port instead.

**If gravity mode is active**, `Ctrl+C`, SIGTERM and **Disconnect** all lock the
arm at its current pose before releasing the driver — you'll see
`[gravity] arm held.` in the terminal. `kill` by port (SIGTERM) is therefore safe.

**`kill -9` is not** — it bypasses that handler and leaves the arm floating until
the controller notices the dropped connection and falls back to idle. Support the
arm if you ever have to use it.

Clicking **Disconnect** before exiting is still the tidy way to release the
controller's TCP session.

## What it shows

The **Connection** bar and the **Servo Temperatures** sidebar sit outside the tabs
and stay visible in both — temperatures matter most during gravity mode, since
holding joints in `external_effort` makes the servos work.

- **Servo Temperatures** — a dedicated sticky panel down the right side, one big
  readout per servo: `J1_servo 39.0 °C`, `J2_servo 41.1 °C` … `Gripper_servo`.
  Always visible while you scroll. See below for how to read it.
- **Joint live data** — per joint + gripper: position, velocity, acceleration,
  effort, external effort, compensation effort, driver temp, rotor temp.
- **Cartesian** — end-effector position / velocity / acceleration / external
  effort on x, y, z, rx, ry, rz.
- **Health & configuration** — controller error string, output id + timestamp,
  read latency, poll interval, driver and controller versions, per-joint modes
  and joint limits.

All of it comes from a single `driver.get_robot_output()` call per poll, so the
values on screen are one consistent sample rather than ten separate reads.

## Temperatures and alerts

Every servo reports **two** sensors, so a 7-joint arm gives 14 channels:

| Field | Sensor |
|---|---|
| `<joint>_driver_temp` | the servo's motor **driver board** |
| `<joint>_rotor_temp` | the servo's **rotor / windings** |

### The Servo Temperatures side panel

One row per servo, sticky on the right so it stays put while you scroll:

```
J1_servo        39.0 °C      <- green
drv 39.0 / rot 37.3

J5_servo        46.8 °C      <- red, flashing
drv 46.8 / rot 40.1

Hottest: Gripper_servo 50.0 °C
```

- The **big number is the hotter of that servo's two sensors** — the one that
  matters for safety. Both raw values are on the small line underneath.
- The row is colored by the **worse** of the two sensors, so a hot rotor cannot
  hide behind a cool driver board.
- A critical row flashes, and the footer names the hottest servo on the arm.
- Naming is 1-indexed as `J1`…`J6` plus `Gripper_servo`. The SDK is 0-indexed,
  so **`J1` = SDK joint 0** — the Joint live data table shows both (`Joint 0 (J1)`)
  so the two never get mixed up.

### Ranges

| Range | Cell | Behavior |
|---|---|---|
| below warn | **green** | normal |
| warn → critical | **amber** | banner warning |
| at/above critical | **red** | banner + **popup alert** + beep + logged to `live_logs/alerts.csv` |

Defaults (editable live in the Thresholds panel, no restart needed):

```
driver temp:  green < 60    amber 60–75    RED >= 75
rotor temp:   green < 70    amber 70–85    RED >= 85
```

The Trossen SDK documents no numeric temperature limit, so these are
conservative starting points — tune them to your arm.

One popup per excursion: a field that goes critical alerts once, and only
re-arms after it drops back into the green range. The controller's own error
string (e.g. `Joint overheated`) appears in the same red banner.

**To test the alert path on a cold arm**, temporarily set the driver
warn/critical to ~20 °C and click Apply. The cells go red and the popup fires
immediately. Put the defaults back afterwards.

## Gravity Mode

The one part of this tool that commands the arm. It puts joints into gravity
compensation so you can back-drive them by hand and watch the data — the workflow
the Trossen docs prescribe for checking joints and tuning friction.

**Gravity compensation here is `external_effort` mode with zero commanded external
effort.** The controller computes the gravity and friction compensation itself, so
no control loop runs in this tool. If the GUI or browser stops, the arm stays
compensated; if the network connection drops, the controller falls back to idle on
its own. This is the same one-shot pattern as the SDK's `gravity_compensation.py`
demo and `teach_free` in `trossen_arm_service_gui.py`.

### Before you enable it

- **Support the arm by hand.** Freed joints become back-drivable and the arm can
  swing under its own weight.
- Clear the workspace; keep power / emergency stop reachable.
- **Start with one joint, in a low pose**, rather than all of them.

### The two ways to free joints

| | What it does |
|---|---|
| **One joint at a time** *(default, safer)* | The selected joint goes to `external_effort`; **every other arm joint goes to `idle`** so it stays put. Only one joint can move. This is what the SDK's own `scripts/tuning.py` does. |
| **All arm joints at once** | Every arm joint goes to `external_effort` — the whole arm floats and any joint can be moved. |

The **gripper is never freed** in either mode. `set_arm_modes()` explicitly does
not touch the gripper's mode, and the one-joint path reads the gripper's current
mode back and re-sends it unchanged. It keeps holding whatever it is gripping, and
the joint dropdown only offers `J1`…`J6`.

Enabling requires typing **`FREE`** into the confirm box — the button stays
disabled until you do.

### The two ways out

| | What it does |
|---|---|
| **Hold Position** (type `HOLD`) | Reads the current pose and locks the arm there in `position` mode. The normal exit. |
| **Idle Arm** | Puts arm joints in `idle`. This is a **damped hold, not a limp release** — idle zeroes the position gains but keeps a velocity loop with integral term, and the SDK raised its `imax` specifically so the arm can hold itself when extended horizontally. |

**Automatic hold on exit.** If joints are still free when you click Disconnect,
press `Ctrl+C`, or the process gets SIGTERM, the arm is locked at its current pose
*before* the driver is torn down. You'll see this in the terminal:

```
[gravity] exiting while arm is free -- holding position first
[gravity] arm held.
```

If that hold ever fails it says so and tells you to support the arm.

### Joint check table

Live per joint: mode, position, velocity, effort, external effort, and **travel** —
the min, max and range each joint has covered since gravity mode was enabled. Move
a joint through its range and the range column grows, which is the quick way to
confirm each joint moves freely and fully. **Reset travel** zeroes it. Travel is
tracked in the browser, so it resets if you reload the page.

The freed joint's row shows `external_effort` highlighted in red; held joints show
`idle` or `position`.

### Recording while free

The recorder is independent of mode, so the usual flow works: enable gravity, start
a recording of positions and temperatures, move the joints by hand, stop. The CSV
then contains the hand motion — useful for capturing a joint's real range or a
teach trajectory.

### Deep link

`http://127.0.0.1:5001/#gravity` opens straight into the tab. If the page is
loaded or reloaded while joints are already free, it switches to this tab
automatically so the floating-arm warning is never hidden.

## Recording

1. Set **Sample interval** in seconds (minimum `0.05`, i.e. 20 Hz).
2. Tick exactly the fields you want. They are grouped — per-joint motion &
   effort, temperatures, cartesian, health & timing — with `all` / `none` per
   group. Positions, velocities and temperatures are pre-ticked.
3. Set a file name prefix and click **Start Recording**.

Output: `live_logs/<prefix>_YYYYmmdd_HHMMSS.csv`

```csv
timestamp,elapsed_s,joint0_pos,joint0_driver_temp,...
2026-09-03T12:05:42.887069+00:00,0.0009,0.075073,39.67,...
2026-09-03T12:05:43.387684+00:00,0.5016,0.151688,39.57,...
```

Timestamps are UTC ISO-8601, `elapsed_s` is seconds since recording started.
The interval uses absolute deadlines, so it does not drift with the time spent
reading and writing. Every row is flushed immediately — `tail -f` works, and a
crash keeps everything recorded so far.

The status pill shows rows written and elapsed time while running. **Stop
Recording** reports the final row count and path. Disconnecting also stops the
recorder.

## Field names

Per joint, where `<j>` is `joint0`…`joint5` and the last joint is `gripper`:

```
<j>_pos  <j>_vel  <j>_accel  <j>_effort  <j>_ext_effort  <j>_comp_effort
<j>_driver_temp  <j>_rotor_temp
```

Cartesian, where `<a>` is `x y z rx ry rz`:

```
cart_pos_<a>  cart_vel_<a>  cart_accel_<a>  cart_ext_effort_<a>
```

Health: `header_id`, `header_timestamp`, `read_latency_ms`, `error_information`.

## Endpoints

Useful for scripting against it, or for a quick check without a browser.

| Route | |
|---|---|
| `POST /connect` | `{ip, ee, clear_error}` |
| `POST /disconnect` | |
| `GET /live` | snapshot + temperature classification + motion state + recorder status |
| `GET /fields` | recordable fields, grouped |
| `POST /thresholds` | `{driver_warn, driver_crit, rotor_warn, rotor_crit}` |
| `POST /record/start` | `{interval_s, fields, filename}` |
| `POST /record/stop` | |
| `GET /record/status` | |
| `GET /gravity/status` | per-joint modes, free joints, active flag |
| `POST /gravity/enable` | `{mode: "all"\|"joint", joint, confirm: "FREE"}` — **moves the arm** |
| `POST /gravity/hold` | `{confirm: "HOLD"}` — locks the current pose |
| `POST /gravity/idle` | arm joints to idle |

```bash
curl -s -X POST http://127.0.0.1:5001/connect \
  -H 'Content-Type: application/json' \
  -d '{"ip":"192.168.1.2","ee":"follower"}'

curl -s http://127.0.0.1:5001/live

# free J3 only (SDK joint 2) -- support the arm first
curl -s -X POST http://127.0.0.1:5001/gravity/enable \
  -H 'Content-Type: application/json' \
  -d '{"mode":"joint","joint":2,"confirm":"FREE"}'

# lock it again
curl -s -X POST http://127.0.0.1:5001/gravity/hold \
  -H 'Content-Type: application/json' -d '{"confirm":"HOLD"}'
```

Connection failures come back as JSON (`{"ok": false, "error": "..."}`) and
leave the server running — a bad IP or a yanked cable will not take the page
down. The gravity routes reject a missing or wrong confirmation, an unknown mode,
and any attempt to free the gripper or an out-of-range joint.
