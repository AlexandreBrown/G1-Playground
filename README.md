# G1 Playground

A clean, minimal starting point for learning to control the Unitree G1 humanoid robot. The repo wraps the low-level Unitree SDK behind a small strategy-pattern interface so you can focus on writing policies instead of plumbing DDS topics, motor mode bytes, and threading.

The goal is to give you a fast path from "sim works" to "real robot works" without inheriting the weight of a research codebase.

## What's included

- A `UnitreeG1Robot` class that handles DDS plumbing, the motion-service handshake, mode_machine matching, the 500Hz control loop, and shutdown safety.
- A `Policy` abstract base class. Subclass it, implement `reset` and `step`, and the robot runs your policy at whatever rate you specify.
- Body-only mode (29 motors) and body + Dex3 hand mode (43 motors), switched by a single constructor argument.
- An `AnkleSwingPolicy` example that recreates Unitree's official low-level demo through the new architecture, useful as a sanity check end-to-end.
- A two-thread design: a policy thread at your chosen rate and a tracker thread at 500Hz. Plug in slow learned policies (10-50Hz) without losing motor command rate.

## Repository structure

```
G1-Playground/
├── scripts/
│   └── ankle_swing.py              # entry point that runs the demo
├── src/g1_playground/
│   ├── __init__.py
│   ├── action.py                   # JointAction dataclass + Mode constants
│   ├── dds.py                      # DDS channel initialization
│   ├── robot.py                    # UnitreeG1Robot + joint indices + gain defaults
│   ├── state.py                    # G1State dataclass (body + hand state bundle)
│   └── policies/
│       ├── __init__.py
│       ├── policy.py               # Policy abstract base class
│       └── ankle_swing.py          # example policy
├── pyproject.toml
├── uv.lock
└── README.md
```

## Setup

### 1. Install `uv`

This project uses [uv](https://docs.astral.sh/uv/) for Python dependency management. If you don't have it:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 2. Clone the project

```bash
cd ~
git clone https://github.com/AlexandreBrown/G1-Playground.git
cd G1-Playground
```

### 3. Clone and link `unitree_sdk2_python`

The upstream `unitree_sdk2_python` ships precompiled `.so` files that get stripped out when uv builds a wheel from a git source. The fix is an editable install pointing at a local clone:

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git

cd ~/G1-Playground
uv add --editable ~/unitree_sdk2_python
```

If you ever hit `Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH` during install, install the dev libs:

```bash
sudo apt install libcyclonedds-dev
```

### 4. Install everything else

```bash
uv sync
```

This installs `mujoco`, `numpy`, and any other declared dependencies into `.venv/`.

### 5. Verify the install

```bash
uv run python -c "from g1_playground.robot import UnitreeG1Robot; from g1_playground.policies.ankle_swing import AnkleSwingPolicy; print('ok')"
```

Should print `ok` with no traceback.

## Simulation setup (MuJoCo)

Start in simulation. Always. Real hardware has consequences and the sim catches most bugs.

### 1. Install MuJoCo

Download the latest MuJoCo release from https://github.com/google-deepmind/mujoco/releases and extract it to `~/.mujoco/`:

```bash
mkdir -p ~/.mujoco
cd ~/.mujoco
# download mujoco-3.3.x-linux-x86_64.tar.gz from the releases page
tar -xf mujoco-3.3.x-linux-x86_64.tar.gz
```

### 2. Clone `unitree_mujoco`

This is Unitree's official MuJoCo simulator. It runs as a separate process and communicates with your control code over DDS, exactly as the real robot does.

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```

### 3. Configure `unitree_mujoco` for G1

Edit `~/unitree_mujoco/simulate_python/config.py`:

```python
import os

ROBOT = "g1"
ROBOT_SCENE = os.path.expanduser("~/unitree_mujoco/unitree_robots/g1/scene_29dof.xml")
DOMAIN_ID = 0
INTERFACE = "lo"

USE_JOYSTICK = 0
JOYSTICK_TYPE = "xbox"
JOYSTICK_DEVICE = 0

PRINT_SCENE_INFORMATION = True
ENABLE_ELASTIC_BAND = True  # virtual strap so the robot doesn't collapse

SIMULATE_DT = 0.002
VIEWER_DT = 0.02
```

The important settings:

- `DOMAIN_ID = 0` matches the script's DDS domain. If they differ, the two processes can't see each other and you'll get `No LowState received within timeout`.
- `INTERFACE = "lo"` uses loopback so sim and script can talk on the same machine.
- `ENABLE_ELASTIC_BAND = True` hangs the robot from a virtual strap. The `AnkleSwingPolicy` ramps to zero pose, which is not a stable standing posture, so without the strap the robot collapses.

### 4. Run the simulator

Terminal 1:

```bash
cd ~/G1-Playground
uv run python ~/unitree_mujoco/simulate_python/unitree_mujoco.py
```

A MuJoCo viewer window opens with G1 loaded. Click the window once to focus it, then press **9** to engage the elastic band so the robot hangs in the air.

### 5. Run your control script

Terminal 2:

```bash
cd ~/G1-Playground
uv run python scripts/ankle_swing.py lo
```

Where `lo` is the network interface (loopback for sim). You should see:

1. Warning message and an Enter prompt.
2. Press Enter.
3. Within ~1 second: "Running. Press Ctrl+C to stop."
4. In the sim window: robot ramps to zero pose (3 seconds), then ankles swing in PR mode (3 seconds), then AB mode with wrist roll.

Stop with Ctrl+C. The robot's `stop()` releases hands if present and the threads die on process exit.

### Common simulation problems

| Symptom | Cause | Fix |
|---|---|---|
| `No LowState received within timeout` | Domain ID or interface mismatch between sim and script | Verify `DOMAIN_ID = 0` and `INTERFACE = "lo"` in `config.py`, and pass `lo` as the script argument |
| `ValueError: ParseXML: Error opening file '~/...'` | `~` not expanded in the config path string | Use `os.path.expanduser(...)` as shown above |
| Robot falls immediately and twitches on the ground | `ENABLE_ELASTIC_BAND = False` (no virtual strap) | Set `True`, restart sim, press `9` after it loads |
| `crc_amd64.so: cannot open shared object file` | `unitree_sdk2py` installed as a non-editable wheel from git, which dropped the precompiled libs | Use editable install from a local clone (see setup step 3) |

## Real robot deployment

Hardware is unforgiving. Read this section in full before plugging anything in.

### Safety checklist (non-negotiable for first runs)

1. **Hang the robot from an overhead gantry or strap.** The `AnkleSwingPolicy` ramps to zero pose, which from a standing posture means the robot collapses to a crouch and falls. Standing self-balance is not implemented in this repo.
2. **Wireless controller and E-stop within reach.** Know which button it is before you start.
3. **Clear area.** No people, no objects, no cables in reach of the robot.
4. **Charged battery.** Low battery causes erratic behavior under load.
5. **Body-only first.** Default to `num_motors=29`. Do not attempt `num_motors=43` (Dex3 hands) until body-only is solid.

### Network setup

The G1 has an ethernet port for SDK communication. Connect it to your laptop's wired adapter.

Find the interface name:

```bash
ip a
```

Look for the wired adapter connected to the robot. Common names: `enp2s0`, `enp3s0`, `eth0`. Configure that adapter's IP to be in the same subnet as the robot. The default robot onboard PC IP is `192.168.123.161`.

Verify the connection:

```bash
ping 192.168.123.161
```

If ping fails, the network isn't set up correctly. Fix that before going further. The Unitree quick-start docs cover network configuration in detail.

### Robot preparation

1. Power on the G1. It boots through its startup sequence.
2. Enter Developer mode : **HOLD L2 + CLICK R2**
3. Go DAMP : **HOLD L2 + CLICK B** 
4. Go DEBUG POSE : **HOLD L2 + CLICK A**
5. Go DAMP : **HOLD L2 + CLICK B**  
**Do not** press L2 + UP for locked standing. Stay in damping mode for the first test.

### Run

```bash
cd ~/G1-Playground
uv run python scripts/ankle_swing.py enp2s0
```

Replace `enp2s0` with your actual ethernet interface.

What you should see:

1. Script prints the warning and waits for Enter.
2. Press Enter.
3. `_release_motion_mode` shuts down the onboard sport service. You may hear a small click from the robot or feel it briefly lose stiffness.
4. State arrives in milliseconds and the wait completes.
5. The 500Hz tracker starts publishing commands. The robot ramps to zero pose, then ankles swing, etc.

### Stop immediately if you see

- Unusual motor noise, vibration, or buzzing → likely a gain mismatch
- A joint jerking or moving unexpectedly during the ramp → `_initial_q` may be wrong
- Any joint hitting its mechanical limit → zero pose is outside the safe range for that joint configuration
- Motors getting hot → don't leave it running long-term with stiff gains

Ctrl+C the script. The robot's `stop()` injects a release action; the threads die when the process exits. Power off via the wireless controller.

### Tuning notes for hardware

Sim and hardware behave differently in two important ways:

1. **Hardware exposes timing jitter.** The Python control loop at 500Hz is near its CPU budget on a laptop. You may see a tick-tick sound during fast motion that wasn't audible in sim. Drop `control_dt` to `0.004` (250Hz) or run with `sudo chrt -f 80 uv run python ...` for realtime scheduling priority. Either fixes most of it.

2. **The default PD gains are starting points, not tuned values.** `ARMS_Kp = [40] * 7` is uniform across shoulder, elbow, and wrist, but the wrist has much lower inertia and benefits from softer `kp` and higher `kd` to avoid ringing. Override per joint in your policy:

```python
kp = DEFAULT_KP.copy()
kd = DEFAULT_KD.copy()
for idx in (G129DofJointIndex.LeftWristRoll, G129DofJointIndex.RightWristRoll):
    kp[idx] = 25.0
    kd[idx] = 2.0
return JointAction(q=q, kp=kp, kd=kd, mode_pr=Mode.AB)
```

### Going further

Once `AnkleSwingPolicy` runs cleanly on hardware, the next steps are:

1. **Write a `HoldPosePolicy`** that records initial pose in `reset` and returns it every step. Useful as a baseline before any learned policy.
2. **Test Dex3 hands incrementally.** Start with a `HoldHandsPolicy` (passive, commands current pose with low gains) before any policy that moves the hands. The mode encoding for the Dex3 `RIS_Mode_t` byte is in `robot.py:_encode_dex3_motor_mode`.
3. **Plug in a learned policy.** Subclass `Policy`, run inference in `step`, convert torch output to numpy at the boundary (do not run the policy at 500Hz, keep it at 10-50Hz). For smooth tracking at lower policy rates, add target interpolation in the control loop.
4. **Move from MuJoCo to Isaac Lab** for closer-to-reality contact dynamics. Will be added to this repo later.

## Architecture notes

A short explanation of why the code is structured the way it is.

**Strategy pattern.** `UnitreeG1Robot` knows nothing about specific behaviors. It owns DDS, threading, CRC, mode_machine handshake, and the motor command layout. The `Policy` knows nothing about DDS or threading; it consumes a `G1State` and returns a `JointAction`. They communicate through a single dataclass boundary, which makes policies trivial to unit-test without a robot.

**Two threads** The DDS subscriber callback updates state. The policy thread reads state and writes target actions. The control thread reads target actions and publishes commands. Splitting compute from publish further would add lock overhead for no benefit, since they run at the same rate.

**`G1State` bundles body and hand state.** When `num_motors=43`, hand states are populated; otherwise they are `None`. Policies that don't need hands simply ignore those fields.

**Mode encoding differs between body and Dex3 hands.** Body motors take `mode = 1` for enable. Dex3 motors take a packed `RIS_Mode_t` byte. This is handled in `_apply_hand_actions` and `_encode_dex3_motor_mode`. A subtle bug here is silent on hardware (hands just don't move), so verify carefully when first enabling Dex3.

**Stop is best-effort, not clean.** `RecurrentThread` in `unitree_sdk2py` has no shutdown API. `stop()` injects a release action so the hands go limp, waits 50ms for the tracker to publish it, then returns. The threads die when the process exits via `sys.exit(0)`.

## License

MIT.

## Acknowledgements

Built on top of [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) and [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco). The architecture borrows the Init/Start lifecycle pattern from Unitree's C++ examples.