# G1 Playground

A clean, minimal starting point for learning to control the Unitree G1 humanoid robot using the Unitree Python SDK. The repo decouples low-level DDS communication and policy execution so you can create new policy classes without worrying about DDS topics, motor mode bytes, or threading for quick testing of your policy in sim and real.

Write a policy once, validate it across multiple simulators (MuJoCo, Isaac Lab) and deploy to the real robot with no code changes. All communication goes through DDS, the same protocol the real robot uses, so sim and real share the same code path.

## What's included

- A `UnitreeG1Robot` class that handles DDS plumbing, the motion-service handshake, mode_machine matching, the 500Hz control loop, and shutdown safety.
- A `Policy` abstract base class. Subclass it, implement `reset` and `step`, and the robot runs your policy at whatever rate you specify.
- Body-only mode (29 motors) and body + Dex3 hand mode (43 motors), switched by a single constructor argument.
- An `AnkleSwingPolicy` example that recreates Unitree's official low-level demo through the new architecture, useful as a sanity check end-to-end.
- A two-thread design: a policy thread at your chosen rate and a control thread at 500Hz. Plug in slow learned policies (10-50Hz) without losing motor command rate.

## Repository structure

```
G1-Playground/
├── scripts/
│   └── run_ankle_swing_demo.py              # entry point that runs the demo
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
git clone https://github.com/AlexandreBrown/G1-Playground.git
cd G1-Playground
```

### 3. Clone and link `unitree_sdk2_python`

The upstream `unitree_sdk2_python` ships precompiled `.so` files that get stripped out when uv builds a wheel from a git source. The fix is an editable install pointing at a local clone next to this repo:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git ../unitree_sdk2_python
uv add --editable ../unitree_sdk2_python
```

### 4. Install everything and activate the environment

```bash
uv sync
source .venv/bin/activate
```

This installs `mujoco`, `numpy`, and any other declared dependencies into `.venv/`. The `source` command activates the virtual environment so you can use `python` directly instead of `uv run` for the rest of the session.

### 5. Verify the install

```bash
python -c "from g1_playground.robot import UnitreeG1Robot; from g1_playground.policies.ankle_swing import AnkleSwingPolicy; print('ok')"
```

Should print `ok` with no traceback.

## Simulation setup (MuJoCo)

Start in simulation. Always. Real hardware has consequences and the sim catches most bugs.

MuJoCo is already installed as a Python dependency (`mujoco` in `pyproject.toml`), so no manual download is needed.

### 1. Clone `unitree_mujoco`

This is Unitree's official MuJoCo simulator. It runs as a separate process and communicates with your control code over DDS, exactly as the real robot does. Clone it next to this repo:

```bash
git clone https://github.com/unitreerobotics/unitree_mujoco.git ../unitree_mujoco
```

### 2. Configure `unitree_mujoco` for G1

Edit `../unitree_mujoco/simulate_python/config.py`:

```python
import os

ROBOT = "g1"
ROBOT_SCENE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "unitree_robots", "g1", "scene_29dof.xml")
DOMAIN_ID = 1
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

- `DOMAIN_ID = 1` matches the script's default DDS channel ID. We use 1 for simulation so that running a script without explicit flags never accidentally sends commands to a real robot (which uses channel 0). If they differ, the two processes can't see each other and you'll get `No LowState received within timeout`.
- `INTERFACE = "lo"` uses loopback so sim and script can talk on the same machine.
- `ENABLE_ELASTIC_BAND = True` hangs the robot from a virtual strap. The `AnkleSwingPolicy` ramps to zero pose, which is not a stable standing posture, so without the strap the robot collapses.

### 3. Run the simulator
Terminal 1:

Ensure env is activate :  
```bash
source .venv/bin/activate
```

Run the sim :
```bash
python ../unitree_mujoco/simulate_python/unitree_mujoco.py
```

A MuJoCo viewer window opens with G1 loaded.

<img src="resources/mujoco_g1_example.png" height="400px">  

### 4. Run your control script

Terminal 2:  

Ensure env is activate :  
```bash
source .venv/bin/activate
```

Run your code : 
```bash
python scripts/run_ankle_swing_demo.py --network_interface lo --dds_channel_id 1
```   
You should see:

1. Warning message and an Enter prompt.
2. Press Enter.
3. Within ~1 second the policy will start running on the robot.  
4. In the sim window: robot ramps to zero pose (first 3 seconds), then ankles swing in PR mode (3 seconds), then AB mode with wrist roll.

Stop with Ctrl+C. The robot's `stop()` releases hands if present and the threads die on process exit.

### Common simulation problems

| Symptom | Cause | Fix |
|---|---|---|
| `No LowState received within timeout` | Domain ID or interface mismatch between sim and script | Verify `DOMAIN_ID = 1` and `INTERFACE = "lo"` in `config.py`, and pass `lo` as the script argument |
| Robot falls immediately and twitches on the ground | `ENABLE_ELASTIC_BAND = False` (no virtual strap) | Set `True`, restart sim, press `9` after it loads |
| `crc_amd64.so: cannot open shared object file` | `unitree_sdk2py` installed as a non-editable wheel from git, which dropped the precompiled libs | Use editable install from a local clone (see setup step 3) |

## Simulation setup (Isaac Lab)

Like `unitree_mujoco`, the `unitree_sim_isaaclab` simulator runs as a separate process and communicates with your control code over DDS, so the existing scripts work without modification.

**Requires an NVIDIA GPU** with recent drivers. If you don't have one, stick with the MuJoCo setup above.

### 1. Install Isaac Sim 5.1.0

Create a virtual environment with Python 3.11 and install Isaac Sim:

```bash
cd ../
uv venv unitree_sim_env --python 3.11
source unitree_sim_env/bin/activate

uv pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu126
uv pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

Verify by running `isaacsim` (first run will ask you to accept the EULA).

### 2. Install Isaac Lab

```bash
git clone https://github.com/isaac-sim/IsaacLab.git ./IsaacLab
sudo apt install cmake build-essential
cd ../IsaacLab
./isaaclab.sh --install
cd ../G1-Playground
```

Verify:

```bash
python ../IsaacLab/scripts/tutorials/00_sim/create_empty.py
```

It's gonna take a little while but you should end up seeing "[INFO]: Setup complete..." in the terminal.

### 3. Clone and set up `unitree_sim_isaaclab`

```bash
git clone https://github.com/unitreerobotics/unitree_sim_isaaclab.git ../unitree_sim_isaaclab
cd ../unitree_sim_isaaclab
git submodule update --init --depth 1
```

Build CycloneDDS from source (needed for the `unitree_sdk2_python` install):

```bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x ../cyclonedds
cd ../cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
cd ../../unitree_sim_isaaclab
```

Install its dependencies:

```bash
export CYCLONEDDS_HOME=$(realpath ../cyclonedds/install)
uv pip install wheel
CYCLONEDDS_HOME=$CYCLONEDDS_HOME uv pip install --no-build-isolation -e ../unitree_sdk2_python
uv pip install -e ./teleimager
uv pip install -r requirements.txt
```

Fetch the robot assets (requires `git-lfs`):

```bash
bash fetch_assets.sh
```

This downloads USDA/URDF models from HuggingFace and places them in `../assets/`.

Return to the project directory:

```bash
cd ../G1-Playground
```

### 4. Run the simulator

Both `unitree_sim_isaaclab` and `unitree_mujoco` use DDS channel **1**, which matches the script's default `--dds_channel_id`.

Terminal 1 (with the `unitree_sim_env` venv active):

```bash
source ../unitree_sim_env/bin/activate
cd ../unitree_sim_isaaclab
python sim_main.py --device cuda --enable_cameras --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint --enable_dex3_dds --robot_type g129
```

<img src="resources/isaac_lab_g1_example.png" height="400px"/>

Available G1 tasks:

| Task | Hand type | Flag |
|---|---|---|
| `Isaac-PickPlace-Cylinder-G129-Dex3-Joint` | Dex3 | `--enable_dex3_dds` |
| `Isaac-PickPlace-RedBlock-G129-Dex3-Joint` | Dex3 | `--enable_dex3_dds` |
| `Isaac-Stack-RgyBlock-G129-Dex3-Joint` | Dex3 | `--enable_dex3_dds` |
| `Isaac-Move-Cylinder-G129-Dex3-Wholebody` | Dex3 | `--enable_dex3_dds` |

Only one hand flag can be active at a time. `Joint` tasks fix the robot's base in place (good for arm/hand testing), while `Wholebody` tasks allow full locomotion. Use a `Wholebody` task when testing policies that move the legs. You can also register your own custom Isaac Lab tasks for deployment testing.

### 5. Run your control script

Terminal 2 (with the G1-Playground venv active):

```bash
source .venv/bin/activate
python scripts/run_ankle_swing_demo.py --dds_channel_id 1
```

No `--network_interface` is needed since Isaac Lab communicates via shared memory on the same machine.

### Common Isaac Lab problems

| Symptom | Cause | Fix |
|---|---|---|
| `No LowState received within timeout` | DDS channel mismatch | Ensure `--dds_channel_id` is 1 (the default) to match the Isaac Lab simulator |
| `libstdc++.so.6: version GLIBCXX_3.4.30 not found` | System libstdc++ too old for Isaac Sim | `sudo apt install libstdc++-12-dev` or update your distro's gcc/g++ package |
| EULA prompt blocks startup | First-time Isaac Sim launch | Run `isaacsim` once manually and accept the EULA |
| `Could not locate cyclonedds` | Missing CycloneDDS dev libs | `sudo apt install cyclonedds-dev` |

## Real robot deployment

Hardware is unforgiving. Read this section in full before plugging anything in.

### Safety checklist

1. **Hang the robot from an overhead gantry or strap.** The `AnkleSwingPolicy` ramps to zero pose, which from a standing posture means the robot collapses to a crouch and falls. Standing self-balance is not implemented in this repo.
2. **Wireless controller and E-stop within reach.** Know which button it is before you start.
3. **Clear area.** No people, no objects, no cables in reach of the robot.
4. **Charged battery.** Low battery causes erratic behavior under load.
5. **Body-only first.** Default to `num_motors=29`. Do not attempt `num_motors=43` (Dex3 hands) until body-only is solid.

### Network setup

The G1 has an ethernet port for SDK communication. Connect it to your laptop's wired adapter.

Find the interface name:

```bash
ifconfig
```

Look for the wired adapter connected to the robot. Common names: `enp2s0`, `enp3s0`, `eth0`. Configure that adapter's IP to be in the same subnet as the robot. The default robot onboard PC IP is `192.168.123.161`, this one is private and cannot be ssh-ed into and the dev computer on the robot is `192.168.123.164` (that one we can ssh into).

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
python scripts/run_ankle_swing_demo.py --network_interface enp2s0 --dds_channel_id 0
```

The real robot uses DDS channel 0 by default. Replace `enp2s0` with your actual ethernet interface.

What you should see:

1. Script prints the warning and waits for Enter.
2. Press Enter.
3. `_release_motion_mode` shuts down the onboard sport service. You may hear a small click from the robot or feel it briefly lose stiffness.
4. State arrives in milliseconds and the wait completes.
5. The 500Hz control thread starts publishing commands. The robot ramps to zero pose, then ankles swing, etc.

### Stop immediately if you see

- Unusual motor noise, vibration, or buzzing → likely a gain mismatch
- A joint jerking or moving unexpectedly during the ramp → `_initial_q` may be wrong
- Any joint hitting its mechanical limit → zero pose is outside the safe range for that joint configuration
- Motors getting hot → don't leave it running long-term with stiff gains

Ctrl+C the script. The robot's `stop()` injects a release action; the threads die when the process exits. Alternatively use an emergency stop if you have one attached to the G1.

### Tuning notes for hardware

Sim and hardware behave differently in two important ways:

1. **Hardware exposes timing jitter.** The Python control loop at 500Hz is near its CPU budget on a laptop. You may see a tick-tick sound during fast motion that wasn't audible in sim. Drop `control_dt` to `0.004` (250Hz) or run with `sudo chrt -f 80 python ...` for realtime scheduling priority. Either fixes most of it.

2. **The default PD gains are starting points, not tuned values.** `ARMS_Kp = [40] * 7` is uniform across shoulder, elbow, and wrist, but the wrist has much lower inertia and benefits from softer `kp` and higher `kd` to avoid ringing. Override per joint in your policy:

```python
kp = DEFAULT_KP.copy()
kd = DEFAULT_KD.copy()
for idx in (G129DofJointIndex.LeftWristRoll, G129DofJointIndex.RightWristRoll):
    kp[idx] = 25.0
    kd[idx] = 2.0
return JointAction(q=q, kp=kp, kd=kd, mode_pr=Mode.AB)
```

## Architecture notes

A short explanation of why the code is structured the way it is.

**Strategy pattern.** `UnitreeG1Robot` knows nothing about specific behaviors. It owns DDS, threading, CRC, mode_machine handshake, and the motor command layout. The `Policy` knows nothing about DDS or threading; it consumes a `G1State` and returns a `JointAction`. They communicate through a single dataclass boundary, which makes policies trivial to unit-test without a robot.

**Two threads** The DDS subscriber callback updates state. The policy thread reads state and writes target actions. The control thread reads target actions and publishes commands. Splitting compute from publish further would add lock overhead for no benefit, since they run at the same rate.

**`G1State` bundles body and hand state.** When `num_motors=43`, hand states are populated; otherwise they are `None`. Policies that don't need hands simply ignore those fields.

**Mode encoding differs between body and Dex3 hands.** Body motors take `mode = 1` for enable. Dex3 motors take a packed `RIS_Mode_t` byte. This is handled in `_apply_hand_actions` and `_encode_dex3_motor_mode`. A subtle bug here is silent on hardware (hands just don't move), so verify carefully when first enabling Dex3.

**Stop is best-effort, not clean.** `RecurrentThread` in `unitree_sdk2py` has no shutdown API. In our `stop()`, we inject a release action so the hands go limp, waits 50ms for the control thread to publish it, then returns. The threads die when the process exits via `sys.exit(0)`.

## License

MIT.

## Acknowledgements

Built on top of [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python), [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco), and [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab). The architecture borrows the Init/Start lifecycle pattern from Unitree's C++ examples.