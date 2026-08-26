# Introduction
## Pineapple mujoco
Contains pineapple v0 and v1 model

# Installation
## C++ Simulator (simulate)
### 1. Dependencies
#### unitree_sdk2
It is recommended to install `unitree_sdk2` in `/opt/unitree_robotics` path.
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```
For more details, see: https://github.com/unitreerobotics/unitree_sdk2
#### mujoco
Current version is tested in mujoco-3.2.7
```bash
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev
```
```bash
git clone https://github.com/google-deepmind/mujoco.git
mkdir build && cd build
cmake ..
make -j4
sudo make install
```
Test:
```bash
simulate
```
If the mujoco simulator pops up, the installation is successful.
#### yaml-cpp
yaml-cpp is mainly used for reading configuration files:
```bash
sudo apt install libyaml-cpp-dev
```
### 2. Compile unitree_mujoco
```bash
cd unitree_mujoco/simulate
mkdir build && cd build
cmake ..
make -j4
```
### 3. Test:
Run:
```bash
./unitree_mujoco
```
You should see the mujoco simulator with the Go2 robot loaded.
In a new terminal, run:
```bash
./test
```
The program will output the robot's pose and position information in the simulator, and each motor of the robot will continuously output 1Nm of torque.

**Note:** The testing program sends the unitree_go message. If you want to test G1 robot, you need to modify the program to use the unitree_hg message.

## Python Simulator (simulate_python)
### 0. Conda environment (recommended)
Section 1 below is upstream's system-wide `pip3` install. This fork is developed against a
conda env named `pineapple_mujoco`, shared with `pineapple_rl_deploy` so that one env runs both
the simulator and the controller that drives it over DDS.

```bash
conda create -n pineapple_mujoco python=3.10 -y
conda activate pineapple_mujoco

# From the pineapple_mujoco directory. Brings cyclonedds==0.10.2, numpy and opencv-python
# with it -- see that repo's setup.py. Clone it first if you have not:
#   git clone https://github.com/unitreerobotics/unitree_sdk2_python.git ../unitree_sdk2_python
pip install -e ../unitree_sdk2_python

# mujoco pulls in glfw, pyopengl and etils.
pip install mujoco pygame pyyaml

# Only for running pineapple_rl_deploy in this same env; the simulator itself never imports it.
pip install onnxruntime
```

Two details are load-bearing:

- **Python 3.10.** `unitree_sdk2py` pins `cyclonedds==0.10.2`, and that version ships a binary
  wheel with `libddsc` bundled inside it. That wheel is what makes the `CYCLONEDDS_HOME` source
  build described further down unnecessary -- and it is a `cp310` wheel, so the interpreter
  version is not free choice. If you see "Could not locate cyclonedds", you are on a Python the
  wheel does not cover and have fallen back to building it from source.
- **`-e` on the SDK.** The DDS IDL message types are imported by dotted path
  (`unitree_sdk2py.idl.geometry_msgs.msg.dds_.TwistStamped_` and friends) from configs in both
  repos, so an editable install keeps the checkout the single copy rather than a stale one.

Known-good versions, as installed: python 3.10.20, mujoco 3.12.0, numpy 2.2.6, cyclonedds
0.10.2, onnxruntime 1.23.2, pygame 2.6.1, PyYAML 6.0.3, opencv-python 5.0.0.

`terrain_tool/terrain_generator.py` additionally needs `pip install noise` (opencv arrives with
the SDK). Nothing else imports it, so skip it unless you are generating terrain.

#### Check the env is actually self-contained

Conda envs here share Python 3.10 with the system interpreter, so anything sitting in
`~/.local/lib/python3.10/site-packages` is importable from inside the env and hides the fact
that it was never installed into it. The env then works on this machine and fails on the robot.
To check:

```bash
PYTHONNOUSERSITE=1 python -c "import mujoco, numpy, pygame, yaml, cyclonedds, unitree_sdk2py"
```

If that fails while the same import without the variable succeeds, the package it names is
coming from the user site. `pip install` it again with the env activated.

#### Run it

```bash
conda activate pineapple_mujoco
cd simulate_python
python unitree_mujoco.py
```

`config.py` picks the robot (`ROBOT = "pineapple_v3"`), the DDS domain (`DOMAIN_ID = 1`) and the
network interface (`INTERFACE = "lo"`). All three have to match whatever controller you point at
it -- a domain or interface mismatch is silent, and looks like a simulator that simply never
receives a command.

### 1. Dependencies
#### unitree_sdk2_python
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```
If you encounter an error during installation:
```bash
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```
Refer to: https://github.com/unitreerobotics/unitree_sdk2_python
#### mujoco-python
```bash
pip3 install mujoco
```

#### joystick
```bash
pip3 install pygame
```

### 2. Test
```bash
cd ./simulate_python
python3 ./unitree_mujoco.py
```
You should see the mujoco simulator with the Go2 robot loaded.
In a new terminal, run:
```bash
python3 ./test/test_unitree_sdk2.py
```
The program will output the robot's pose and position information in the simulator, and each motor of the robot will continuously output 1Nm of torque.

**Note:** The testing program sends the unitree_go message. If you want to test G1 robot, you need to modify the program to use the unitree_hg message.


# Usage
## 1. Simulation Configuration
### C++ Simulator
The configuration file for the C++ simulator is located at `/simulate/config.yaml`:
```yaml
# Robot name loaded by the simulator
# "pineapple_v0", "pineapple_v1"
robot: "pineapple_v0"
# Robot simulation scene file
robot_scene: "scene.xml"
# DDS domain id, it is recommended to distinguish from the real robot (default is 0 on the real robot)
domain_id: 1

use_joystick: 1 # Simulate Unitree WirelessController using a gamepad
joystick_type: "xbox" # support "xbox" and "switch" gamepad layout
joystick_device: "/dev/input/js0" # Device path
joystick_bits: 16 # Some game controllers may only have 8-bit accuracy

# Network interface name, for simulation, it is recommended to use the local loopback "lo"
interface: "lo"
# Whether to output robot link, joint, sensor information, 1 for output
print_scene_information: 1
# Whether to use virtual tape, 1 to enable
# Mainly used to simulate the hanging process of H1 robot initialization
enable_elastic_band: 0 # For H1
```
### Python Simulator
The configuration file for the Python simulator is located at `/simulate_python/config.py`:
```python
# Robot name loaded by the simulator
# "pineapple_v0", "pineapple_v1"
ROBOT = "pineapple_v0"
# Robot simulation scene file
ROBOT_SCENE = "../pineapple_robots/" + ROBOT + "/scene.xml"  # Robot scene
# DDS domain id, it is recommended to distinguish from the real robot (default is 0 on the real robot)
DOMAIN_ID = 1  # Domain id
# Network interface name, for simulation, it is recommended to use the local loopback "lo"
INTERFACE = "lo"  # Interface
# Whether to output robot link, joint, sensor information, True for output
PRINT_SCENE_INFORMATION = True

USE_JOYSTICK = 1 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

# Whether to use virtual tape, 1 to enable
# Mainly used to simulate the hanging process of H1 robot initialization
ENABLE_ELASTIC_BAND = False
# Simulation time step (unit: s)
# To ensure the reliability of the simulation, it needs to be greater than the time required for viewer.sync() to render once
SIMULATE_DT = 0.003  
# Visualization interface runtime step, 0.02 corresponds to 50fps/s
VIEWER_DT = 0.02
```
### Joystick
The simulator will use an Xbox or Switch gamepad  to simulate the wireless controller of the robot. The button and joystick information of the wireless controller will be published through "rt/wireless_controller" topic. `use_joystick/USE_JOYSTICK` in `config.yaml/config.py` needs to be set to 0, when there is no gamepad. If your gamepad is not in Xbox or Switch layout, you can modify it in the source code (The button and joystick IDs can be  determined  using `jstest`):

In `simulate/src/unitree_sdk2_bridge/unitree_sdk2_bridge.cc`: 
```C++
 if (js_type == "xbox")
{
    js_id_.axis["LX"] = 0; // Left stick axis x
    js_id_.axis["LY"] = 1; // Left stick axis y
    js_id_.axis["RX"] = 3; // Right stick axis x
    js_id_.axis["RY"] = 4; // Right stick axis y
    js_id_.axis["LT"] = 2; // Left trigger
    js_id_.axis["RT"] = 5; // Right trigger
    js_id_.axis["DX"] = 6; // Directional pad x
    js_id_.axis["DY"] = 7; // Directional pad y

    js_id_.button["X"] = 2;
    js_id_.button["Y"] = 3;
    js_id_.button["B"] = 1;
    js_id_.button["A"] = 0;
    js_id_.button["LB"] = 4;
    js_id_.button["RB"] = 5;
    js_id_.button["SELECT"] = 6;
    js_id_.button["START"] = 7;
}
```

In `simulate_python/unitree_sdk2_bridge.py`: 
```python
if js_type == "xbox":
    self.axis_id = {
        "LX": 0,  # Left stick axis x
        "LY": 1,  # Left stick axis y
        "RX": 3,  # Right stick axis x
        "RY": 4,  # Right stick axis y
        "LT": 2,  # Left trigger
        "RT": 5,  # Right trigger
        "DX": 6,  # Directional pad x
        "DY": 7,  # Directional pad y
    }

    self.button_id = {
        "X": 2,
        "Y": 3,
        "B": 1,
        "A": 0,
        "LB": 4,
        "RB": 5,
        "SELECT": 6,
        "START": 7,
    }
```

## 2. Terrain Generation Tool
We provide a tool to parametrically create simple terrains in the mujoco simulator, including stairs, rough ground, and height maps. The program is located in the `terrain_tool` folder. For specific usage instructions, refer to the README file in the `terrain_tool` folder.
![Terrain Generation Example](./doc/terrain.png)

