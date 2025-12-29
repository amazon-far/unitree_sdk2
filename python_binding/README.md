# Unitree Interface Python Binding

This is a universal Unitree robot Python interface based on pybind11, supporting multiple robot types (G1, H1, H1-2) and message formats (HG, GO2).

## Features

- **Multi-Robot Support**: Supports multiple robot types including G1, H1, H1-2
- **Multiple Message Formats**: Supports HG and GO2 message formats
- **Real-time Control**: Supports 500Hz real-time control loop
- **Data Reading**: Read robot state (IMU, motor state, etc.) and wireless controller input
- **Command Sending**: Send motor control commands to the robot
- **Dual Control Modes**: Supports PR (Pitch/Roll) and AB (A/B) control modes
- **Type Safety**: Provides complete Python type hints (.pyi files)
- **Thread Safety**: Uses buffer mechanism to ensure thread-safe data exchange
- **Factory Methods**: Provides convenient robot creation methods

## Supported Robot Types

| Robot Type | Number of Motors | Default Message Format | Description |
|-----------|------------------|----------------------|-------------|
| G1        | 29               | HG                   | G1 Humanoid Robot |
| H1        | 19               | GO2                  | H1 Humanoid Robot |
| H1-2      | 29               | HG                   | H1-2 Humanoid Robot |
| CUSTOM    | Custom           | HG                   | Custom Robot Configuration |

## Build

### Build from Main Project (Recommended)

Execute in the main project root directory:

```bash
mkdir build
cd build
cmake -DBUILD_PYTHON_BINDING=ON -DCMAKE_BUILD_TYPE=Release .. 
make -j$(nproc)
```

### Standalone Build

In the `python_binding` directory:

```bash
./build.sh --sdk-path /opt/unitree_sdk2
```

## System Requirements

- Ubuntu 18.04/20.04/22.04 or compatible system
- Python 3.6+
- CMake 3.12+
- GCC 7+ or Clang 6+
- Unitree SDK2
- pybind11

## Install Dependencies

```bash
# Install basic dependencies
sudo apt-get update
sudo apt-get install build-essential cmake python3-dev python3-pip

# Install Python dependencies
pip3 install pybind11 pybind11-stubgen numpy
```

## Usage

### Basic Usage

```python
import unitree_interface

# Method 1: Create robot using factory method
robot = unitree_interface.create_robot("eth0", unitree_interface.RobotType.G1)

# Method 2: Create interface directly
robot = unitree_interface.UnitreeInterface("eth0", unitree_interface.RobotType.H1, unitree_interface.MessageType.GO2)

# Method 3: Use predefined configuration
robot = unitree_interface.UnitreeInterface("eth0", unitree_interface.RobotConfigs.G1_HG)

# Read robot state
state = robot.read_low_state()
print(f"IMU RPY: {state.imu.rpy}")
print(f"Joint positions: {state.motor.q}")

# Read wireless controller state
controller = robot.read_wireless_controller()
print(f"Left stick: {controller.left_stick}")

# Create zero position command
cmd = robot.create_zero_command()

# Set target position (adjust joint index according to robot type)
num_motors = robot.get_num_motors()
if num_motors > 4:
    cmd.q_target[4] = 0.1  # Left ankle pitch
if num_motors > 5:
    cmd.q_target[5] = 0.0  # Left ankle roll

# Send command to robot
robot.write_low_command(cmd)
```

### General Interface Example

Run the provided general interface example:

```bash
# G1 robot example
python3 example_general_interface.py eth0 G1

# H1 robot example
python3 example_general_interface.py eth0 H1 GO2

# H1-2 robot example
python3 example_general_interface.py eth0 H1_2
```

This example demonstrates:
1. Robot moves to zero position (3 seconds)
2. Joint swing demonstration (continuous)
3. Controller manual control (press A button)

## API Reference

### Main Classes

#### `UnitreeInterface`

Main robot control interface class.

```python
def __init__(self, network_interface: str, robot_type: RobotType, message_type: MessageType = MessageType.HG) -> None
def __init__(self, network_interface: str, config: RobotConfig) -> None
def __init__(self, network_interface: str, robot_type: RobotType, message_type: MessageType, num_motors: int) -> None

def read_low_state(self) -> LowState
def read_wireless_controller(self) -> WirelessController
def write_low_command(self, command: MotorCommand) -> None
def set_control_mode(self, mode: ControlMode) -> None
def get_control_mode(self) -> ControlMode
def create_zero_command(self) -> MotorCommand
def get_default_kp(self) -> List[float]
def get_default_kd(self) -> List[float]
def get_config(self) -> RobotConfig
def get_num_motors(self) -> int
def get_robot_name(self) -> str
```

### Factory Methods

```python
# Convenient robot creation methods
unitree_interface.create_g1(network_interface: str, message_type: MessageType = MessageType.HG) -> UnitreeInterface
unitree_interface.create_h1(network_interface: str, message_type: MessageType = MessageType.GO2) -> UnitreeInterface
unitree_interface.create_h1_2(network_interface: str, message_type: MessageType = MessageType.HG) -> UnitreeInterface
unitree_interface.create_custom(network_interface: str, num_motors: int, message_type: MessageType = MessageType.HG) -> UnitreeInterface

# General creation method
unitree_interface.create_robot(network_interface: str, robot_type: RobotType, message_type: MessageType = MessageType.HG) -> UnitreeInterface
```

### Enumeration Types

```python
class RobotType(Enum):
    G1 = 0      # G1 Humanoid Robot (29 motors)
    H1 = 1      # H1 Humanoid Robot (19 motors)
    H1_2 = 2    # H1-2 Humanoid Robot (29 motors)
    CUSTOM = 99 # Custom Robot

class MessageType(Enum):
    HG = 0   # Humanoid/Go1 message format
    GO2 = 1  # Go2 message format

class ControlMode(Enum):
    PR = 0  # Pitch/Roll mode
    AB = 1  # A/B mode
```

### Predefined Configurations

```python
unitree_interface.RobotConfigs.G1_HG    # G1 + HG message
unitree_interface.RobotConfigs.H1_GO2   # H1 + GO2 message
unitree_interface.RobotConfigs.H1_2_HG  # H1-2 + HG message
```

### Data Structures

```python
class LowState:
    imu: ImuState           # IMU state
    motor: MotorState       # Motor state
    mode_machine: int       # Robot mode machine state

class MotorState:
    q: List[float]          # Joint positions [rad]
    dq: List[float]         # Joint velocities [rad/s]
    tau_est: List[float]    # Estimated joint torques [N*m]
    temperature: List[int]  # Motor temperatures [°C]
    voltage: List[float]    # Motor voltages [V]

class MotorCommand:
    q_target: List[float]   # Target joint positions [rad]
    dq_target: List[float]  # Target joint velocities [rad/s]
    kp: List[float]         # Position gains
    kd: List[float]         # Velocity gains
    tau_ff: List[float]     # Feedforward torques [N*m]

class WirelessController:
    left_stick: List[float]  # Left joystick [x, y]
    right_stick: List[float] # Right joystick [x, y]
    A: bool                  # A button
    B: bool                  # B button
    X: bool                  # X button
    Y: bool                  # Y button
    L1: bool                 # L1 button
    L2: bool                 # L2 button
    R1: bool                 # R1 button
    R2: bool                 # R2 button
```

## Safety Precautions

⚠️ **Important Safety Notes**:

- Ensure the robot is in a safe environment before running any control program
- Always have an emergency stop button ready
- Use smaller motion amplitudes when testing new control algorithms
- Monitor joint temperatures and voltages
- Use the B button on the wireless controller as an emergency stop
- Ensure the correct message format is used (HG vs GO2)

## Troubleshooting

### Build Errors

1. **Unitree SDK not found**:
   ```
   CMake Error: UNITREE_SDK not found
   ```
   Solution: Ensure building from the main project root directory, or check SDK path

2. **pybind11 not found**:
   ```
   CMake Error: pybind11 not found
   ```
   Solution: `pip install pybind11`

### Runtime Errors

1. **Module import failed**:
   ```python
   ImportError: No module named 'unitree_interface'
   ```
   Solution: Ensure the compiled `.so` file is in the Python path

2. **Network connection failed**:
   ```
   Error: Failed to initialize DDS
   ```
   Solution: Check if the network interface name is correct and if the robot is connected

3. **Message format mismatch**:
   ```
   Error: Message type mismatch
   ```
   Solution: Ensure the correct message format is used (H1 uses GO2, G1/H1-2 use HG)

### Robot Type Selection

- **G1**: 29 motors, uses HG message format
- **H1**: 19 motors, uses GO2 message format
- **H1-2**: 29 motors, uses HG message format

## License

Please refer to the Unitree SDK2 license terms.
