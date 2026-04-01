# voice_drone_safe_ws

This is a ROS 2 workspace for a Jetson Orin Nano + Pixhawk 6C + OAK-D style bench stack.

It gives you:

- typed commands from a console
- optional mic recording + OpenAI transcription
- OpenAI intent parsing into simple drone actions
- OAK-D RGB + depth publishing
- YOLO object detection on the Jetson
- a mission node that can do things like "go to coke can"
- PX4 uXRCE-DDS telemetry listeners
- a guarded bridge that **does not send live propulsion / takeoff / motion commands to a real aircraft**
- a fake drone node so the whole graph still runs end to end

## important

This build is for:

- bench testing
- SITL style logic testing
- camera and voice testing
- PX4 telemetry monitoring

This build is **not** a live autonomous flight stack. The real-motion layer is left blocked on purpose.

## folder layout

- `src/voice_drone` - ROS 2 python package
- `tools/get_px4_deps.sh` - clones `px4_msgs` into the workspace
- `tools/run_uxrce_agent_udp.sh` - starts the agent for SITL / UDP
- `tools/run_uxrce_agent_serial.sh` - starts the agent for a serial link to Pixhawk
- `notes/px4_raw_sensor_topics.md` - how to expose raw rangefinder / optical flow topics if you want them

## what the nodes do

- `console_node`  
  Reads terminal input. Normal text goes straight into the graph.  
  `/rec 4` records 4 seconds from the mic and sends it to OpenAI transcription.

- `intent_node`  
  Turns text into one JSON intent. It uses OpenAI if the key is there, else a simple local parser.

- `oak_node`  
  Reads RGB and depth from the OAK-D and publishes ROS images.

- `detect_node`  
  Runs YOLO on the RGB image and estimates distance from the depth image.

- `mission_node`  
  Keeps a tiny state machine. Examples:
  - `arm and takeoff`
  - `land`
  - `stop`
  - `go to coke can`
  - `turn right 45`
  - `move forward 2`

- `telemetry_node`  
  Listens to PX4 uXRCE-DDS topics like vehicle status and local position.

- `safe_bridge_node`  
  Guard layer. It blocks any idea of sending real takeoff / motion commands to a live quad.

- `fake_drone_node`  
  Small kinematic sim so the rest of the system still behaves like a full stack.

- `status_node`  
  Prints the important stuff to the terminal.

## tested shape of the graph

```text
terminal/mic -> intent -> mission -> safe_bridge -> fake_drone
                         ^         \
camera -> yolo ----------/          -> status prints
pixhawk telemetry ------------------> status prints
```

## install idea on the jetson

### 1) ROS 2 + PX4 side

Install ROS 2 on Ubuntu 22.04 first.

Then clone the matching PX4 message package into this workspace:

```bash
cd ~/voice_drone_safe_ws
./tools/get_px4_deps.sh
```

### 2) Python deps

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

If you want mic recording, make sure PortAudio is installed:

```bash
sudo apt update
sudo apt install -y portaudio19-dev python3-colcon-common-extensions \
    ros-humble-cv-bridge ros-humble-launch-ros
```

### 3) build

```bash
cd ~/voice_drone_safe_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## start the px4 agent

### for SITL / UDP

```bash
./tools/run_uxrce_agent_udp.sh
```

### for a serial link to a Pixhawk

Example:

```bash
./tools/run_uxrce_agent_serial.sh /dev/ttyACM0 921600
```

## run the full safe bench stack

```bash
cd ~/voice_drone_safe_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch voice_drone bench.launch.py
```

If you just want the console + parser + fake drone and no camera:

```bash
ros2 launch voice_drone text_only.launch.py
```

## commands

Type plain text:

```text
arm and takeoff
land
stop
go to coke can
turn right 30
move forward 2
status
```

Mic record command:

```text
/rec 4
```

Help:

```text
/help
```

Quit:

```text
/quit
```

## object names and "coke can"

The default YOLO model is generic. A plain `yolov8n.pt` model may call a coke can `bottle` or `cup`, or it may miss it depending on angle and distance.

This repo already maps a few aliases so `coke can` can match detections like:

- `bottle`
- `cup`
- `can`
- `soda can`

For better real results, point `YOLO_MODEL` at your own custom weights.

## px4 topic notes

The telemetry node subscribes to:

- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/distance_sensor` (if bridged)
- `/fmu/out/sensor_optical_flow` (if bridged)

A lot of PX4 builds do **not** expose the raw distance / optical flow topics by default over uXRCE-DDS.  
If you want those raw topics, check `notes/px4_raw_sensor_topics.md`.

## launch files

- `bench.launch.py` - console + camera + detection + telemetry + fake drone
- `text_only.launch.py` - console + parser + fake drone only

## files that matter most

- `src/voice_drone/voice_drone/common.py`
- `src/voice_drone/voice_drone/console_node.py`
- `src/voice_drone/voice_drone/intent_node.py`
- `src/voice_drone/voice_drone/oak_node.py`
- `src/voice_drone/voice_drone/detect_node.py`
- `src/voice_drone/voice_drone/mission_node.py`
- `src/voice_drone/voice_drone/telemetry_node.py`
- `src/voice_drone/voice_drone/safe_bridge_node.py`
- `src/voice_drone/voice_drone/fake_drone_node.py`

## real flight note

There is a `real_px4_adapter_template.py` file in the package. It is just a placeholder and raises errors on purpose.
