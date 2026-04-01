# raw rangefinder / optical flow topics over uXRCE-DDS

PX4 only exposes the topics that are listed in its `dds_topics.yaml` file.

If you want raw rangefinder and raw optical flow visible to ROS 2, add entries like this to the PX4 `publications:` list in:

```text
PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

Example:

```yaml
publications:
  - topic: /fmu/out/distance_sensor
    type: px4_msgs::msg::DistanceSensor
    rate_limit: 30.

  - topic: /fmu/out/sensor_optical_flow
    type: px4_msgs::msg::SensorOpticalFlow
    rate_limit: 30.
```

After that:

1. rebuild PX4 firmware
2. flash it
3. make sure your `px4_msgs` branch still matches the firmware release
4. restart the uXRCE-DDS client and agent

If you do not add these raw topics, this repo still works with fused telemetry from:

- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status`
