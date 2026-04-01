#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus
)
import time

class PX4Adapter(Node):
    def __init__(self):
        super().__init__('real_px4_adapter')

        self.offboard_mode = False
        self.armed = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL

        self.last_pose = TrajectorySetpoint()

        self.cmd_sub = self.create_subscription(
            PoseStamped,
            '/drone/cmd_pose',
            self.cmd_pose_cb,
            10)

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/drone/cmd_vel',
            self.cmd_vel_cb,
            10)

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb,
            10)

        self.timer = self.create_timer(0.1, self.publish_setpoint)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10)

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10)

        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10)

        self.get_logger().info('PX4 Adapter ready.')

    def vehicle_status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard_mode = self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def cmd_pose_cb(self, msg: PoseStamped):
        self.last_pose.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.last_pose.yaw = 0.0

    def cmd_vel_cb(self, msg: TwistStamped):
        pass

    def publish_setpoint(self):
        if self.offboard_mode and self.armed:
            offboard_msg = OffboardControlMode(
                timestamp=int(time.time() * 1e6),
                position=True,
                velocity=False,
                acceleration=False,
                attitude=False,
                body_rate=False
            )
            self.offboard_pub.publish(offboard_msg)

            traj_msg = TrajectorySetpoint(
                timestamp=int(time.time() * 1e6),
                position=self.last_pose.position,
                yaw=self.last_pose.yaw
            )
            self.setpoint_pub.publish(traj_msg)
        else:
            self.enter_offboard()

    def enter_offboard(self):
        if not self.offboard_mode:
            self.get_logger().info('Switching to offboard mode...')
            cmd = VehicleCommand(
                timestamp=int(time.time() * 1e6),
                param1=1.0,
                param2=6.0,
                command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                target_system=1,
                target_component=1,
                source_system=1,
                source_component=1,
                from_external=True
            )
            self.vehicle_cmd_pub.publish(cmd)

        if not self.armed:
            self.get_logger().info('Arming vehicle...')
            cmd_arm = VehicleCommand(
                timestamp=int(time.time() * 1e6),
                param1=1.0,
                command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                target_system=1,
                target_component=1,
                source_system=1,
                source_component=1,
                from_external=True
            )
            self.vehicle_cmd_pub.publish(cmd_arm)

def main(args=None):
    rclpy.init(args=args)
    px4_adapter = PX4Adapter()
    rclpy.spin(px4_adapter)
    px4_adapter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
