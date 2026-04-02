import rclpy
from rclpy.node import Node
import math
import pandas as pd
from datetime import datetime, timezone
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32


class TRC_Log_Maker(Node):

    def __init__(self):
        super().__init__('TRC_log_maker')

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10
            )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
            )

        self.aruco_subscription = self.create_subscription(
            MarkerArray,
            '/aruco/id',
            self.aruco_callback,
            10
            )
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
            )

        self.imu_yaw_subscription = self.create_subscription(
            Float32,
            'imu/yaw',
            self.imu_yaw_callback,
            10
            )
        
        self.arucoid_gps = pd.read_csv("arucoid_gps_B.csv", index_col=0)
        print(self.arucoid_gps)
        self.checkpoints = pd.read_csv("checkpoint_number.csv")

        self.discriminant_history = [1,1]

        self.log_filename = "TRC_log_test.csv"

        self.column_names = [
            "timestamp",
            "latitude",
            "longitude",
            "imu_orientation_x",
            "imu_orientation_y",
            "imu_orientation_z",
            "imu_orientation_w",
            "imu_angular_velocity_x",
            "imu_angular_velocity_y",
            "imu_angular_velocity_z",
            "imu_linear_acceleration_x",
            "imu_linear_acceleration_y",
            "imu_linear_acceleration_z",
            "aruco_marker_id",
            "cmd_angular_z_deg",
            "yaw_deg",
            "checkpoint_id",
            "discriminant"
        ]

        self.timestamp = 0
        self.latitude = 0
        self.longitude = 0
        self.imu_orientation_x =0
        self.imu_orientation_y = 0
        self.imu_orientation_z = 0
        self.imu_orientation_w = 0
        self.imu_angular_velocity_x = 0   
        self.imu_angular_velocity_y = 0
        self.imu_angular_velocity_z = 0
        self.imu_linear_acceleration_x = 0
        self.imu_linear_acceleration_y = 0
        self.imu_linear_acceleration_z = 0
        self.aruco_marker_id = 0
        self.cmd_angular_z_deg = 0
        self.yaw_deg = 0
        self.checkpoint_id = None
        self.discriminant =0
        self.discriminant_sign = 0
        self.passed_id = None

        pd.DataFrame([[0]*len(self.column_names)], columns=self.column_names).to_csv(self.log_filename, mode='x', index=False)


        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def checkpoint_calculate(self, latitude, longitude, checkpoint_id):
        aruco1 = self.checkpoints.loc[checkpoint_id, 'aruco1']
        aruco2 = self.checkpoints.loc[checkpoint_id, 'aruco2']
        aruco1_latitude = self.arucoid_gps.loc[aruco1, 'latitude_north']
        aruco1_longitude = self.arucoid_gps.loc[aruco1, 'longitude_east']
        aruco2_latitude = self.arucoid_gps.loc[aruco2, 'latitude_north']
        aruco2_longitude = self.arucoid_gps.loc[aruco2, 'longitude_east']

        self.discriminant = (aruco2_latitude - aruco1_latitude) / (aruco2_longitude - aruco1_longitude) * (longitude - aruco1_longitude) - (latitude - aruco1_latitude)
        if self.discriminant < 0:
            self.discriminant_sign = -10
        else:
            self.discriminant_sign = 10
       
        self.discriminant_history[1] = self.discriminant_history[0]
        self.discriminant_history[0] = self.discriminant_sign
        if self.discriminant_history[0] * self.discriminant_history[1] < 0:
            self.get_logger().info(f"Checkpoint {checkpoint_id} passed")
            self.passed_id = checkpoint_id
        else:
            self.passed_id = None
        


    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        #self.get_logger().info(f"GPS data: {self.latitude}, {self.longitude}")
        for id in range(10):
            self.checkpoint_calculate(self.latitude, self.longitude, id)
            if self.passed_id is not None:
                break
    
    def imu_callback(self, msg):
        self.imu_orientation_x = msg.orientation.x
        self.imu_orientation_y = msg.orientation.y
        self.imu_orientation_z = msg.orientation.z
        self.imu_orientation_w = msg.orientation.w
        self.imu_angular_velocity_x = msg.angular_velocity.x
        self.imu_angular_velocity_y = msg.angular_velocity.y
        self.imu_angular_velocity_z = msg.angular_velocity.z
        self.imu_linear_acceleration_x = msg.linear_acceleration.x
        self.imu_linear_acceleration_y = msg.linear_acceleration.y
        self.imu_linear_acceleration_z = msg.linear_acceleration.z
        #self.get_logger().info(f"IMU data: {self.imu_orientation}, {self.imu_angular_velocity}, {self.imu_linear_acceleration}")
    
    def aruco_callback(self, msg):
        self.aruco_marker_id = msg.markers[0].id
    
    def cmd_vel_callback(self, msg):
        self.cmd_angular_z = msg.angular.z
        self.cmd_angular_z_deg = self.cmd_angular_z * 180.0 / math.pi -self.yaw_deg
        if self.cmd_angular_z_deg < -180.0:
            self.cmd_angular_z_deg += 360.0
        if self.cmd_angular_z_deg > 180.0:
            self.cmd_angular_z_deg -= 360.0
        #self.get_logger().info(f"CMD_VEL angular_z: {self.cmd_angular_z}")
    
    def imu_yaw_callback(self, msg):
        self.yaw_rad = msg.data
        self.yaw_deg = self.yaw_rad * 180.0 / math.pi - 90.0
        if self.yaw_deg < -180.0:
            self.yaw_deg += 360.0
        #self.get_logger().info(f"IMU yaw: {self.yaw_deg}")

    def timer_callback(self):
        now = self.get_clock().now()
        sec = now.nanoseconds * 1e-9
        self.timestamp = datetime.fromtimestamp(sec, tz=timezone.utc).strftime('%Y-%m-%d-%H:%M:%S')

        df = pd.DataFrame(
                [[
                    self.timestamp,
                    self.latitude,
                    self.longitude,
                    self.imu_orientation_x,
                    self.imu_orientation_y,
                    self.imu_orientation_z,
                    self.imu_orientation_w,
                    self.imu_angular_velocity_x,
                    self.imu_angular_velocity_y,
                    self.imu_angular_velocity_z,
                    self.imu_linear_acceleration_x,
                    self.imu_linear_acceleration_y,
                    self.imu_linear_acceleration_z,
                    self.aruco_marker_id,
                    self.cmd_angular_z_deg,
                    self.yaw_deg,
                    self.passed_id,
                    self.discriminant_sign
                ]],
                columns = self.column_names
            )
        df.to_csv(self.log_filename, mode='a', header=False, index=False)


def main(args=None):
    rclpy.init(args=args)

    TRC_log_maker = TRC_Log_Maker()

    rclpy.spin(TRC_log_maker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    TRC_log_maker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()