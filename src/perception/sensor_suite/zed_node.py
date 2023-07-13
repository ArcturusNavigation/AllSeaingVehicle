import pyzed.sl as sl
import rospy 

from geometry_msgs.msg import PoseStamped

class ZedNode:
    def __init__(self):
        self.pose_pub = rospy.Publisher("/zed/pose", PoseStamped, queue_size=1)
        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode (default fps: 60)
        # Use a right-handed Y-up coordinate system
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        self.init_params.coordinate_units = sl.UNIT.METER  # Set units in meters

        # Open the camera
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            rospy.logerr(f"Failed to open ZED camera:{err}")
            # exit(1)
        
        zed_serial = self.zed.get_camera_information().serial_number
        print(zed_serial)

        # Enable positional tracking with default parameters
        self.py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
        self.tracking_parameters = sl.PositionalTrackingParameters(_init_pos=self.py_transform)
        err = self.zed.enable_positional_tracking(self.tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            rospy.logerr("Failed to enable positional tracking")
            # exit(1)
        self.zed_pose = sl.Pose()
        self.zed_sensors = sl.SensorsData()
        self.runtime_parameters = sl.RuntimeParameters()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Get the pose of the left eye of the camera with reference to the world frame
                self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
                self.zed.get_sensors_data(self.zed_sensors, sl.TIME_REFERENCE.IMAGE)
                self.zed_imu = self.zed_sensors.get_imu_data()

                # Display the translation and timestamp
                py_translation = sl.Translation()
                tx = round(self.zed_pose.get_translation(py_translation).get()[0], 3)
                ty = round(self.zed_pose.get_translation(py_translation).get()[1], 3)
                tz = round(self.zed_pose.get_translation(py_translation).get()[2], 3)
                rospy.loginfo("Translation: Tx: {0}, Ty: {1}, Tz {2}, Timestamp: {3}\n".format(tx, ty, tz, self.zed_pose.timestamp.get_milliseconds()))

                # Display the orientation quaternion
                py_orientation = sl.Orientation()
                ox = round(self.zed_pose.get_orientation(py_orientation).get()[0], 3)
                oy = round(self.zed_pose.get_orientation(py_orientation).get()[1], 3)
                oz = round(self.zed_pose.get_orientation(py_orientation).get()[2], 3)
                ow = round(self.zed_pose.get_orientation(py_orientation).get()[3], 3)
                rospy.loginfo("Orientation: Ox: {0}, Oy: {1}, Oz {2}, Ow: {3}\n".format(ox, oy, oz, ow))
                
                self.pose = PoseStamped()
                self.pose.header.stamp = rospy.Time.now()
                self.pose.header.frame_id = "zed_camera_center"
                self.pose.pose.position.x = tx
                self.pose.pose.position.y = ty
                self.pose.pose.position.z = tz
                self.pose.pose.orientation.x = ox
                self.pose.pose.orientation.y = oy
                self.pose.pose.orientation.z = oz
                self.pose.pose.orientation.w = ow
                self.pose_pub.publish(self.pose)

                #Display the IMU acceleratoin
                acceleration = [0,0,0]
                self.zed_imu.get_linear_acceleration(acceleration)
                ax = round(acceleration[0], 3)
                ay = round(acceleration[1], 3)
                az = round(acceleration[2], 3)
                rospy.loginfo("IMU Acceleration: Ax: {0}, Ay: {1}, Az {2}\n".format(ax, ay, az))
                
                #Display the IMU angular velocity
                a_velocity = [0,0,0]
                self.zed_imu.get_angular_velocity(a_velocity)
                vx = round(a_velocity[0], 3)
                vy = round(a_velocity[1], 3)
                vz = round(a_velocity[2], 3)
                rospy.loginfo("IMU Angular Velocity: Vx: {0}, Vy: {1}, Vz {2}\n".format(vx, vy, vz))

                # Display the IMU orientation quaternion
                self.zed_imu_pose = sl.Transform()
                ox = round(self.zed_imu.get_pose(self.zed_imu_pose).get_orientation().get()[0], 3)
                oy = round(self.zed_imu.get_pose(self.zed_imu_pose).get_orientation().get()[1], 3)
                oz = round(self.zed_imu.get_pose(self.zed_imu_pose).get_orientation().get()[2], 3)
                ow = round(self.zed_imu.get_pose(self.zed_imu_pose).get_orientation().get()[3], 3)
                rospy.loginfo("IMU Orientation: Ox: {0}, Oy: {1}, Oz {2}, Ow: {3}\n".format(ox, oy, oz, ow))
        rospy.loginfo("Shutting down zed node")
        self.zed.close()


def main():
    rospy.init_node('zed_node')
    zed_node = ZedNode()
    zed_node.run()

if __name__ == "__main__":
    main()
