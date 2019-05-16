import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import os


class ROSClient(object):
    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        # TODO not sure about this
        self.vehicle = os.getenv('HOSTNAME')

        self.cam_sub = rospy.Subscriber('/{}/camera_node/image/compressed'.format(
            self.vehicle),CompressedImage, self._cam_cb)

        self.cmd_pub = rospy.Publisher('/{}/wheels_driver_node/wheels_cmd'.format(
            self.vehicle), WheelsCmdStamped, queue_size=10)

        self.initialized=False

        # Initializes the node
        rospy.init_node('ROSClient')
        rospy.on_shutdown(self.on_shutdown)
        
        self.r = rospy.Rate(15)

    def on_shutdown(self):
        commands = {u'motor_right': 0.0, u'motor_left': 0.0}
        self.send_commands(commands)
        
    def _cam_cb(self, msg):
        """
        Callback to listen to last outputted camera image and store it
        """
        self.image = msg.data
        self.initialized=True


    def send_commands(self, cmds):
        """
        Publishes the wheel commands to ROS
        """
        time = rospy.get_rostime()
        cmd_msg = WheelsCmdStamped()
        cmd_msg.header.stamp.secs = time.secs
        cmd_msg.header.stamp.nsecs = time.nsecs
        cmd_msg.vel_right = cmds[u'motor_right']
        cmd_msg.vel_left  = cmds[u'motor_left']
        self.cmd_pub.publish(cmd_msg)
