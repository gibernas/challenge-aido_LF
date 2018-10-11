import cv2
import numpy as np
import rosbag
import rospy
from rospy import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String


class ROSLogger(object):

    def __init__(self, env, map_name, logfile):
        self.logfile = logfile
        with open(logfile, 'w') as f:
            f.write('starting...')
        self.bag = rosbag.Bag(self.logfile, 'w')

    def close(self):
        self.bag.close()

    def reset(self):

        self.bag.write('/gym/reset', String('Reset'))

    def log_action(self, t, action):
        self.bag.write('/gym/action/0', Float32(action[0]))
        self.bag.write('/gym/action/1', Float32(action[1]))

    def log_observations(self, t, observations):
        timestamp = rospy.Time.from_sec(t)
        # rgb_from_bgr
        msg = d8_compressed_image_from_cv_image(observations, timestamp=timestamp)
        self.bag.write('/gym/observations', msg)

    def log_reward(self, t, reward):
        self.bag.write('/gym/reward', Float32(reward))


def d8_compressed_image_from_cv_image(image_cv, same_timestamp_as=None, timestamp=None):
    """
        Create CompressedIamge from a CV BGR image.

        TODO: assumptions on format?
    """

    compress = cv2.imencode('.jpg', image_cv)[1]
    jpg_data = np.array(compress).tostring()
    msg = CompressedImage()

    if same_timestamp_as is not None:
        msg.header.stamp = same_timestamp_as.header.stamp
    elif timestamp is not None:
        msg.header.stamp = timestamp
    else:
        msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = jpg_data
    return msg


if __name__ == '__main__':
    filename = 'test.bag'
    logger = ROSLogger('env', 'map_name', filename)
    y = np.zeros((124, 123, 3), 'uint8')
    logger.log_observations(0, y)
    logger.close()
