import json
import os

import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String


class ROSLogger(object):

    def __init__(self, logdir):
        self.logdir = logdir
        self.logfile = None
        self.bag = None

    def start_episode(self, episode_name):
        self.logfile = os.path.join(self.logdir, episode_name, 'log.bag')
        d = os.path.dirname(self.logfile)
        if not os.path.exists(d):
            os.makedirs(d)
        with open(self.logfile, 'w') as f:
            f.write('starting...')
        self.bag = rosbag.Bag(self.logfile, 'w')

    def end_episode(self):
        self.bag.close()
        self.bag = None
        self.logfile = None

    def write(self, topic, msg):
        if self.bag is None:
            msg = 'Episode log not started'
            raise Exception(msg)
        self.bag.write(topic, msg)

    def close(self):
        if self.bag is not None:
            msg = 'end_episode() not called.'
            raise Exception(msg)

    def log_misc(self, d):
        """ Logs a dictionary as a json structor"""
        msg = String(json.dumps(d))
        self.write('/gym/misc', msg)

    def log_action(self, t, action):
        self.write('/gym/action/0', Float32(action[0]))
        self.write('/gym/action/1', Float32(action[1]))

    def log_observations(self, t, observations):
        timestamp = rospy.Time.from_sec(t)
        # rgb_from_bgr
        observations_rgb = observations
        observations_bgr = cv2.cvtColor(observations_rgb, cv2.COLOR_BGR2RGB)
        msg = d8_compressed_image_from_cv_image(observations_bgr, timestamp=timestamp)
        self.write('/gym/observations', msg)

    def log_reward(self, t, reward):
        self.write('/gym/reward', Float32(reward))


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
    logger = ROSLogger('logs')
    y = np.zeros((124, 123, 3), 'uint8')
    logger.log_observations(0, y)
    logger.close()
