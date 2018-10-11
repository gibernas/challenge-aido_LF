import pickle
import json
import cv2
import rospy



import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
import rospy

import rosbag
import shutil
class ROSLogger(object):

    def __init__(self, env, map_name, logfile):

        self.logfile = logfile
        with open(logfile, 'w') as f:
            f.write('starting...')
        self.bag = rosbag.Bag(self.logfile, 'w')

    def close(self):
        self.bag.close()

    def reset(self):
        pass

    def log_action(self, i, action):
        print json.dumps(action)


    def log_observations(self, i, observations):
        pass
        # msg = d8_compressed_image_from_cv_image(observations, timestamp=i)
        # self.bag.write('/gym/observations', msg)

    def log_reward(self, i, reward):
        print json.dumps(reward)
        msg = Float32()
        msg.data = reward
        self.bag.write('/gym/reward', msg)


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
        from rospy import Time
        msg.header.stamp = Time.now()
    msg.format = "jpeg"
    msg.data = jpg_data
    return msg
