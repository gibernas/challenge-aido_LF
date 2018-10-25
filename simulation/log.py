import base64
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
        self.logfile_json = os.path.join(self.logdir, episode_name, 'log.gsl1.ds1.json')
        d = os.path.dirname(self.logfile)
        if not os.path.exists(d):
            os.makedirs(d)
        with open(self.logfile, 'w') as f:
            f.write('starting...')
        self.bag = rosbag.Bag(self.logfile, 'w')
        self.json_file = open(self.logfile_json, 'w')

    def end_episode(self):
        self.bag.close()
        self.bag = None
        self.logfile = None
        self.json_file.close()

    def write(self, topic, msg):
        if self.bag is None:
            msg = 'Episode log not started'
            raise Exception(msg)
        self.bag.write(topic, msg)

    def write_json(self, topic, timestamp, data):
        s = {'~LogEntry': dict(topic=topic, timestamp=timestamp, data=data)}
        try:
            sj = json.dumps(s)
        except BaseException:
            msg = 'Cannot serialize topic "%s":\n%s' % (topic, data)
            print(msg)
        else:
            self.json_file.write(sj + '\n')
            self.json_file.flush()

    def close(self):
        if self.bag is not None:
            msg = 'end_episode() not called.'
            raise Exception(msg)

    def log_action(self, t, action):
        self.write('/gym/action/0', Float32(action[0]))
        self.write('/gym/action/1', Float32(action[1]))

        # self.write('/gym/actions', String(json.dumps(misc)))

        self.write_json('actions', t, list(action))

    def log_misc(self, t, misc):
        self.write('/gym/misc', String(json.dumps(misc)))
        self.write_json('misc', t, misc)
        for k, v in misc.items():
            self.write_json(k, t, v)

    def log_observations(self, t, observations):
        timestamp = rospy.Time.from_sec(t)
        # rgb_from_bgr
        observations_rgb = observations
        observations_bgr = cv2.cvtColor(observations_rgb, cv2.COLOR_BGR2RGB)
        msg = d8_compressed_image_from_cv_image(observations_bgr, timestamp=timestamp)
        self.write('/gym/observations', msg)

        data = {'~GenericData': {'base64': str(base64.b64encode(msg.data)), 'content-type': 'image/jpeg'}}
        self.write_json('observations', t, data)

    def log_reward(self, t, reward):
        if not np.isfinite(reward):
            msg = 'Invalid reward received: %s' % reward
            raise Exception(msg)

        self.write('/gym/reward', Float32(reward))
        self.write_json('reward', t, reward)


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
