#!/usr/bin/env python2

from zuper_nodes_python2 import ComponentInterface
import cv2
from rosclient import ROSClient
import numpy as np

class DuckiebotBridge(object):
    def __init__(self):

        AIDONODE_DATA_IN = '/fifos/agent-in'
        AIDONODE_DATA_OUT = '/fifos/agent-out'
        self.ci = ComponentInterface(AIDONODE_DATA_IN, AIDONODE_DATA_OUT, 'agent', timeout=5)
        self.ci.write_topic_and_expect_zero(u'seed', 32)
        self.ci.write_topic_and_expect_zero(u'episode_start', {u'episode_name': u'episode'})
        self.client=ROSClient()

    def run(self):


        while True:
            if not self.client.initialized:
                continue
            np_arr = np.fromstring(self.client.image, np.uint8)
            #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            data = np_arr.tostring()

            obs = {u'camera': {u'jpg_data': data}}
            self.ci.write_topic_and_expect_zero(u'observations', obs)
            commands = self.ci.write_topic_and_expect(u'get_commands', expect=u'commands')
            commands = commands.data[u'wheels']

            self.client.send_commands(commands)


def main():
    node = DuckiebotBridge()
    node.run()

if __name__ == '__main__':
    main()
