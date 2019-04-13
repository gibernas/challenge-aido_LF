#!/usr/bin/env python2
from __future__ import unicode_literals

from zuper_nodes_python2 import wrap_direct


class MinimalAgentPython2(object):

    def init(self, context, data):
        context.info('init()')

    def on_received_seed(self, context, data):
        pass

    def on_received_episode_start(self, context, data):
        context.info('Starting episode %s.' % data)

    def on_received_observations(self, context, data):
        jpg_data = data['camera']['jpg_data']

    def on_received_get_commands(self, context, data):
        rgb = {'r': 0.5, 'g': 0.5, 'b': 0.5}
        commands = {
            'wheels': {
                'motor_left': 0.5,
                'motor_right': 0.5
            },
            'LEDS': {
                'center': rgb,
                'front_left': rgb,
                'front_right': rgb,
                'back_left': rgb,
                'back_right': rgb

            }
        }
        context.write('commands', commands)

    def finish(self, context):
        context.info('finish()')


if __name__ == '__main__':
    agent = MinimalAgentPython2()
    wrap_direct(agent)
