#!/usr/bin/env python
import json
import logging
import os
import sys
import traceback

import numpy as np

from gym_duckietown.config import DEFAULTS
from gym_duckietown.envs import DuckietownEnv

from duckietown_slimremote.networking import make_pull_socket, has_pull_message, receive_data, make_pub_socket, \
    send_gym

from log import ROSLogger


# Settings
DEBUG = True
DEFAULT_LOGFILE = 'evaluation.log'


logging.basicConfig()
logger = logging.getLogger('gym')
logger.setLevel(logging.DEBUG)


# ========== Environment Variables ==================
# DTG_MAP
# DTG_DOMAIN_RAND
# DTG_MAX_STEPS
# DTG_CHALLENGE
# DTG_LOGFILE
# DTG_EPISODES
# DTG_HORIZON


def main():
    # pulling out of the environment
    MAP_NAME = os.getenv('DTG_MAP', DEFAULTS["map"])
    DOMAIN_RAND = bool(os.getenv('DTG_DOMAIN_RAND', DEFAULTS["domain_rand"]))
    LOG_FILE_PATH = os.getenv('DTG_LOGFILE', DEFAULT_LOGFILE)
    EPISODES = json.loads(os.environ.get('DTG_EPISODES', '10'))  # 10
    HORIZON = json.loads(os.environ.get('DTG_HORIZON', '500'))  # 500
    MAX_STEPS = int(os.getenv('DTG_MAX_STEPS', EPISODES * HORIZON))
    misc = {}  # init of info field for additional gym data

    challenge = os.getenv('DTG_CHALLENGE', "")
    if challenge in ["LF", "LFV"]:
        logger.debug("Launching challenge: {}".format(challenge))
        MAP_NAME = DEFAULTS["challenges"][challenge]
        misc["challenge"] = challenge
    else:
        pass
        # XXX: what if not? error?
    logger.debug("Using map: {}".format(MAP_NAME))

    env_parameters = dict(map_name=MAP_NAME,
        max_steps=MAX_STEPS,
        domain_rand=DOMAIN_RAND)

    env = DuckietownEnv(**env_parameters)
    publisher_socket = None
    command_socket, command_poll = make_pull_socket()

    logger.debug("Simulator listening to incoming connections...")

    observations = env.reset()

    logger.debug('Logging gym state to: {}'.format(LOG_FILE_PATH))
    data_logger = ROSLogger(logfile=LOG_FILE_PATH)
    data_logger.log_misc(**env_parameters)
    try:
        steps = 0
        success = False
        while steps < env.max_steps:
                while not success:
                    if has_pull_message(command_socket, command_poll):
                        success, data = receive_data(command_socket)
                        if not success:
                            logger.error(data)  # in error case, this will contain the err msg
                            continue
                        else:
                            pass
                            # logger.debug('received: %s' % data)

                reward = 0  # in case it's just a ping, not a motor command, we are sending a 0 reward
                done = False  # same thing... just for gym-compatibility
                misc_ = {}  # same same

                if data["topic"] == 0:
                    action = data['msg']
                    observations, reward, done, misc_ = env.step(action)
                    if not np.isfinite(reward):
                        msg = 'Invalid reward received: %s' % reward
                        raise Exception(msg)

                    # XXX cannot be serialized later if misc['vels'] is an array
                    if 'vels' in misc_:
                        misc_['vels'] = list(misc_['vels'])

                    DELTA = 0.2 # FIXME
                    t = env.unwrapped.step_count * DELTA
                    data_logger.log_action(t=t, action=action)
                    data_logger.log_observations(t=t, observations=observations)
                    data_logger.log_reward(t=t, reward=reward)

                    steps += 1
                    # logger.debug('action: {}'.format(action))
                    # logger.debug('steps: {}'.format(steps))
                    # we log the current environment step

                    if DEBUG:
                        logger.info("challenge={}, step_count={}, reward={}, done={}".format(
                            challenge,
                            env.unwrapped.step_count,
                            np.around(reward, 3),
                            done)
                        )
                    if done:
                        env.reset()

                if data["topic"] == 1:
                    logger.debug("received ping:", data)

                if data["topic"] == 2:
                    observations = env.reset()
                    data_logger.reset()

                # can only initialize socket after first listener is connected - weird ZMQ bug
                if publisher_socket is None:
                    publisher_socket = make_pub_socket(for_images=True)

                if data["topic"] in [0, 1]:
                    misc.update(misc_)
                    # print('sending misc = %s' % misc)
                    send_gym(publisher_socket, observations, reward, done, misc)

                success = False
    finally:
        data_logger.close()

    logger.info('Simulation done.')
    misc['simulation_done'] = True

    send_gym(
        socket=publisher_socket,
        img=observations,
        reward=0.0,
        done=True,
        misc=misc
    )
    logger.info('Clean exit.')


if __name__ == '__main__':
    try:
        main()
        logger.info('success')
    except BaseException as e:
        logger.error(traceback.format_exc(e))
        sys.exit(2)
    sys.exit(0)
