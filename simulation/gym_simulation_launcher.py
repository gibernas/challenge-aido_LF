#!/usr/bin/env python
import json
import logging
import os
import sys
import time
import traceback

import numpy as np
from duckietown_slimremote.networking import make_pull_socket, has_pull_message, receive_data, make_pub_socket, \
    send_gym
from gym_duckietown.envs import DuckietownEnv

from log import ROSLogger

# Settings
DEBUG = True

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
    MAP_NAME = os.getenv('DTG_MAP')
    environment = os.environ.copy()

    def env_as_json(name):
        if not name in environment:
            msg = 'Could not find variable "%s"; I know:\n%s' % (name, json.dumps(environment, indent=4))
            raise Exception(msg)
        return json.loads(environment[name])

    DOMAIN_RAND = env_as_json('DTG_DOMAIN_RAND')
    EPISODES = env_as_json('DTG_EPISODES')
    STEPS_PER_EPISODE = env_as_json('DTG_STEPS_PER_EPISODE')
    challenge = environment['DTG_CHALLENGE']
    LOG_DIR = environment['DTG_LOG_DIR']

    camera_width = env_as_json('DTG_CAMERA_WIDTH')
    camera_height = env_as_json('DTG_CAMERA_HEIGHT')

    misc = {}  # init of info field for additional gym data

    misc['challenge'] = challenge
    # if challenge in ["LF", "LFV"]:
    #     logger.debug("Launching challenge: {}".format(challenge))
    #     from gym_duckietown.config import DEFAULTS
    #
    #     MAP_NAME = DEFAULTS["challenges"][challenge]
    #     misc["challenge"] = challenge
    # else:
    #     pass

    # XXX: what if not? error?
    logger.debug("Using map: {}".format(MAP_NAME))

    env_parameters = dict(map_name=MAP_NAME,
                          max_steps=STEPS_PER_EPISODE * EPISODES,
                          domain_rand=DOMAIN_RAND,
                          camera_width=camera_width,
                          camera_height=camera_height,
                          )

    env = DuckietownEnv(**env_parameters)

    command_socket, command_poll = make_pull_socket()

    logger.debug("Simulator listening to incoming connections...")

    observations = env.reset()

    logger.debug('Logging gym state to: {}'.format(LOG_DIR))
    data_logger = ROSLogger(logdir=LOG_DIR)

    min_nsteps = 10
    MAX_FAILURES = 5
    nfailures = 0
    episodes = ['ep%03d' % _ for _ in range(EPISODES)]
    try:
        while episodes:
            if nfailures >= MAX_FAILURES:
                msg = 'Too many failures: %s' % nfailures
                raise Exception(msg)  # XXX

            episode_name = episodes[0]

            logger.info('Starting episode %s' % episode_name)
            data_logger.start_episode(episode_name)
            data_logger.log_misc(env_parameters)
            try:
                nsteps = run_episode(env, data_logger, max_steps_per_episode=STEPS_PER_EPISODE,
                                     command_socket=command_socket,
                                     command_poll=command_poll,
                                     misc=misc)
                logger.info('Finished episode %s' % episode_name)

                if nsteps >= min_nsteps:
                    logger.info('%d steps are enough' % nsteps)
                    episodes.pop(0)
                else:
                    logger.error('episode too short with %s steps' % nsteps)
                    nfailures += 1

            finally:
                data_logger.end_episode()

    finally:
        data_logger.close()

    logger.info('Simulation done.')
    misc['simulation_done'] = True

    send_gym(
            socket=Global.publisher_socket,
            img=observations,
            reward=0.0,
            done=True,
            misc=misc
    )
    logger.info('Clean exit.')


class Global:
    publisher_socket = None


def get_next_data(command_socket, command_poll):
    # t0 = time.time()
    # timeout = 5
    while True:
        if has_pull_message(command_socket, command_poll):
            success, data = receive_data(command_socket)
            if success:
                return data
            else:
                logger.error(data)  # in error case, this will contain the err msg
        else:
            time.sleep(0.001)


def run_episode(env, data_logger, max_steps_per_episode, command_socket, command_poll, misc):
    ''' returns number of steps '''
    observations = env.reset()
    steps = 0
    while steps < max_steps_per_episode:
        # logger.debug('received: %s' % data)

        reward = 0  # in case it's just a ping, not a motor command, we are sending a 0 reward
        done = False  # same thing... just for gym-compatibility
        misc_ = {}  # same same

        data = get_next_data(command_socket, command_poll)

        if data["topic"] == 0:
            action = data['msg']
            observations, reward, done, misc_ = env.step(action)
            if not np.isfinite(reward):
                msg = 'Invalid reward received: %s' % reward
                raise Exception(msg)

            # XXX cannot be serialized later if misc['vels'] is an array
            if 'vels' in misc_:
                misc_['vels'] = list(misc_['vels'])

            delta_time = 1.0 / env.unwrapped.frame_rate
            t = env.unwrapped.step_count * delta_time
            data_logger.log_action(t=t, action=action)
            data_logger.log_observations(t=t, observations=observations)
            data_logger.log_reward(t=t, reward=reward)

            steps += 1
            # logger.debug('action: {}'.format(action))
            # logger.debug('steps: {}'.format(steps))
            # we log the current environment step
            #
            if DEBUG:
                logger.info("step_count={}, reward={}, done={}".format(
                        env.unwrapped.step_count,
                        np.around(reward, 3),
                        done)
                )
            if done:
                break

        if data["topic"] == 1:
            logger.debug("received ping:", data)

        if data["topic"] == 2:
            observations = env.reset()

        # can only initialize socket after first listener is connected - weird ZMQ bug
        if Global.publisher_socket is None:
            Global.publisher_socket = make_pub_socket(for_images=True)

        if data["topic"] in [0, 1]:
            misc.update(misc_)
            # print('sending misc = %s' % misc)
            send_gym(Global.publisher_socket, observations, reward, done, misc)
    else:
        logger.info('breaking because steps = %s' % max_steps_per_episode)

    return steps


if __name__ == '__main__':
    try:
        main()
        logger.info('success')
    except BaseException as e:
        logger.error(traceback.format_exc(e))
        sys.exit(2)
    sys.exit(0)
