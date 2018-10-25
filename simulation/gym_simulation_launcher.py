#!/usr/bin/env python
import json
import logging
import os
import sys
import time
import traceback

import numpy as np
import yaml

from duckietown_slimremote.networking import make_pull_socket, has_pull_message, receive_data, make_pub_socket, \
    send_gym
from gym_duckietown.envs import DuckietownEnv
from gym_duckietown.simulator import Simulator, ROAD_TILE_SIZE
from log import ROSLogger

# Settings
DEBUG = True

logging.basicConfig()
logger = logging.getLogger('launcher')
logger.setLevel(logging.DEBUG)


def env_as_yaml(name):
    environment = os.environ.copy()
    if not name in environment:
        msg = 'Could not find variable "%s"; I know:\n%s' % (name, json.dumps(environment, indent=4))
        raise Exception(msg)
    v = environment[name]
    try:
        return yaml.load(v)
    except Exception as e:
        msg = 'Could not load YAML: %s\n\n%s' % (e, v)
        raise Exception(msg)


def main():
    environment = os.environ.copy()

    launcher_parameters = env_as_yaml('launcher_parameters')

    logger.info('launcher parameters:\n\n%s' % json.dumps(launcher_parameters, indent=4))

    env_parameters = launcher_parameters['environment-parameters']
    num_episodes = launcher_parameters['episodes']
    steps_per_episodes = launcher_parameters['steps_per_episode']
    environment_class = launcher_parameters['environment-constructor']
    include_map = launcher_parameters['include_map']
    agent_info = launcher_parameters['agent-info']
    LOG_DIR = environment['DTG_LOG_DIR']

    from gym_duckietown import __version__
    logger.debug('using gym-duckietown version %s' % __version__)

    name2class = {
        'DuckietownEnv': DuckietownEnv,
        'Simulator': Simulator,
    }
    if not environment_class in name2class:
        msg = 'Could not find environment class {} in {}'.format(environment_class, list(name2class))
        raise Exception(msg)

    klass = name2class[environment_class]
    env = klass(**env_parameters)

    command_socket, command_poll = make_pull_socket()

    logger.debug('Logging gym state to: {}'.format(LOG_DIR))
    data_logger = ROSLogger(logdir=LOG_DIR)

    min_nsteps = 10
    MAX_FAILURES = 5
    nfailures = 0
    episodes = ['ep%03d' % _ for _ in range(num_episodes)]

    logger.debug("Simulator listening to incoming connections...")

    observations = env.reset()

    try:
        while episodes:
            if nfailures >= MAX_FAILURES:
                msg = 'Too many failures: %s' % nfailures
                raise Exception(msg)  # XXX

            episode_name = episodes[0]

            logger.info('Starting episode %s' % episode_name)

            # data_logger.log_misc(env_parameters)

            data_logger.start_episode(episode_name)

            logger.info('Logging basic info')
            data_logger.write_json('env_parameters', None, env_parameters)
            data_logger.write_json('environment_class', None, environment_class)
            data_logger.write_json('launcher_parameters', None, launcher_parameters)

            logger.info('Now running episode')
            try:
                nsteps = run_episode(env, data_logger, max_steps_per_episode=steps_per_episodes,
                                     command_socket=command_socket,
                                     command_poll=command_poll,
                                     agent_info=agent_info, include_map=include_map)
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
    agent_info['simulation_done'] = True

    send_gym(
            socket=Global.publisher_socket,
            img=observations,
            reward=0.0,
            done=True,
            misc=agent_info
    )
    logger.info('Clean exit.')


class Global(object):
    publisher_socket = None


def get_next_data(command_socket, command_poll):
    while True:
        if has_pull_message(command_socket, command_poll):
            success, data = receive_data(command_socket)
            if success:
                return data
            else:
                # in error case, data will contain the err msg
                msg = 'Invalid data received: %s' % data
                logger.error(msg)
        else:
            time.sleep(0.001)


def run_episode(env, data_logger, max_steps_per_episode, command_socket, command_poll, agent_info,
                include_map):
    ''' returns number of steps '''
    observations = env.reset()
    e0 = env.unwrapped
    map_info = dict(map_data=e0.map_data,
                    map_name=e0.map_name,
                    map_file_path=e0.map_file_path,
                    tile_size=ROAD_TILE_SIZE)
    data_logger.write_json('map_info', None, map_info)

    steps = 0
    while steps < max_steps_per_episode:

        reward = 0  # in case it's just a ping, not a motor command, we are sending a 0 reward
        done = False  # same thing... just for gym-compatibility
        misc_ = {}  # same same

        start_listen = time.time()
        data = get_next_data(command_socket, command_poll)
        delta_list = time.time() - start_listen
        # logger.debug('Time to get message: %d ms' % (delta_list*1000))
        misc_['delta_listening'] = delta_list

        if data["topic"] == 0:
            action = data['msg']

            if np.linalg.norm(action) == 0 and steps == 0:
                msg = "Command is 0 and we did not start yet. Assuming client not connected."
                logger.debug(msg)
                time.sleep(0.01)
                continue

            # logger.debug('Stepping with data: %s' % data)
            start_render = time.time()
            observations, reward, done, misc_ = env.step(action)
            delta_render = time.time() - start_render
            # logger.debug('Render time: %d ms' % (delta_render*1000))
            misc_['delta_render'] = delta_render

            t = env.unwrapped.timestamp

            data_logger.write_json('render_time', t, delta_render)

            # start_log = time.time()
            data_logger.log_misc(t=t, misc=misc_)
            data_logger.log_action(t=t, action=action)
            data_logger.log_observations(t=t, observations=observations)
            data_logger.log_reward(t=t, reward=reward)
            # delta_log = time.time() - start_log
            # misc_['delta_log'] = delta_log
            # logger.debug('Log time: %d ms' % (delta_log * 1000))
            steps += 1

            if DEBUG and env.unwrapped.step_count % 24 == 0:
                logger.info("step_count={}, reward={}, done={}".format(
                        env.unwrapped.step_count,
                        np.around(reward, 3),
                        done)
                )
            if done:
                break

        if data["topic"] == 1:
            logger.info("received ping: %s" % data)

        if data["topic"] == 2:
            logger.info("received 2, resetting: %s" % data)
            observations = env.reset()

        # can only initialize socket after first listener is connected - weird ZMQ bug
        if Global.publisher_socket is None:
            Global.publisher_socket = make_pub_socket(for_images=True)

        if data["topic"] in [0, 1]:
            agent_info.update(misc_)
            # logger.debug('sending misc = %s' % misc_)
            if include_map:
                agent_info['map_info'] = map_info

            send_gym(Global.publisher_socket, observations, reward, done, agent_info)
    else:
        logger.info('breaking because steps = %s' % max_steps_per_episode)

    return steps


if __name__ == '__main__':
    try:
        main()
        logger.info('success')
    except BaseException:
        logger.error('anomalous exit of gym_simulation_launcher:\n%s' % traceback.format_exc())
        sys.exit(2)
    sys.exit(0)
