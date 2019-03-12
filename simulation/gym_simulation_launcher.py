#!/usr/bin/env python
import base64
import logging
import os
import sys
import time
import traceback

import numpy as np
import yaml

# Settings
DEBUG = True

logging.basicConfig()
logger = logging.getLogger('launcher')
logger.setLevel(logging.DEBUG)

from duckietown_challenges import InvalidEnvironment
from gym_duckietown.envs import DuckietownEnv
from gym_duckietown.simulator import Simulator, ROAD_TILE_SIZE
from log import ROSLogger


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




import json
def main():

    from duckietown_challenges.col_logging import setup_logging
    setup_logging()

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

    logger.debug('Logging gym state to: {}'.format(LOG_DIR))
    data_logger = ROSLogger(logdir=LOG_DIR)

    min_nsteps = 10
    MAX_FAILURES = 5
    nfailures = 0
    episodes = ['ep%03d' % _ for _ in range(num_episodes)]

    try:
        observations = env.reset()
    except:
        msg = 'Could not initialize environment:\n%s' % traceback.format_exc()
        raise InvalidEnvironment(msg)

    try:
        logger.debug("Simulator listening to incoming connections...")

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
                nsteps = run_episode(env, data_logger,
                                     agent_ci,
                                     episode_name=episode_name,
                                     max_steps_per_episode=steps_per_episodes,
                                     agent_info=agent_info, include_map=include_map)
                logger.info('Finished episode %s' % episode_name)

                if nsteps >= min_nsteps:
                    logger.info('%d steps are enough' % nsteps)
                    episodes.pop(0)
                else:
                    logger.error('episode too short with %s steps' % nsteps)
                    nfailures += 1

            except:
                msg = 'Anomalous error from run_episode():\n%s' % traceback.format_exc()
                logger.error(msg)
                raise
            finally:
                data_logger.end_episode()

    except:
        msg = 'Anomalous error while running episodes:\n%s' % traceback.format_exc()
        logger.error(msg)
        raise

    finally:
        data_logger.close()

        logger.info('Simulation done.')

    logger.info('Clean exit.')




def run_episode(env, data_logger, agent_ci, episode_name, max_steps_per_episode,
                agent_info,
                include_map):
    ''' returns number of steps '''

    e0 = env.unwrapped

    # start on the right lane
    while True:
        observations = env.reset()

        lp = e0.get_lane_pos2(e0.cur_pos, e0.cur_angle)

        i, j = e0.get_grid_coords(e0.cur_pos)
        tile = e0._get_tile(i, j)

        kind = tile['kind']
        # angle = tile['angle']

        # Each tile will have a unique set of control points,
        # Corresponding to each of its possible turns

        is_straight = kind.startswith('straight')

        logger.info('Sampled lane pose %s' % str(lp))
        logger.info('Sampled tile  %s %s %s' % (tile['coords'], tile['kind'], tile['angle']))

        if not is_straight:
            continue

        if lp.dist > +0.04:
            break

    map_info = dict(map_data=e0.map_data,
                    map_name=e0.map_name,
                    tile_size=ROAD_TILE_SIZE)
    data_logger.write_json('map_info', None, map_info)

    steps = 0

    agent_ci.write('episode_start', {'episode_name': episode_name})

    action = [0, 0]
    while steps < max_steps_per_episode:
        start_render = time.time()
        observations, reward, done, misc_ = env.step(action)
        delta_render = time.time() - start_render

        agent_ci.write('camera_image', {'jpg_data': encode_bytes_base64(b"bytes")})

        misc_ = {}  # same same

        start_listen = time.time()
        try:
            r = agent_ci.read_one()
        except StopIteration:
            logger.warn('Agent finished on its own.')
            break

        print('received %s' % r)
        # data = get_next_data(command_socket, command_poll)
        delta_list = time.time() - start_listen
        # logger.debug('Time to get message: %d ms' % (delta_list*1000))
        misc_['delta_listening'] = delta_list

        action = r['msg']

        # logger.debug('Stepping with data: %s' % data)
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
            logger.info('breaking as decided by simulator')
            break

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
