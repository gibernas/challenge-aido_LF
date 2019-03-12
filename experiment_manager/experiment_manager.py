#!/usr/bin/env python
import json
import logging
import os
import sys
import time
import traceback
from dataclasses import is_dataclass
import geometry
import yaml

# Settings
from aido_node_wrapper.wrapper_outside import ComponentInterface, MsgReceived
from aido_schemas import EpisodeStart, protocol_agent
from aido_schemas.protocol_simulator import SetMap, SpawnRobot, RobotConfiguration, Step, protocol_simulator, \
    RobotObservations, SetRobotCommands, RobotPerformance, RobotState
from contracts import indent

DEBUG = True

logging.basicConfig()
logger = logging.getLogger('launcher')
logger.setLevel(logging.DEBUG)


#

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


from typing import *


def can_be_used_as(T1: type, T2: type) -> Tuple[bool, str]:
    if is_dataclass(T2):
        if not is_dataclass(T1):
            msg = f'Expecting dataclass to match to {T2}, got {T1}'
            return False, msg
        h1 = get_type_hints(T1)
        h2 = get_type_hints(T2)
        for k, v2 in h2.items():
            if not k in h1:  # and not optional...
                msg = f'Type {T2}\n  requires field "{k}" \n  of type {v2} \n  but {T1} does not have it. '
                return False, msg
            v1 = h1[k]
            ok, why = can_be_used_as(v1, v2)
            if not ok:
                msg = f'Type {T2}\n  requires field "{k}"\n  of type {v2} \n  but {T1}\n  has annotated it as {v1}\n  which cannot be used. '
                msg += '\n\n' + indent(why, '> ')
                return False, msg

        return True, ''
    else:
        if not issubclass(T1, T2):
            msg = f'Type {T1}\n is not a subclass of {T2}'
            return False, msg
        return True, ''


def main():
    from duckietown_challenges.col_logging import setup_logging
    setup_logging()

    agent_in = '/fifos/agent-in'
    agent_out = '/fifos/agent-out'
    agent_ci = ComponentInterface(agent_in, agent_out, expect_protocol=protocol_agent, nickname="agent")

    sim_in = '/fifos/simulator-in'
    sim_out = '/fifos/simulator-out'
    sim_ci = ComponentInterface(sim_in, sim_out, expect_protocol=protocol_simulator, nickname="simulator")

    type_observations_sim = sim_ci.node_protocol.outputs['robot_observations'].__annotations__['observations']
    logger.info(f'Simulation provides observations {type_observations_sim}')
    type_observations_agent = agent_ci.node_protocol.inputs['observations']
    logger.info(f'Agent requires observations {type_observations_agent}')
    type_commands_sim = sim_ci.node_protocol.inputs['set_robot_commands'].__annotations__['commands']
    logger.info(f'Simulation requires commands {type_commands_sim}')
    type_commands_agent = agent_ci.node_protocol.outputs['commands']
    logger.info(f'Agent provides commands {type_commands_agent}')

    can_be_obs, why = can_be_used_as(type_observations_sim, type_observations_agent)
    if not can_be_obs:
        msg = 'Observations mismatch: %s' % why
        logger.error(msg)
        raise Exception(msg)
    can_be_cmd, why = can_be_used_as(type_commands_agent, type_commands_sim)
    if not can_be_cmd:
        msg = 'Commands mismatch: %s' % why
        logger.error(msg)
        raise Exception(msg)

    environment = os.environ.copy()

    experiment_manager_parameters = env_as_yaml('experiment_manager_parameters')

    logger.info('parameters:\n\n%s' % json.dumps(experiment_manager_parameters, indent=4))

    num_episodes = experiment_manager_parameters['episodes']
    steps_per_episodes = experiment_manager_parameters['steps_per_episode']
    min_nsteps = experiment_manager_parameters['min_nsteps']
    LOG_DIR = environment['DTG_LOG_DIR']

    logger.debug('Logging state to: {}'.format(LOG_DIR))

    MAX_FAILURES = 5
    nfailures = 0
    episodes = ['ep%03d' % _ for _ in range(num_episodes)]

    sim_ci.write('seed', 0)

    try:

        while episodes:
            if nfailures >= MAX_FAILURES:
                msg = 'Too many failures: %s' % nfailures
                raise Exception(msg)  # XXX

            episode_name = episodes[0]

            logger.info('Starting episode %s' % episode_name)

            dn = os.path.join(LOG_DIR, episode_name)
            if not os.path.exists(dn):
                os.makedirs(dn)
            fn = os.path.join(dn, 'log.gs2.cbor')
            fw = open(fn, 'wb')

            agent_ci.cc(fw)
            sim_ci.cc(fw)

            # # data_logger.log_misc(env_parameters)
            #
            # data_logger.start_episode(episode_name)
            #
            # logger.info('Logging basic info')
            # data_logger.write_json('env_parameters', None, env_parameters)
            # data_logger.write_json('environment_class', None, environment_class)
            # data_logger.write_json('launcher_parameters', None, launcher_parameters)

            logger.info('Now running episode')
            try:
                nsteps = run_episode(sim_ci,
                                     agent_ci,
                                     episode_name=episode_name,
                                     max_steps_per_episode=steps_per_episodes)
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
                fw.close()
                pass
                # data_logger.end_episode()

    except:
        msg = 'Anomalous error while running episodes:\n%s' % traceback.format_exc()
        logger.error(msg)
        raise

    finally:
        # data_logger.close()

        logger.info('Simulation done.')

    logger.info('Clean exit.')


timeout = None


def run_episode(sim_ci,
                agent_ci,
                episode_name, max_steps_per_episode):
    ''' returns number of steps '''

    steps = 0

    sim_ci.write('clear')
    sim_ci.write('set_map', SetMap(map_data={}))
    robot_name = 'ego'
    pose = geometry.SE2_from_translation_angle([0,0],0)
    velocity = geometry.se2_from_linear_angular([0, 0], 0)
    robot_conf = RobotConfiguration(pose, velocity)
    sim_ci.write('spawn_robot', SpawnRobot(robot_name, robot_conf))

    sim_ci.write('start_episode')
    agent_ci.write('episode_start', EpisodeStart(episode_name))

    current_sim_time = 0.0
    DELTA = 0.1
    # action = [0, 0]
    while steps < max_steps_per_episode:
        current_sim_time += DELTA

        sim_ci.write('step', Step(current_sim_time))

        start_render = time.time()
        sim_ci.write('get_robot_observations', robot_name)
        recv: MsgReceived[RobotObservations] = sim_ci.read_one(timeout=timeout, expect_topic='robot_observations')
        delta_render = time.time() - start_render

        fn = '/logs/%d.jpg' % steps
        with open(fn, 'wb') as f:
            f.write(recv.data.observations.camera.jpg_data)
        # logger.debug('received: %s' % recv)

        agent_ci.write('observations', recv.data.observations)

        misc_ = {}  # same same

        start_listen = time.time()
        try:
            r: MsgReceived = agent_ci.read_one(timeout=timeout, expect_topic='commands')
        except StopIteration:
            logger.warning('Agent finished on its own.')
            break

        # logger.debug('received %s' % r)
        # data = get_next_data(command_socket, command_poll)
        delta_list = time.time() - start_listen
        # logger.debug('Time to get message: %d ms' % (delta_list*1000))
        misc_['delta_listening'] = delta_list

        # logger.debug('Stepping with data: %s' % data)
        # logger.debug('Render time: %d ms' % (delta_render*1000))
        misc_['delta_render'] = delta_render

        sim_ci.write('get_robot_performance', robot_name)
        recv: MsgReceived[RobotPerformance] = sim_ci.read_one(timeout=timeout, expect_topic='robot_performance')

        sim_ci.write('get_robot_state', robot_name)
        recv: MsgReceived[RobotState] = sim_ci.read_one(timeout=timeout, expect_topic='robot_state')

        commands = SetRobotCommands(robot_name='ego', commands=r.data, t_effective=current_sim_time)
        sim_ci.write('set_robot_commands', commands)

        # data_logger.write_json('render_time', t, delta_render)

        # start_log = time.time()
        # data_logger.log_misc(t=t, misc=misc_)
        # data_logger.log_action(t=t, action=action)
        # data_logger.log_observations(t=t, observations=observations)
        # data_logger.log_reward(t=t, reward=reward)
        # delta_log = time.time() - start_log
        # misc_['delta_log'] = delta_log
        # logger.debug('Log time: %d ms' % (delta_log * 1000))
        steps += 1

        # if DEBUG and env.unwrapped.step_count % 24 == 0:
        #     logger.info("step_count={}, reward={}, done={}".format(
        #             env.unwrapped.step_count,
        #             np.around(reward, 3),
        #             done)
        #     )
        # if done:
        #     logger.info('breaking as decided by simulator')
        #     break

    else:
        logger.info('breaking because steps = %s' % max_steps_per_episode)

    return steps


if __name__ == '__main__':
    # noinspection PyBroadException
    try:
        main()
        logger.info('success')
        sys.exit(0)
    except SystemExit:
        raise
    except BaseException:
        logger.error('anomalous exit of experiment_manager:\n%s' % traceback.format_exc())
        sys.exit(2)
