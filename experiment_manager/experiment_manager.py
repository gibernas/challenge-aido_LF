#!/usr/bin/env python
import json
import logging
import os
import traceback
from dataclasses import dataclass
from typing import Iterator, List

import yaml

from aido_schemas import EpisodeStart, protocol_agent, SetMap, SpawnRobot, Step, protocol_simulator, RobotObservations, \
    SetRobotCommands, RobotPerformance, RobotState, SimulationState, Scenario, protocol_scenario_maker
from aido_schemas.utils import TimeTracker
from aido_schemas.utils_drawing import read_and_draw
from zuper_json.ipce import object_to_ipce
from zuper_json.subcheck import can_be_used_as
from zuper_nodes_wrapper.wrapper_outside import ComponentInterface

logging.basicConfig()
logger = logging.getLogger('launcher')
logger.setLevel(logging.DEBUG)


#

def main(log_dir):
    # first open all fifos
    agent_in = '/fifos/agent-in'
    agent_out = '/fifos/agent-out'
    agent_ci = ComponentInterface(agent_in, agent_out, expect_protocol=protocol_agent, nickname="agent")

    sim_in = '/fifos/simulator-in'
    sim_out = '/fifos/simulator-out'
    sim_ci = ComponentInterface(sim_in, sim_out, expect_protocol=protocol_simulator, nickname="simulator")

    sm_in = '/fifos/scenario_maker-in'
    sm_out = '/fifos/scenario_maker-out'
    sm_ci = ComponentInterface(sm_in, sm_out, expect_protocol=protocol_scenario_maker, nickname="scenario_maker")

    # then check compatibility
    # so that everything fails gracefully in case of error
    agent_ci._get_node_protocol()
    sm_ci._get_node_protocol()
    sim_ci._get_node_protocol()

    check_compatibility_between_agent_and_sim(agent_ci, sim_ci)

    try:
        experiment_manager_parameters = env_as_yaml('experiment_manager_parameters')

        logger.info('parameters:\n\n%s' % json.dumps(experiment_manager_parameters, indent=4))

        steps_per_episodes = experiment_manager_parameters['steps_per_episode']
        min_nsteps = experiment_manager_parameters['min_nsteps']
        seed = experiment_manager_parameters.get('seed', 0)
        episodes_per_scenario = experiment_manager_parameters.get('episodes_per_scenario', 1)

        MAX_FAILURES = 5
        nfailures = 0

        sim_ci.write('seed', seed)
        agent_ci.write('seed', seed)

        episodes = get_episodes(sm_ci, episodes_per_scenario=episodes_per_scenario, seed=seed)

        while episodes:

            if nfailures >= MAX_FAILURES:
                msg = 'Too many failures: %s' % nfailures
                raise Exception(msg)  # XXX

            episode_spec = episodes[0]
            episode_name = episode_spec.episode_name

            logger.info('Starting episode %s' % episode_name)

            dn = os.path.join(log_dir, episode_name)
            if not os.path.exists(dn):
                os.makedirs(dn)
            fn = os.path.join(dn, 'log.gs2.cbor')
            fw = open(fn, 'wb')

            agent_ci.cc(fw)
            sim_ci.cc(fw)

            logger.info('Now running episode')
            try:
                nsteps = run_episode(sim_ci,
                                     agent_ci,
                                     episode_name=episode_name,
                                     scenario=episode_spec.scenario,
                                     max_steps_per_episode=steps_per_episodes)
                logger.info('Finished episode %s' % episode_name)

                if nsteps >= min_nsteps:
                    logger.info('%d steps are enough' % nsteps)
                    episodes.pop(0)

                    output = os.path.join(dn, 'visualization')
                    read_and_draw(fn, output)

                else:
                    logger.error('episode too short with %s steps' % nsteps)
                    nfailures += 1

            except:
                msg = 'Anomalous error from run_episode():\n%s' % traceback.format_exc()
                logger.error(msg)
                raise
            finally:
                fw.close()


    except:
        msg = 'Anomalous error while running episodes:\n%s' % traceback.format_exc()
        logger.error(msg)
        raise

    finally:
        agent_ci.close()
        sim_ci.close()
        logger.info('Simulation done.')

    logger.info('Clean exit.')


def run_episode(sim_ci,
                agent_ci,
                episode_name, scenario: Scenario, max_steps_per_episode):
    ''' returns number of steps '''

    robot_name = 'ego'

    # clear simulation
    sim_ci.write('clear')
    # set map data
    sim_ci.write('set_map', SetMap(map_data=scenario.environment))

    # spawn robot
    for robot_name, robot_conf in scenario.robots.items():
        sim_ci.write('spawn_robot', SpawnRobot(robot_name, robot_conf.configuration))

    # start episode
    sim_ci.write('episode_start', EpisodeStart(episode_name))
    agent_ci.write('episode_start', EpisodeStart(episode_name))

    current_sim_time = 0.0

    # for now, fixed timesteps
    DELTA = 0.1

    steps = 0

    # Get an observation also at t=0
    # sim_ci.write('get_robot_observations', robot_name)
    # recv: MsgReceived[RobotObservations] = sim_ci.read_one(expect_topic='robot_observations')

    while steps < max_steps_per_episode:
        tt = TimeTracker(steps)

        with tt.measure('sim_compute_sim_state'):
            sim_ci.write('get_sim_state')
            recv: MsgReceived[SimulationState] = sim_ci.read_one(expect_topic='sim_state')
            sim_state: SimulationState = recv.data
            if sim_state.done:
                logger.info(f'Breaking because of simulator ({sim_state.done_code} - {sim_state.done_why}')
                break

        # have this first, so we have something for t = 0
        with tt.measure('sim_compute_state'):
            sim_ci.write('get_robot_state', robot_name)
            _recv: MsgReceived[RobotState] = sim_ci.read_one(expect_topic='robot_state')

        with tt.measure('sim_compute_performance'):
            sim_ci.write('get_robot_performance', robot_name)
            _recv: MsgReceived[RobotPerformance] = sim_ci.read_one(expect_topic='robot_performance')

        with tt.measure('sim_physics'):
            current_sim_time += DELTA
            sim_ci.write('step', Step(current_sim_time))

        with tt.measure('sim_render'):
            sim_ci.write('get_robot_observations', robot_name)
            recv: MsgReceived[RobotObservations] = sim_ci.read_one(expect_topic='robot_observations')

        # fn = '/logs/%d.jpg' % steps
        # with open(fn, 'wb') as f:
        #     f.write(recv.data.observations.camera.jpg_data)

        with tt.measure('agent_compute'):
            agent_ci.write('observations', recv.data.observations)

            agent_ci.write('get_commands')
            try:
                r: MsgReceived = agent_ci.read_one(expect_topic='commands')
            except StopIteration:
                msg = 'Agent finished on its own.'
                raise Exception(msg)  # XXX
        with tt.measure('set_robot_commands'):
            commands = SetRobotCommands(robot_name='ego', commands=r.data, t_effective=current_sim_time)
            sim_ci.write('set_robot_commands', commands)

        log_timing_info(tt, sim_ci)

        steps += 1

    else:
        logger.info('Breaking because steps = %s' % max_steps_per_episode)

    return steps


def log_timing_info(tt, sim_ci):
    ipce = object_to_ipce(tt, {}, with_schema=True)
    msg = {'compat': ['aido2'], 'topic': 'timing_information', 'data': ipce}
    j = sim_ci._serialize(msg)
    sim_ci._cc.write(j)
    sim_ci._cc.flush()


def check_compatibility_between_agent_and_sim(agent_ci, sim_ci):
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


@dataclass
class EpisodeSpec:
    episode_name: str
    scenario: Scenario


def get_episodes(sm_ci, episodes_per_scenario: int, seed: int) -> List[EpisodeSpec]:
    sm_ci.write('seed', seed)

    def iterate_scenarios() -> Iterator[Scenario]:
        while True:
            sm_ci.write('next_scenario')
            recv = sm_ci.read_one()
            if recv.topic == 'finished':
                sm_ci.close()
                break
            else:
                yield recv.data

    episodes = []
    for scenario in iterate_scenarios():
        scenario_name = scenario.scenario_name
        logger.info(f'Received scenario {scenario}')
        for i in range(episodes_per_scenario):
            episode_name = f'{scenario_name}-{i}'
            es = EpisodeSpec(episode_name=episode_name, scenario=scenario)
            episodes.append(es)
    return episodes


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


# import duckietown_challenges as dc
#
#
# class GymEvaluator(dc.ChallengeEvaluator):
#     def prepare(self, cie: dc.ChallengeInterfaceEvaluator):
#         cie.set_challenge_parameters({})
#
#     def score(self, cie: dc.ChallengeInterfaceEvaluator):
#         logdir = '/logs'
#         try:
#             main(logdir)
#         except:
#             cr = ChallengeResults(status, message, {})
#         else:
#             cr = ChallengeResults(status, message, {})
#         declare_challenge_results(root, cr)
#
#         cie.set_score('simulation-passed', 1)
#
#         cie.info('saving files')
#         cie.set_evaluation_dir('episodes', logdir)
#         cie.info('score() terminated gracefully.')


#
# if __name__ == '__main__':
#     # noinspection PyBroadException
#     try:
#         main()
#         logger.info('success')
#         sys.exit(0)
#     except SystemExit:
#         raise
#     except BaseException:
#         logger.error('anomalous exit of experiment_manager:\n%s' % traceback.format_exc())
#         sys.exit(2)

if __name__ == '__main__':
    # dc.wrap_evaluator(GymEvaluator())
    main("/logs")
