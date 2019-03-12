#!/usr/bin/env python3
import json
import os
import traceback
from dataclasses import dataclass

import numpy as np
import yaml

import geometry
from aido_node_wrapper import wrap_direct, Context
from aido_nodes import logger
from aido_schemas import PWMCommands, JPGImage, Duckiebot1Observations, Duckiebot1Commands
from aido_schemas.protocol_simulator import SetMap, SpawnRobot, RobotName, StateDump, Step, RobotInterfaceDescription, \
    RobotObservations, RobotState, protocol_simulator, SetRobotCommands, RobotPerformance, Metric, PerformanceMetrics
from gym_duckietown.envs import DuckietownEnv
from gym_duckietown.simulator import Simulator


# Specialize to our datatype


@dataclass
class MySetRobotCommands(SetRobotCommands):
    robot_name: RobotName
    t_effective: float
    commands: Duckiebot1Commands


@dataclass
class MyRobotObservations(RobotObservations):
    robot_name: RobotName
    t_effective: float
    observations: Duckiebot1Observations


@dataclass
class MyRobotInfo:
    pose: np.ndarray


@dataclass
class MyRobotState(RobotState):
    robot_name: RobotName
    t_effective: float
    state: MyRobotInfo


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


class GymDuckiebotSimulator:
    current_time: float
    reward_cumulative: float

    def init(self, context: Context):
        launcher_parameters = env_as_yaml('launcher_parameters')

        logger.info('launcher parameters:\n\n%s' % json.dumps(launcher_parameters, indent=4))

        env_parameters = launcher_parameters['environment-parameters']

        environment_class = launcher_parameters['environment-constructor']

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
        self.env = klass(**env_parameters)
        self.robot_name = None

        self.last_action = np.array([0.0, 0.0])

    def on_received_seed(self, context, data: int):
        context.log(f'seed({data})')

    def on_received_clear(self, context):
        context.log(f'clear()')

    def on_received_set_map(self, context, data: SetMap):
        context.log(f'set_map({data})')

    def on_received_spawn_robot(self, context, data: SpawnRobot):
        # TODO: check location
        self.robot_name = data.robot_name

    def on_received_get_robot_interface_description(self, context, data: RobotName):
        rid = RobotInterfaceDescription(robot_name=data, observations=JPGImage, commands=PWMCommands)
        context.write('robot_interface_description', rid)

    def on_received_get_robot_performance(self, context, data: RobotName):
        metrics = {}
        metrics['survival_time'] = Metric(higher_is_better=True, cumulative_value=self.current_time,
                                          description="Survival time.")
        metrics['reward'] = Metric(higher_is_better=True, cumulative_value=self.reward_cumulative,
                                   description="Cumulative reward.")
        pm = PerformanceMetrics(metrics)
        rid = RobotPerformance(robot_name=data, t_effective=self.current_time, performance=pm)
        context.write('robot_performance', rid)

    def on_received_start_episode(self, context):
        context.log(f'start_episode()')
        self.current_time = 0.0
        self.reward_cumulative = 0
        try:
            self.env.reset()
        except:
            msg = 'Could not initialize environment:\n%s' % traceback.format_exc()
            raise Exception(msg)

    def on_received_step(self, context, data: Step):
        delta_time = data.until - self.current_time
        if delta_time > 0:
            self.env.update_physics(self.last_action, delta_time=delta_time)
        else:
            context.warning(f'Already at time {data.until}')

        done, reward, msg = self.env._compute_done_reward()
        self.reward_cumulative += reward * delta_time
        self.current_time = data.until

    def on_received_set_robot_commands(self, data: MySetRobotCommands):
        wheels = data.commands.wheels
        action = np.array([wheels.motor_left, wheels.motor_right])
        action = np.clip(action, -1.0, +1.0)
        self.last_action = action

    def on_received_get_robot_observations(self, context: Context):
        obs = self.env.render_obs()

        jpg_data = rgb2jpg(obs)
        camera = JPGImage(jpg_data)
        obs = Duckiebot1Observations(camera)
        ro = MyRobotObservations(self.robot_name, self.current_time, obs)
        context.write('robot_observations', ro, with_schema=True)

    def on_received_get_robot_state(self, context):
        info = self.env.get_agent_info()
        pos = info['Simulator']['cur_pos_cartesian']
        theta = info['Simulator']['cur_angle']

        q = geometry.SE2_from_translation_angle(pos, theta)
        state = MyRobotInfo(pose=q)
        rs = MyRobotState(robot_name=self.robot_name, t_effective=self.current_time, state=state)
        context.write('robot_state', rs)

    def on_received_dump_state(self, context):
        context.write('dump_state', StateDump(None))


# noinspection PyUnresolvedReferences
def rgb2jpg(rgb) -> bytes:
    import cv2
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    compress = cv2.imencode('.jpg', bgr)[1]
    jpg_data = np.array(compress).tostring()
    return jpg_data


def main():
    node = GymDuckiebotSimulator()
    protocol = protocol_simulator
    protocol.inputs['set_robot_commands'] = MySetRobotCommands
    protocol.outputs['robot_observations'] = MyRobotObservations
    protocol.outputs['robot_state'] = MyRobotState
    wrap_direct(node=node, protocol=protocol)


if __name__ == '__main__':
    main()
