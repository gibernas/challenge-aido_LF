#!/usr/bin/env python3
import random
from dataclasses import dataclass
from typing import *

import numpy as np

import duckietown_world as dw
import geometry
from aido_schemas import protocol_scenario_maker, Scenario, ScenarioRobotSpec, RobotConfiguration, wrap_direct, Context
from duckietown_world import list_maps, load_map
from duckietown_world.world_duckietown.map_loading import _get_map_yaml
from duckietown_world.world_duckietown.sampling_poses import sample_good_starting_pose


@dataclass
class MyScenario(Scenario):
    scenario_name: str
    environment: str
    robots: Dict[str, ScenarioRobotSpec]


@dataclass
class MyConfig:
    maps: Tuple[str] = ('4way',)
    scenarios_per_map: int = 2


@dataclass
class MyState:
    scenarios_to_go: List[MyScenario]

# scenario_maker_1      | 08:02:04|SimScenarioMaker|scenario_maker.py:58|_create_scenarios(): 63fcec6e268b:SimScenarioMaker: sampled pose: Pose(Rot(0.0deg),[0.7019999 0.1638   ])
# scenario_maker_1      | 08:02:04|SimScenarioMaker|scenario_maker.py:58|_create_scenarios(): 63fcec6e268b:SimScenarioMaker: sampled pose: Pose(Rot(-0.0deg),[1.87199986 2.50379992])
class SimScenarioMaker:
    config: MyConfig = MyConfig()
    state: MyState = MyState([])

    def init(self, context: Context):
        pass

    def _create_scenarios(self, context: Context):
        available = list_maps()
        for map_name in self.config.maps:
            if not map_name in available:
                msg = f'Cannot find map name "{map_name}, know {available}'
                raise Exception(msg)

            yaml_str: str = _get_map_yaml(map_name)

            po = load_map(map_name)

            for i in range(self.config.scenarios_per_map):
                scenario_name = f'{map_name}-{i}'
                robots = {}
                pose = sample_good_starting_pose(po, only_straight=True)

                # t, a = geometry.translation_angle_from_SE2(pose)
                context.info(f'sampled pose: {geometry.SE2.friendly(pose)}')

                po.set_object('db18-4', dw.DB18(), ground_truth=dw.SE2Transform.from_SE2(pose))

                vel = geometry.se2_from_linear_angular([0, 0], 0)
                configuration = RobotConfiguration(pose, vel)
                robots['ego'] = ScenarioRobotSpec(description='Ego robot', configuration=configuration)

                ms = MyScenario(scenario_name=scenario_name, environment=yaml_str, robots=robots)
                self.state.scenarios_to_go.append(ms)

    def on_received_seed(self, context: Context, data: int):
        context.info(f'seed({data})')
        np.random.seed(data)
        random.seed(data)

        self._create_scenarios(context)

    def on_received_next_scenario(self, context: Context):
        if self.state.scenarios_to_go:
            scenario = self.state.scenarios_to_go.pop(0)
            context.write('scenario', scenario)
        else:
            context.write('finished', None)

    def finish(self, context: Context):
        pass


def main():
    node = SimScenarioMaker()
    protocol = protocol_scenario_maker
    protocol.outputs['scenario'] = MyScenario
    wrap_direct(node=node, protocol=protocol)


if __name__ == '__main__':
    main()
