import json

import numpy as np
from duckietown_challenges import ChallengeInterfaceEvaluatorConcrete

from duckietown_utils import d8n_bag_read_with_progress


def read_scores_data(log_file, cie):
    import rosbag
    bag = rosbag.Bag(log_file)

    topic = '/gym/reward'
    entries = []
    for msg in d8n_bag_read_with_progress(bag, topic, yield_tuple=False):
        entries.append(msg.data)

    entries = np.array(entries)

    reward = np.mean(entries)

    stats = {}
    stats['nsteps'] = len(entries)
    stats['reward'] = float(reward)
    return stats


VALID_ANGLE_LIMIT_DEG = 20


def compute_rules_violation(log_file, cie):
    import rosbag
    bag = rosbag.Bag(log_file)

    accum_score_good_angle = 0.0
    accum_score_valid_direction = 0.0

    tile_coords_history = []
    old_timestamp = None
    for msg in d8n_bag_read_with_progress(bag, '/gym/misc', yield_tuple=False):
        data = json.loads(msg.data)
        # DuckietownEnv = data['DuckietownEnv']
        Simulator = data['Simulator']
        tile_coords = Simulator['tile_coords']
        timestamp = Simulator['timestamp']
        tile_coords_history.append(tuple(tile_coords))
        if old_timestamp is None:
            old_timestamp = timestamp
            continue

        dt = timestamp - old_timestamp
        old_timestamp = timestamp

        cur_pos = Simulator['cur_pos']
        cur_angle_deg = Simulator['cur_angle']

        lane_position = Simulator['lane_position']
        angle_rad = lane_position['angle_rad']
        invalid = int(np.abs(angle_rad) > np.deg2rad(VALID_ANGLE_LIMIT_DEG))

        accum_score_good_angle += dt * np.abs(angle_rad)**2
        accum_score_valid_direction += dt * invalid

    stats = {}
    stats['traveled_tiles'] = len(set(tile_coords_history))
    stats['good_angle'] = float(accum_score_good_angle)
    stats['valid_direction'] = float(accum_score_valid_direction)
    stats['survival_time'] = old_timestamp
    return stats


if __name__ == '__main__':
    cie = ChallengeInterfaceEvaluatorConcrete()
    print read_scores_data('example.bag', cie)
    print compute_rules_violation('example.bag', cie)
