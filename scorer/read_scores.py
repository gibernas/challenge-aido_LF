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
    finite = np.isfinite(entries)
    # infinite = np.logical_not(finite)
    print(entries)

    rewards = entries[finite]

    reward = np.mean(rewards)

    stats = {}
    stats['nsteps'] = len(entries)
    stats['reward'] = float(reward)
    return stats


if __name__ == '__main__':
    cie = ChallengeInterfaceEvaluatorConcrete()
    print read_scores_data('example.bag', cie)
