# !/usr/bin/env python
import os

import numpy as np

import duckietown_challenges as dc
from make_video import make_video_bag
from read_scores import read_scores_data


class Scorer(dc.ChallengeScorer):
    def score(self, cie):
        # get bag from previous steps
        logdir = cie.get_completed_step_evaluation_file('step1-simulation', 'episodes')
        episodes = list(os.listdir(logdir))
        print('episodes: %s' % episodes)

        rewards = []
        for episode_name in episodes:
            bag_filename = os.path.join(logdir, episode_name, 'log.bag')

            stats = read_scores_data(bag_filename, cie)
            cie.set_score(episode_name, stats)

            rewards.append(stats['reward'])

            # create a video
            tmp_dir = cie.get_tmp_dir()

            mp4 = make_video_bag(bag_filename, tmp_dir=tmp_dir)
            if mp4 is not None:
                cie.set_evaluation_file(os.path.join('episodes', episode_name, 'video.mp4'), mp4)

        cie.set_score('reward', float(np.mean(rewards)))


if __name__ == '__main__':
    dc.wrap_scorer(Scorer())
