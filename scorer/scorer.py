# !/usr/bin/env python
import os

import numpy as np

import duckietown_challenges as dc
from make_video import make_video_bag
from read_scores import read_scores_data, compute_rules_violation


class Scorer(dc.ChallengeScorer):
    def score(self, cie):
        # get bag from previous steps
        logdir = cie.get_completed_step_evaluation_file('step1-simulation', 'episodes')
        episodes = list(os.listdir(logdir))
        print('episodes: %s' % episodes)

        compute_average_for = ['reward', 'good_angle', 'valid_direction', 'survival_time', 'traveled_tiles']

        per_episode = {}
        for episode_name in episodes:
            bag_filename = os.path.join(logdir, episode_name, 'log.bag')

            stats = {}
            stats.update(read_scores_data(bag_filename, cie))
            stats.update(compute_rules_violation(bag_filename, cie))

            per_episode[episode_name] = stats

            # create a video
            tmp_dir = cie.get_tmp_dir()

            mp4 = make_video_bag(bag_filename, tmp_dir=tmp_dir)
            if mp4 is not None:
                cie.set_evaluation_file(os.path.join('episodes', episode_name, 'video.mp4'), mp4)

        cie.set_score('episodes', per_episode)

        for k in compute_average_for:
            values = [_[k] for _ in per_episode.values()]
            cie.set_score('%s_mean' % k, float(np.mean(values)))
            cie.set_score('%s_median' % k, float(np.median(values)))
            cie.set_score('%s_min' % k, float(np.min(values)))
            cie.set_score('%s_max' % k, float(np.max(values)))



if __name__ == '__main__':
    dc.wrap_scorer(Scorer())
