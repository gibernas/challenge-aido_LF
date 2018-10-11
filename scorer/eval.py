# !/usr/bin/env python
import duckietown_challenges as dc
from make_video import make_video_bag
from read_scores import read_scores_data


class Scorer(dc.ChallengeScorer):
    def score(self, cie):
        # get bag from previous steps
        bag_filename = cie.get_completed_step_evaluation_file('step1-simulation', 'log.bag')

        # compute stats and scores
        stats = read_scores_data(bag_filename, cie)
        cie.set_scores(stats)

        # create a video
        tmp_dir = cie.get_tmp_dir()

        mp4 = make_video_bag(bag_filename, tmp_dir=tmp_dir)
        cie.set_evaluation_file('video.mp4', mp4)


if __name__ == '__main__':
    dc.wrap_scorer(Scorer())
