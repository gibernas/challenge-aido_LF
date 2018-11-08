# !/usr/bin/env python
import os

import duckietown_challenges as dc


class VideoMaker(dc.ChallengeScorer):
    def score(self, cie):
        from duckietown_utils import locate_files
        from make_video import make_video_bag

        # get bag from previous steps
        logdir = cie.get_completed_step_evaluation_file('step1-simulation', 'episodes')

        tmp_dir = cie.get_tmp_dir()

        bags = locate_files(logdir, '*.bag')
        for bag_filename in bags:
            cie.info('Creating video for %s' % bag_filename)
            mp4 = make_video_bag(bag_filename, tmp_dir=tmp_dir)
            if mp4 is not None:
                rpath = os.path.relpath(bag_filename.replace('.bag', '.mp4'), logdir)
                cie.set_evaluation_file(rpath, mp4)

        cie.set_score('videos', 1)


if __name__ == '__main__':
    dc.wrap_scorer(VideoMaker())
