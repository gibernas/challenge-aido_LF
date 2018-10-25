# !/usr/bin/env python
import os

import duckietown_challenges as dc
from locate import locate_files
from duckietown_world.svg_drawing import draw_logs_main_


class Visualizer(dc.ChallengeScorer):
    def score(self, cie):
        # get bag from previous steps
        logdir = cie.get_completed_step_evaluation_file('step1-simulation', 'episodes')

        tmp_dir = cie.get_tmp_dir()

        logs = locate_files(logdir, '*.gsl1.ds1.json')
        for log in logs:
            rel = os.path.relpath(log, logdir)
            rpath = rel.replace('.gsl1.ds1.json', '')
            output = rpath

            draw_logs_main_(filename=log, output=output)

            fn_svg = os.path.join(output, 'drawing.svg')
            fn_html = os.path.join(output, 'drawing.html')
            for f in [fn_svg, fn_html]:
                cie.set_evaluation_file(f, f)


        cie.set_score('videos', 1)


if __name__ == '__main__':
    dc.wrap_scorer(Visualizer())
