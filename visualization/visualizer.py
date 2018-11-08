# !/usr/bin/env python
import os

import duckietown_challenges as dc
import numpy as np
from duckietown_world.rules import RuleEvaluationResult
from duckietown_world.rules.rule import EvaluatedMetric


class Visualizer(dc.ChallengeScorer):
    def score(self, cie):
        from contracts import disable_all
        disable_all()

        from duckietown_world.svg_drawing.draw_log import draw_logs_main_

        # get bag from previous steps
        logdir = cie.get_completed_step_evaluation_file('step1-simulation', 'episodes')
        episodes = list(os.listdir(logdir))
        #
        # logs = locate_files(logdir, '*.gsl1.ds1.json')
        # for log in logs:
        #     cie.info('processing log %s' % log)
        #     rel = os.path.relpath(log, logdir)
        #     rpath = rel.replace('.gsl1.ds1.json', '')
        #     output = rpath
        #
        #
        if not episodes:
            msg = 'Could not find any episode.'
            raise Exception(msg)
        else:
            per_episode = {}
            for episode_name in episodes:
                log = os.path.join(logdir, episode_name, 'log.gsl1.ds1.json')
                output = os.path.join(episode_name)
                evaluated = draw_logs_main_(filename=log, output=output)

                stats = {}
                for k, evr in evaluated.items():
                    assert isinstance(evr, RuleEvaluationResult)
                    for m, em in evr.metrics.items():
                        assert isinstance(em, EvaluatedMetric)

                        # kk = "/".join((k,) + m)
                        # stats[kk] = em.total
                        assert isinstance(m, tuple)
                        if m:
                            M = "/".join(m)
                        else:
                            M = k
                        stats[M] = float(em.total)
                per_episode[episode_name] = stats

                fn_svg = os.path.join(output, 'drawing.svg')
                fn_html = os.path.join(output, 'drawing.html')
                for f in [fn_svg, fn_html]:
                    cie.set_evaluation_file(f, f)

        cie.set_score('per-episodes', per_episode)

        for k in list(stats):
            values = [_[k] for _ in per_episode.values()]
            cie.set_score('%s_mean' % k, float(np.mean(values)))
            cie.set_score('%s_median' % k, float(np.median(values)))
            cie.set_score('%s_min' % k, float(np.min(values)))
            cie.set_score('%s_max' % k, float(np.max(values)))


if __name__ == '__main__':
    dc.wrap_scorer(Visualizer())
