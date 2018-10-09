#!/usr/bin/env python

from duckietown_challenges import wrap_evaluator, ChallengeEvaluator, ChallengeInterfaceEvaluator


class Scorer(ChallengeEvaluator):

    def prepare(self, cie):
        assert isinstance(cie, ChallengeInterfaceEvaluator)

        cie.set_challenge_parameters({})

    def score(self, cie):
        assert isinstance(cie, ChallengeInterfaceEvaluator)
        previous = 'step1'
        assert previous in cie.get_completed_steps()
        fn = cie.get_completed_step_evaluation_file(previous, 'log.pickle')
        from read_scores import read_scores_data
        stats, scores = read_scores_data(fn)

        for k,v in scores.items():
            cie.set_score(k, v)


if __name__ == '__main__':
    wrap_evaluator(Scorer())
