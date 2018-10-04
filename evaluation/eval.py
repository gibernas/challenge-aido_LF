#!/usr/bin/env python
import logging
import math

from duckietown_challenges import wrap_evaluator, ChallengeEvaluator, InvalidSubmission

logging.basicConfig()
logger = logging.getLogger('evaluator')
logger.setLevel(logging.DEBUG)

# we are already in a container

class GymEvaluator(ChallengeEvaluator):

    # first
    def prepare(self, cie):
        # no parameters for the submission
        cie.set_challenge_parameters({})

        # start the *process* that is the gym environment
        # start_simulator_process(nsteps, nepisodes, log_output='ros_output.bag')
        from gym_duckietown import main
        main(log_output='ros_output.bag')

        # now there is an environment listening on the socket

    # then,
    #
    #  submission.run()
    #
    # will be started in the other container

    # third
    def score(self, cie):
        fn = 'ros_output.bag'
        assert os.path.exists(fn)

        scores = compute_scores(fn)

        for score_name, score_value in scores.items():
            cie.set_score(score_name, score_value)


if __name__ == '__main__':
    wrap_evaluator(GymEvaluator())
