#!/usr/bin/env python
import logging
import os
from duckietown_challenges import wrap_evaluator, ChallengeEvaluator, InvalidSubmission

logging.basicConfig()
logger = logging.getLogger('evaluator')
logger.setLevel(logging.DEBUG)


# we are in Evaluation Container
class GymEvaluator(ChallengeEvaluator):
    # first thing to start

    # 1) run prepare() in Evaluation Container  (this one)
    def prepare(self, cie):
        # no parameters for the submission
        cie.set_challenge_parameters({})

        # this command is on the base image gym-duckietown-server
        cmd = ['launch-gym-server-with-xvfb']
        ret = os.system(" ".join(cmd))

        print(ret)
        # start the *process* that is the gym environment
        # start_simulator_process(nsteps, nepisodes, log_output='ros_output.bag')


        # now there is an environment listening on the socket

    # then, the system runs:
    #
    #  submission.run() -- defined in solution.py in the user's container
    #
    # will be started in Submission Container

    # after the previous is finished,
    # we run score() in Evaluation Container (this container)
    def score(self, cie):
        log_file = 'ros_output.bag'
        assert os.path.exists(log_file)

        scores = self.compute_scores(log_file)

        for score_name, score_value in scores.items():
            cie.set_score(score_name, score_value)

    def compute_scores(self, log_file):
        return {
            'task_a': 100.0,
            'task_b': 100.0
        }


if __name__ == '__main__':
    wrap_evaluator(GymEvaluator())
