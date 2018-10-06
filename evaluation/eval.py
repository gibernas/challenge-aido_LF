#!/usr/bin/env python
import logging
import os
import random
import subprocess

from duckietown_challenges import wrap_evaluator, ChallengeEvaluator

logging.basicConfig()
logger = logging.getLogger('evaluator')
logger.setLevel(logging.DEBUG)


# we are in Evaluation Container
class GymEvaluator(ChallengeEvaluator):

    def __init__(self):
        # gym process handler
        self.gym_process = None

    # first thing to start
    # 1) run prepare() in Evaluation Container  (this one)
    # start the *process* that is the gym environment
    # start_simulator_process(nsteps, nepisodes, log_output='ros_output.bag')
    # now there is an environment listening on the socket
    def prepare(self, cie):
        EPISODES = int(os.environ.get('DTG_EPISODES'))  # 10
        HORIZON = int(os.environ.get('DTG_HORIZON'))  # 500
        ENVIRONMENT = os.environ.get('DTG_ENVIRONMENT')  # 'Duckietown-Lf-Lfv-Navv-Silent-v0'

        # parameters for the submission
        cie.set_challenge_parameters({
            'env': ENVIRONMENT,
            'episodes': EPISODES,
            'horizon': HORIZON
        })

        # we can configure the gym launcher via environment variables

        environment = os.environ.copy()
        environment['DUCKIETOWN_DOMAIN_RAND'] = 'false'
        environment['DUCKIETOWN_MAX_STEPS'] = str(EPISODES * HORIZON)  # TODO: verify this actually controls the MAX
        logger.info('challenge: %s' % os.environ['DUCKIETOWN_CHALLENGE'])  #

        # this command is on the base image gym-duckietown-server
        cmd = ['launch-gym-server-with-xvfb']
        self.gym_process = subprocess.Popen(
                args=cmd,
                env=environment
        )
        logger.debug('gym launched with pid = {}'.format(self.gym_process.pid))

    # then, the system runs:
    #   submission.run() -- defined in solution.py in the user's container
    # will be started in Submission Container

    # submission.run() is done
    # we run score() in Evaluation Container (this container)
    def score(self, cie):
        log_file = 'ros_output.bag'
        # assert os.path.exists(log_file)

        scores = self.compute_scores(log_file)

        for score_name, score_value in scores.items():
            cie.set_score(score_name, score_value)

        self.gym_process.terminate()  # TODO: Here? Maybe?

    def compute_scores(self, log_file):
        return {
            'lf': random.uniform(0, 100),  # let the God of Randomness decide who's the first on the leaderboard
        }


if __name__ == '__main__':
    wrap_evaluator(GymEvaluator())
