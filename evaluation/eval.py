#!/usr/bin/env python
import logging
import os
import pickle
import random
import subprocess
import numpy

from duckietown_challenges import wrap_evaluator, ChallengeEvaluator, ChallengeInterfaceEvaluator, wait_for_file

logging.basicConfig()
logger = logging.getLogger('evaluator')
logger.setLevel(logging.DEBUG)

LOGFILE = '/challenge-evaluation-output/' + 'logfile.pickle'


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
        assert isinstance(cie, ChallengeInterfaceEvaluator)

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
        environment['DTG_DOMAIN_RAND'] = 'false'
        environment['DTG_MAX_STEPS'] = str(EPISODES * HORIZON)  # TODO: verify this actually controls the MAX
        environment['DTG_LOGFILE'] = LOGFILE  # TODO: verify
        logger.info('challenge: %s' % os.environ['DTG_CHALLENGE'])  #

        # this command is on the base image gym-duckietown-server
        cmd = ['/bin/bash', 'launch.sh']
        self.gym_process = subprocess.Popen(
                args=cmd,
                env=environment
        )
        logger.debug('gym started with pid = {}'.format(self.gym_process.pid))

        # FIXME: very fragile process synchronization
        wait_for_file(LOGFILE, 20, 1)

    # then, the system runs:
    #   submission.run() -- defined in solution.py in the user's container
    # will be started in Submission Container

    # submission.run() is done
    # we run score() in Evaluation Container (this container)
    def score(self, cie):
        self.gym_process.wait()

        assert isinstance(cie, ChallengeInterfaceEvaluator)

        scores = self.compute_scores(LOGFILE)

        for score_name, score_value in scores.items():
            cie.set_score(score_name, score_value)


    def compute_scores(self, log_file):
        with open(log_file, mode='rb') as f:
            map_name = pickle.load(f)
            logger.debug('Computing score for: {}'.format(map_name))
            eof = False
            while not eof:
                try:
                    position = pickle.load(f)
                    print(position)
                except EOFError:
                    eof = True

        return {
            'lf': random.uniform(0, 100),  # let the God of Randomness decide who's the first on the leaderboard
        }


if __name__ == '__main__':
    wrap_evaluator(GymEvaluator())
