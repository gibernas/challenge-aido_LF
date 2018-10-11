#!/usr/bin/env python
import json
import os
import subprocess
import sys

import duckietown_challenges as dc


# we are in Evaluation Container
class GymEvaluator(dc.ChallengeEvaluator):

    def __init__(self):
        # gym process handler
        self.gym_process = None
        self.logdir = None

    # first thing to start
    # 1) run prepare() in Evaluation Container  (this one)
    # start the *process* that is the gym environment
    # start_simulator_process(nsteps, nepisodes, log_output='ros_output.bag')
    # now there is an environment listening on the socket
    def prepare(self, cie):
        cie.info('Preparing..')
        assert isinstance(cie, dc.ChallengeInterfaceEvaluator)

        # EPISODES = int(os.environ.get('DTG_EPISODES'))  # 10
        # STEPS_PER_EPISODE = json.loads(os.environ.get('DTG_STEPS_PER_EPISODE'))
        ENVIRONMENT = os.environ.get('DTG_ENVIRONMENT')  # 'Duckietown-Lf-Lfv-Navv-Silent-v0'

        d = cie.get_tmp_dir()

        self.logdir = os.path.join(d, 'episodes')

        # parameters for the submission
        parameters = {
            'env': ENVIRONMENT,
            # 'episodes': EPISODES,
            # 'horizon': STEPS_PER_EPISODE
        }
        cie.info('Parameters: %s' % parameters)
        cie.set_challenge_parameters(parameters)

        # we can configure the gym launcher via environment variables

        environment = os.environ.copy()

        environment['DTG_LOG_DIR'] = self.logdir  # TODO: verify

        # cie.info('challenge: %s' % os.environ['DTG_CHALLENGE'])  #

        # this command is on the base image gym-duckietown-server
        cmd = ['/bin/bash', 'launch.sh']
        self.gym_process = subprocess.Popen(
                args=cmd,
                env=environment,
                stderr=sys.stderr,
                stdout=sys.stdout,
        )
        cie.debug('gym started with pid = {}'.format(self.gym_process.pid))

        # FIXME: very fragile process synchronization
        cie.info('Waiting for Gym to activate...')
        dc.wait_for_file(self.logdir, 20, 1)

        cie.info('Preparation done.')

    # then, the system runs:
    #   submission.run() -- defined in solution.py in the user's container
    # will be started in Submission Container

    # submission.run() is done
    # we run score() in Evaluation Container (this container)
    def score(self, cie):
        assert isinstance(cie, dc.ChallengeInterfaceEvaluator)

        cie.info('waiting for gym to finish')
        self.gym_process.wait()
        cie.info('finished with return code %s' % self.gym_process.returncode)
        if self.gym_process.returncode:
            msg = 'Gym exited with code %s' % self.gym_process.returncode
            raise dc.InvalidEvaluator(msg)

        cie.set_score('simulation', '1.0')

        set_evaluation_dir(cie, 'episodes', self.logdir)


def set_evaluation_dir(cie, basename, realdir):
    for bn in os.listdir(realdir):
        fn = os.path.join(realdir, bn)
        if os.path.isdir(fn):
            set_evaluation_dir(cie, os.path.join(basename, bn), fn)
        else:
            cie.set_evaluation_file(os.path.join(basename, bn), fn)


if __name__ == '__main__':
    dc.wrap_evaluator(GymEvaluator())
