#!/usr/bin/env python
import os
import subprocess
import sys
import time

import duckietown_challenges as dc
import yaml
from duckietown_challenges import Timeout


def env_as_yaml(name):
    environment = os.environ.copy()
    if not name in environment:
        env_s = yaml.safe_dump(environment, default_flow_style=False)
        msg = 'Could not find variable "%s"; I know:\n%s' % (name, env_s)
        raise Exception(msg)
    v = environment[name]
    try:
        return yaml.load(v)
    except Exception as e:
        msg = 'Could not load YAML: %s\n\n%s' % (e, v)
        raise Exception(msg)


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
        cie.info('Preparing here..')
        assert isinstance(cie, dc.ChallengeInterfaceEvaluator)

        parameters = env_as_yaml('solution_parameters')
        cie.info('Solution parameters: %s' % parameters)

        d = cie.get_tmp_dir()

        # d = 'tmp-files'

        self.logdir = os.path.join(d, 'episodes')
        if not os.path.exists(self.logdir):
            os.makedirs(self.logdir)
        # parameters for the submission

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

        cie.info('Waiting for Gym to activate...')
        for _ in wait_for_file_yield(cie, self.logdir, 20, 1):
            if self.gym_process.poll() is not None:
                msg = 'The gym terminated with return code %s before starting.' % self.gym_process.returncode
                raise dc.InvalidEvaluator(msg)

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

        cie.set_score('simulation-passed', 1)

        cie.info('saving files')
        set_evaluation_dir(cie, 'episodes', self.logdir)
        cie.info('score() terminated gracefully.')


def wait_for_file_yield(cie, fn, timeout, wait):
    t0 = time.time()
    i = 0
    notice_period = 10
    while not os.path.exists(fn):
        passed = int(time.time() - t0)
        to_wait = timeout - passed
        if i % notice_period == 0:
            cie.debug('Output %s not ready yet (%s secs passed, will wait %s secs more)' % (fn, passed, to_wait))
        if time.time() > t0 + timeout:
            msg = 'Timeout of %s while waiting for %s.' % (timeout, fn)
            raise Timeout(msg)
        yield
        i += 1
        time.sleep(wait)


def set_evaluation_dir(cie, basename, realdir):
    for bn in os.listdir(realdir):
        fn = os.path.join(realdir, bn)
        if os.path.isdir(fn):
            set_evaluation_dir(cie, os.path.join(basename, bn), fn)
        else:
            cie.set_evaluation_file(os.path.join(basename, bn), fn)


if __name__ == '__main__':
    dc.wrap_evaluator(GymEvaluator())
