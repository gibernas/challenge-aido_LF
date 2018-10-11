#!/usr/bin/env python
import traceback

import gym
import gym_duckietown_agent  # DO NOT CHANGE THIS IMPORT (the environments are defined here)
from duckietown_challenges import wrap_solution, ChallengeSolution, ChallengeInterfaceSolution


def solve(params, cis):
    cis.info('Creating model.')
    from model import TfInference
    # define observation and output shapes
    model = TfInference(observation_shape=(1, 120, 160, 3),
                        action_shape=(1, 2),
                        graph_location='trained_models/')
    # the environment
    cis.info('Making environment')
    env = gym.make(params['env'])
    # we make sure we have connection with the environment and it is ready to go
    cis.info('Reset environment')
    observation = env.reset()
    reward_acc = 0
    # we run the predictions for a number of episodes

    while True:
        action = model.predict(observation)
        observation, reward, done, info = env.step(action)
        if 'simulation_done' in info:
            cis.info('Terminating because received %s' % info)
            break
        reward_acc += reward
        if done:
            cis.info('done = true, so we call env.reset()')
            env.reset()  # break if we have a way to close the gym

    # release the CPU/GPU resources our computation graph is using
    model.close()
    cis.info('graceful exit')


class Submission(ChallengeSolution):
    def run(self, cis):
        assert isinstance(cis, ChallengeInterfaceSolution)  # this is a hack that would help with autocompletion
        # load the model: this is a tensorflow-based example

        # get the configuration parameters for this challenge
        params = cis.get_challenge_parameters()
        cis.info('Parameters: %s' % params)

        try:
            cis.info('Starting.')
            solve(params, cis)  # let's try to solve it
        except Exception as e:
            msg = 'Exception while running solve():\n%s' % traceback.format_exc(e)
            cis.error(msg)
            cis.declare_failure(msg=msg)
        else:
            cis.set_solution_output_dict({})

        # TODO: What's exactly this?
        cis.info('Finished.')
        # for debugging you can create
        #cis.set_output_file('checkpoint', 'trained_models/checkpoint')


if __name__ == '__main__':
    print('Starting submission')
    wrap_solution(Submission())
