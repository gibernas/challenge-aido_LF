#!/usr/bin/env python

import gym
from tqdm import tqdm
import gym_duckietown_agent  # DO NOT CHANGE THIS IMPORT (the environments are defined here)
from duckietown_challenges import wrap_solution, ChallengeSolution, ChallengeInterfaceSolution


class Submission(ChallengeSolution):
    def run(self, cis):
        assert isinstance(cis, ChallengeInterfaceSolution)  # this is a hack that would help with autocompletion
        # load the model: this is a tensorflow-based example
        from model import TfInference

        # define observation and output shapes
        model = TfInference(observation_shape=(1, 120, 160, 3),
                            action_shape=(1, 2),
                            graph_location='trained_models/')
        # get the configuration parameters for this challenge
        params = cis.get_challenge_parameters()

        output = {'status': 'ok'}

        # the environment
        env = gym.make(params['env'])
        done = False
        # we make sure we have connection with the environment and it is ready to go
        try:
            observation = env.reset()
            reward_acc = 0
            # we run the predictions for a number of episodes
            for episode in tqdm(range(params['episodes'])):
                # for each episodes we do a 'horizon' number of steps
                if done:
                    env.reset()
                for _ in range(params['horizon']):
                    action = model.predict(observation)
                    observation, reward, done, info = env.step(action)
                    reward_acc += reward
                    if done:
                        pass  # break if we have a way to close the gym
            # let's close gracefully our environment
            # env.close()
            # release the CPU/GPU resources our computation graph is using
            model.close()
        except Exception as e:
            output['status'] = 'bad'
            output['msg'] = e.message

        # TODO: What's exactly this?
        cis.set_solution_output_dict(output)

        # for debugging you can create
        #cis.set_output_file('checkpoint', 'trained_models/checkpoint')


if __name__ == '__main__':
    wrap_solution(Submission())
