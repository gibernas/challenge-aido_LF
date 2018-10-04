#!/usr/bin/env python
import random
import gym
import gym_duckietown_agent
from duckietown_challenges import wrap_solution, ChallengeSolution, ChallengeInterfaceSolution


class Submission(ChallengeSolution):
    def run(self, cis):
        # run the agent
        from model import TfInference

        model = TfInference(input_shape=(1, 120, 160, 3),
                            output_shape=(1, 2),
                            graph_location='trained_models/')

        env = gym.make("Duckietown-Lf-Lfv-Navv-Silent-v0")

        observation = env.reset()
        done = False

        while not done:
            action = model.predict(observation)
            observation, reward, done, info = env.step(action)

        env.close()
        model.close()

        # for debugging you can create
        #cis.set_output_file('checkpoint', 'trained_models/checkpoint')


if __name__ == '__main__':
    wrap_solution(Submission())
