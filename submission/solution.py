#!/usr/bin/env python
import gym
from tqdm import tqdm
import gym_duckietown_agent
from duckietown_challenges import wrap_solution, ChallengeSolution, ChallengeInterfaceSolution

EPISODES = 10
HORIZON = 500


class Submission(ChallengeSolution):
    def run(self, cis):
        # run the agent
        print('Agent running!')
        from model import TfInference

        model = TfInference(input_shape=(1, 120, 160, 3),
                            output_shape=(1, 2),
                            graph_location='trained_models/')

        env = gym.make("Duckietown-Lf-Lfv-Navv-Silent-v0")

        observation = env.reset()

        for episode in tqdm(range(EPISODES)):
            for horizon in tqdm(range(HORIZON)):
                action = model.predict(observation)
                observation, reward, done, info = env.step(action)

                if done:
                    break

        env.close()
        model.close()

        data = {'guess': 100}
        cis.set_solution_output_dict(data)

        # for debugging you can create
        #cis.set_output_file('checkpoint', 'trained_models/checkpoint')


if __name__ == '__main__':
    wrap_solution(Submission())
