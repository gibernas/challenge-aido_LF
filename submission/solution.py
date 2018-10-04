#!/usr/bin/env python
import random

from duckietown_challenges import wrap_solution, ChallengeSolution, ChallengeInterfaceSolution


class Submission(ChallengeSolution):
    def run(self, cis):
        # run the agent

        # we can assume that there is a duckietown-gym instance listening
        # on 127.0.0.1:port

        from aido_model import TensorflowLearningModel

        model = TensorflowLearningModel('trained_models')

        while True:

            observations = get_from_slimremote()

            actions = model.predict(observations)

            send_with_slimremote(actions)

        # for debugging you can create
        cis.set_output_file('checkpoint', 'trained_models/checkpoint')



if __name__ == '__main__':
    wrap_solution(Submission())
