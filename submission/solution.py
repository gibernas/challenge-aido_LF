#!/usr/bin/env python
import random

from duckietown_challenges import wrap_solution, ChallengeSolution, ChallengeInterfaceSolution


class Submission(ChallengeSolution):
    def run(self, cis):
        # run the agent

        # we can assume that there is a duckietown-gym instance listening
        # on 127.0.0.1:port

        my_agent('127.0.0.1')

        # for debugging you can create
        cis.set_output_file('filename.txt', my_filename)


# def my_agent(hostname):
#     socket = open_socket(hostname)
#
#     while True:
#         msg = receive_msg

if __name__ == '__main__':
    wrap_solution(Submission())
