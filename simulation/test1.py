import logging
import os
import subprocess
import sys
import gym
# noinspection PyUnresolvedReferences
import gym_duckietown_agent
# noinspection PyUnresolvedReferences
from gym_duckietown.config import DEFAULTS
# noinspection PyUnresolvedReferences
from gym_duckietown.envs import DuckietownEnv

logging.basicConfig()
logger = logging.getLogger('test1')


def test1_go():
    environment = os.environ.copy()
    environment['DTG_DOMAIN_RAND'] = 'false'
    environment['DTG_MAX_STEPS'] = '10'
    environment['DTG_ENVIRONMENT'] = 'Duckietown-Lf-Lfv-Navv-Silent-v0'

    cmd = ['/project/launch.sh']
    print(environment)
    gym_process = subprocess.Popen(
            args=cmd,
            env=environment,
            stderr=sys.stderr,
            stdout=sys.stdout,
            shell=True
    )
    logger.debug('gym started with pid = {}'.format(gym_process.pid))
    #
    # logger.info('Making environment')
    import time
    time.sleep(4)

    env = gym.make(environment['DTG_ENVIRONMENT'])
    # # we make sure we have connection with the environment and it is ready to go
    logger.info('Reset environment')
    observation = env.reset()
    reward_acc = 0
    # # we run the predictions for a number of episodes
    #
    while True:
        action = [0, 0]
        observation, reward, done, info = env.step(action)
        if 'simulation_done' in info:
            break
        reward_acc += reward
        if done:
            env.reset()  # break if we have a way to close the gym

    print('waiting for gym')
    gym_process.wait()
    print('done')

if __name__ == '__main__':
    test1_go()
