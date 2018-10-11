import pickle
import cv2

class Logger:
    def __init__(self, env, map_name, logfile):
        self.env = env
        self.logfile = logfile
        self.map_name = map_name

    def log(self, reward=0.0, obs=None):
        x, y, z = self.env.cur_pos
        self._log(
            x=x,
            y=y,
            z=z,
            theta=self.env.cur_angle,
            reward=reward,
            obs=obs
        )

    def _log(self, x, y, z, theta, reward, obs):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError


class PickleLogger(Logger):
    def __init__(self, env, map_name, logfile):
        Logger.__init__(self, env, map_name, logfile)
        self.file = open(logfile, mode='+wb')
        self._write_metadata()

    def _write_metadata(self):
        # first line is a metadata
        pickle.dump({
            'map'
        }, self.file, protocol=2)

    def _log(self, x, y, z, theta, reward, obs):
        # easily substituted by a ROS Message
        pickle.dump({
            'x': x,
            'y': y,
            'z': z,
            'theta': theta,
            'reward': reward,
            'obs': cv2.resize(obs, None, fx=0.5, fy=0.5)
        }, self.file, protocol=2)

    def close(self):
        self.file.flush()
        self.file.close()
