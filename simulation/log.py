import pickle
import rosbag


class Logger:
    def __init__(self, env, map_name, logfile):
        self.env = env
        self.logfile = logfile
        self.map_name = map_name

    def log(self, reward=0.0):
        x, y, z = self.env.cur_pos
        self._log(
            x=x,
            y=y,
            z=z,
            theta=self.env.cur_angle,
            reward=reward
        )

    def _log(self, x, y, z, theta, reward):
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

    def _log(self, x, y, z, theta, reward):
        # easily substituted by a ROS Message
        pickle.dump({
            'x': x,
            'y': y,
            'z': z,
            'theta': theta,
            'reward': reward
        }, self.file, protocol=2)

    def close(self):
        self.file.flush()
        self.file.close()


class ROSLogger(Logger):
    def __init__(self, env, map_name, logfile):
        Logger.__init__(self, env, map_name, logfile)
        self.file = logfile + '.bag'  # open(logfile, mode='+wb')

    def _log(self, x, y, z, theta, reward):
        from std_msgs.msg import String, Float32
        from os import path
        # rosbag to write into
        # # TODO add .bag to logfile name
        # # TODO: how to add time to logfile

        # TODO: make it able to overwrite
        if not path.exists(self.file):
            with rosbag.Bag(self.file, 'w') as bag:
                # Initializing logfile with name of logfile
                string = String()
                string.data = self.file
                metadata_msg = String(data=self.logfile)
                bag.write('/metadata', metadata_msg)
                # bag.write('filename', string)

        with rosbag.Bag(self.logfile, 'a') as bag:
            # TODO: Add image logging
            #### Create CompressedIamge ####
            # img_msg = CompressedImage()
            msg_x = Float32()
            msg_x.data = x
            msg_y = Float32()
            msg_y.data = y
            msg_z = Float32()
            msg_z.data = z
            msg_theta = Float32()
            msg_theta.data = theta
            msg_reward = Float32()
            msg_reward.data = reward

            # TODO: respect existing conventions of duckietown rosbags
            # TODO: Which DUCKIEBOT_NAME to use?
            # Writing observed location and orientation to rosbag
            bag.write('/watchtower-sim/x-position', msg_x)
            bag.write('/watchtower-sim/y-position', msg_y)
            bag.write('/watchtower-sim/z-position', msg_z)
            bag.write('/watchtower-sim/theta-angle', msg_theta)
            bag.write('/watchtower-sim/reward', msg_reward)

            #     # Write perceived image to rosbag
            #     # TODO: Add observation
            #     # img_msg.format = "jpeg"
            #     # img_msg.data = np.array(cv2.imencode('.jpg', observation)[1]).tostring()
