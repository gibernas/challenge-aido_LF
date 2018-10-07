# challenges-aido1_dummy_sim


We need gym-duckietown-agent (https://github.com/duckietown/gym-duckietown-agent)


To run lane following (LF) task type:

docker-compose -f docker-compose-lf.yml pull && \
docker-compose -f docker-compose-lf.yml up

(This leads to an error because python is misplaced:
/opt/conda/bin/python agent/agent.py --no-render # instead of path /usr/local/bin/python)

This runs two images:
- The duckietown gym server
- The duckietown gym agent (the file agent.py uses random actions and only serves as a template)
- TODO: Need to add duckietown logger as another container to run

TODO - Logging container:
- Need dependency on ROS
- Start by logging perceived images and communicated commands/actions
- Save in rosbag

Something like:
try:
          #### Create CompressedIamge ####
          img_msg = CompressedImage()

          # TODO: fix timing issue
          # img_msg.header.stamp = 0 # rospy.get_rostime() or rospy.Time.now()

          # Write perceived image to rosbag
          img_msg.format = "jpeg"
          img_msg.data = np.array(cv2.imencode('.jpg', observation)[1]).tostring()

          # Write wheel commands to rosbag
          cmd_msg = WheelsCmdStamped()
          cmd_msg.vel_left = action[1]
          cmd_msg.vel_right = action[0]

          # Writing
          #TODO: fix substitution BOTNAME
          bag.write("/BOTNAME/camera_node/image/compressed", img_msg)
          bag.write("/BOTNAME/wheels_driver_node/wheels_cmd_executed", cmd_msg)

      finally:
          bag.close()
