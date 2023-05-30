# Artificial_Potential_Fields
ROS package for reactive obstacle avoidance using artificial potential fields.

2D APF

![image](https://github.com/linden713/artificial_potential_fields/blob/main/result/APF)

Make sure you have following topics:
Subscribers: "/odom" "/attractive_velocity"(geometry_msgs::Twist) "/scan"

And will publish:
Publishers: "/cmd_vel"...

run:

      roslaunch artificial_potential_fields APF.launch

or:

      roslaunch artificial_potential_fields APF_unomni.launch (moving forward demo)
      


Fork from: https://github.com/andriyukr/artificial_potential_fields

Related vedio: https://www.bilibili.com/video/BV1bG41137G2/ (bilibili)
