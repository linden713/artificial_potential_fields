# Artificial_Potential_Fields
ROS package for reactive obstacle avoidance using artificial potential fields.

2D APF

![image](https://github.com/linden713/artificial_potential_fields/blob/main/result/APF)

Make sure you have following topics:
Subscribers: "/odom" "/attractive_velocity"(geometry_msgs::Twist) "/scan"

And will publish:
Publishers: "/cmd_vel"...
# Step to run the project
1. Prepare your simulation environment:
   You can install a same environment like mine by:
   
         git clone https://github.com/6-robot/wpr_simulation.git
   
   or choose a robot model you like (Some parameter need to be tune base on different model)
   
3. Open your simulation environment:

         roslaunch wpr_simulation wpb_simple.launch

3. Run APF.launch
run:

         roslaunch artificial_potential_fields APF.launch

      or:

               roslaunch artificial_potential_fields APF_unomni.launch (moving forward demo)
      
4. Begin to control:


            rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/attractive_velocity

5. Press "x" to decrease the linear speed to 0.2m/s,which is the highly recommend!

# Other information

Fork from: https://github.com/andriyukr/artificial_potential_fields

Related vedio: https://www.bilibili.com/video/BV1bG41137G2/ (bilibili)
