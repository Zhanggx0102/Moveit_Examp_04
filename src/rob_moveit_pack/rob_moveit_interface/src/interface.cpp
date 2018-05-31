/*********************************************************************
Copyright (c) <2018>, <Shawn Zhang>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the <organization>.
4. Neither the name of the <organization> nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY <Shawn Zhang> ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <Shawn Zhang> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *********************************************************************/
// Created on: May 18th 2017
// Last change: Oct. 23 2017
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <time.h>
#include <sstream>
#include <pthread.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

/*--------------------------------------------------------------------------------------------------------*/
/*definitions and variables */
/*--------------------------------------------------------------------------------------------------------*/
using namespace moveit;

#define pi          3.1415926
#define degTorad 	pi/180
#define radTodeg 	180/pi

/*--------------------------------------------------------------------------------------------------------*/
/*function declaration                                                                                   */
/*--------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------------*/
/*main function                                                                                       */
/*--------------------------------------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    std::vector<double> L_Joint_goal;
    std::vector<double> R_Joint_goal;
    double left_movement[5];
    double right_movement[5];

	// Get the arm planning group
    moveit::planning_interface::MoveGroup l_plan_group("left_arm");
	moveit::planning_interface::MoveGroup r_plan_group("right_arm");

    //Show msg
    ROS_INFO("Left Planning Reference frame: %s", l_plan_group.getPlanningFrame().c_str());
    ROS_INFO("Left EndEffector Reference frame: %s", l_plan_group.getEndEffectorLink().c_str());
	ROS_INFO("Right Planning Reference frame: %s", r_plan_group.getPlanningFrame().c_str());
    ROS_INFO("Right EndEffector Reference frame: %s", r_plan_group.getEndEffectorLink().c_str());
    //

    //Get random target
    srand((unsigned)time(NULL)); 

    left_movement[0] = (rand()%(175-(-25)+1)) + (-25);
    left_movement[1] = (rand()%(65-(0)+1)) + (0);
    left_movement[2] = (rand()%(80-(-80)+1)) + (-80);
    left_movement[3] = (rand()%(90-(0)+1)) + (0);
    left_movement[4] = (rand()%(80-(-80)+1)) + (-80);
    printf("left_movement:[%f][%f][%f][%f][%f]\n",left_movement[0],left_movement[1],left_movement[2],left_movement[3],left_movement[4]);

    right_movement[0] = (rand()%(175-(-25)+1)) + (-25);
    right_movement[1] = (rand()%(65-(0)+1)) + (0);
    right_movement[2] = (rand()%(80-(-80)+1)) + (-80);
    right_movement[3] = (rand()%(90-(0)+1)) + (0);
    right_movement[4] = (rand()%(80-(-80)+1)) + (-80);
    printf("right_movement:[%f][%f][%f][%f][%f]\n",right_movement[0],right_movement[1],right_movement[2],right_movement[3],right_movement[4]);

    //set Velocity Factor (0,1]
    double Velocity_Factor = (double)((rand()%(100-(1)+1)) + (1))/100;
    l_plan_group.setMaxVelocityScalingFactor(Velocity_Factor);
    r_plan_group.setMaxVelocityScalingFactor(Velocity_Factor);
    printf("Velocity_Factor = [%f]\n", Velocity_Factor);

    //excute
    for(int i=0;i<5;i++)
    {
        L_Joint_goal.push_back(left_movement[i]*degTorad);
        R_Joint_goal.push_back(right_movement[i]*degTorad);
    }
    l_plan_group.setJointValueTarget (L_Joint_goal);
    r_plan_group.setJointValueTarget (R_Joint_goal);
    l_plan_group.move();
    r_plan_group.move();

    L_Joint_goal.clear();
    R_Joint_goal.clear();
	

    ros::shutdown();
    return 0;
}









