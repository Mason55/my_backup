#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>// Note: "Action" is appended
#include <control_msgs/JointTolerance.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;


void doneCb(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result )
{
  ROS_INFO("[State Result]: %s", state.toString().c_str());
  ROS_INFO("The Action has been completed");
}
// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}
void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
//   ROS_INFO("[Feedback] image n.%d received", nImage);
//   ++nImage;
}


     



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_tra_control");
    TrajClient  client ("pos_based_pos_traj_controller/follow_joint_trajectory/",true);
    client.waitForServer();

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.header.stamp = ros::Time::now()+ ros::Duration(1.0);
    goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("elbow_joint");
    goal.trajectory.joint_names.push_back("wrist_1_joint");
    goal.trajectory.joint_names.push_back("wrist_2_joint");
    goal.trajectory.joint_names.push_back("wrist_3_joint");

    goal.trajectory.points.resize(1);

    int ind = 0;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = 0;
    goal.trajectory.points[ind].positions[1] = -1.;
    goal.trajectory.points[ind].positions[2] = -2;
    goal.trajectory.points[ind].positions[3] = -0.85;
    goal.trajectory.points[ind].positions[4] = 1.550;
    goal.trajectory.points[ind].positions[5] = -0.343;

    // //Velocities
    // goal.trajectory.points[ind].velocities.resize(6);
    // for (size_t a = 0; a < 6; ++a){
    //     goal.trajectory.points[ind].velocities[a] = 0.3;
    // }

    // goal.trajectory.points[ind].accelerations.resize(6);
    // for (size_t b = 0; b < 6; ++b) {
    //     goal.trajectory.points[ind].accelerations[b] = 0.2;
    // }

    // goal.trajectory.points[ind].time_from_start + ros::Duration(1.0);
    

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  
    ros::Rate loop_rate(1);
    actionlib::SimpleClientGoalState state_result = client.getState();
    ROS_INFO("[State Result]: %s", state_result.toString().c_str());

    ros::spin();
    return 0;
}