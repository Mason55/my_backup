#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>// Note: "Action" is appended
#include <control_msgs/JointTolerance.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>
#include <vector>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class UrRobotArm
{
private:
    TrajClient * traj_client_;
    // control_msgs::FollowJointTrajectoryGoal goal;
    // control_msgs::FollowJointTrajectoryActionFeedback feedback;
public:
    UrRobotArm()
    {
        traj_client_ = new TrajClient("vel_based_pos_traj_controller/follow_joint_trajectory/",true);
        // traj_client_ = new TrajClient("pos_based_pos_traj_controller/follow_joint_trajectory/",true);

        while(!traj_client_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
    }
    //! Clean up the action client
    ~UrRobotArm()
    {
        delete traj_client_;
    }
    //! Sends the command to start a given trajectory
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
    {    // When to start the trajectory: 1s from now
    //header文件的填写
        goal.trajectory.header.stamp = ros::Time::now()+ ros::Duration(1.0);
        traj_client_->sendGoal(goal);
    }

    // void getCurrentJointAngle(control_msgs::FollowJointTrajectoryActionFeedback)//我不确定这里是不是使用feedback
    // {

    // }

    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
    {   //our goal variable
        control_msgs::FollowJointTrajectoryGoal goal;
        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("elbow_joint");
        goal.trajectory.joint_names.push_back("wrist_1_joint");
        goal.trajectory.joint_names.push_back("wrist_2_joint");
        goal.trajectory.joint_names.push_back("wrist_3_joint");
        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(2);
         // First trajectory point 第一个点的位置需要使用成当前的位置的信息
        // Positions
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(6);
        goal.trajectory.points[ind].positions[0] = 0.18;
        goal.trajectory.points[ind].positions[1] = -1.34;
        goal.trajectory.points[ind].positions[2] = -2.31;
        goal.trajectory.points[ind].positions[3] = -0.37;
        goal.trajectory.points[ind].positions[4] = 0.45;
        goal.trajectory.points[ind].positions[5] = 0.23;
        // goal.trajectory.points[ind].positions[6] = 0.0;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(6);
        for (size_t a = 0; a < 6; ++a)
        {
        goal.trajectory.points[ind].velocities[a] = 0.0;
        }
        
        goal.trajectory.points[ind].accelerations.resize(6);
        for (size_t b = 0; b < 6; ++b)
        {
        goal.trajectory.points[ind].accelerations[b] = 0.0;
        }
        for (size_t c = 0; c < 6; ++c)
        {
        goal.trajectory.points[ind].effort[c] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);
         // Second trajectory point
        // Positions
        ind += 1;
        goal.trajectory.points[ind].positions.resize(6);
        goal.trajectory.points[ind].positions[0] = 0.2;
        goal.trajectory.points[ind].positions[1] = -1.2;
        goal.trajectory.points[ind].positions[2] = -2.31;
        goal.trajectory.points[ind].positions[3] = -0.6;
        goal.trajectory.points[ind].positions[4] = 0.5;
        goal.trajectory.points[ind].positions[5] = 0.5;
        // goal.trajectory.points[ind].positions[6] = 0.0;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(6);
        for (size_t a = 0; a < 6; ++a)
        {
        goal.trajectory.points[ind].velocities[a] = 0.05;
        }
        
        goal.trajectory.points[ind].accelerations.resize(6);
        for (size_t b = 0; b < 6; ++b)
        {
        goal.trajectory.points[ind].accelerations[b] = 0.0;
        }
        for (size_t c = 0; c < 6; ++c)
        {
        goal.trajectory.points[ind].effort[c] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

        return goal;
    }
     //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
        return traj_client_->getState();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_tra_control");
    UrRobotArm arm;
    arm.startTrajectory(arm.armExtensionTrajectory());

    while(!arm.getState().isDone() && ros::ok())
    {
        usleep(50000);
    }

    return 0;
}