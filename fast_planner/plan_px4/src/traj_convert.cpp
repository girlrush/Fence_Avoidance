#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>

#include <std_msgs/Bool.h>

ros::Publisher pub_converted_traj;
bool enable_planner = false;


void CallbackPlannerTraj(const quadrotor_msgs::PositionCommandConstPtr& traj_msg)
{
    if (!enable_planner)
        return;
    
    mavros_msgs::PositionTarget traj_proc;

    traj_proc.header = traj_msg->header;
    traj_proc.coordinate_frame = 1;
    
    traj_proc.position = traj_msg->position;
    traj_proc.velocity = traj_msg->velocity;
    traj_proc.acceleration_or_force = traj_msg->acceleration;
    traj_proc.yaw = traj_msg->yaw;

    pub_converted_traj.publish(traj_proc);
}

void CallbackPlannerTrigger(const std_msgs::BoolConstPtr& trigger_msg)
{
    if (trigger_msg->data)
    {
        enable_planner = trigger_msg->data;
    }
    else
    {
        enable_planner = trigger_msg->data;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_convert_node");
    ros::NodeHandle nh;

    
    pub_converted_traj = nh.advertise<mavros_msgs::PositionTarget>("/mavros_traj", 1);

    ros::Subscriber sub_traj = nh.subscribe<quadrotor_msgs::PositionCommand>("/planner_traj", 10, &CallbackPlannerTraj);
    ros::Subscriber sub_trigger = nh.subscribe<std_msgs::Bool>("/trigger", 10, &CallbackPlannerTrigger);

    
    ros::spin();

    return 0;
}