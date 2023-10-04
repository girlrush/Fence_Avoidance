#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <map>

class Drone{

    private:
        ros::NodeHandle nh;
    
        ros::Publisher pub_setpoint;
        ros::Publisher pub_planner_trigger;

        ros::Subscriber sub_state;
        ros::Subscriber sub_pose;
        ros::Subscriber sub_planner;

        ros::ServiceClient client_arm;
        ros::ServiceClient client_mode;

        mavros_msgs::PositionTarget set_localRaw;
        geometry_msgs::PoseStamped current_pose;

        double current_yaw = 0; //Radian

        mavros_msgs::State current_state;

        ros::Rate rate = ros::Rate(10.0);

        double takeoff_height = 2.0;
        bool is_takeoff = false;
        bool enable_planner = false;

        std::map<int, std::function<void()>> teleop_map;

        // Initialize
        bool ReadParam();
        void TeleopMapping(std::map<int, std::function<void()>>&);

        // Checking Function
        bool CheckConnection();
        bool CheckArm(bool requested_arm);
        bool CheckFlying();
        bool CheckMode(std::string mode);

        // Command
        void Arm();
        void ChangeMode(std::string mode);
        void Land();
        void Hover();
        
        // Movement
        void SetCurrentPose();
        void SetCurrentYaw();
        void MoveLinear(double pose_x, double pose_y, double pose_z);
        void MoveAngular(double degree);

        void CallbackState(const mavros_msgs::State::ConstPtr& state_msg);
        void CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

        void SetPlanner();
        void CallbackPlanner(const mavros_msgs::PositionTarget::ConstPtr& planner_msg);


    public:
        explicit Drone(const ros::NodeHandle& _nodeHandle);
        ~Drone();
        void _Shutdown();
        void _Run();
};