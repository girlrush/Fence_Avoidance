#include <drone.h>
#include <convert.h>

void Drone::CallbackState(const mavros_msgs::State::ConstPtr& state_msg)
{
    if (current_state.mode != state_msg->mode){
        ROS_INFO("Mode Changed [%s->%s]", current_state.mode.c_str(), state_msg->mode.c_str());
        
        if (state_msg->mode == "OFFBOARD")
        {   
            // Flying & Arming+offboard => set current Position & set default height for takeoff
            if (current_state.armed){
                SetCurrentPose();
                SetCurrentYaw();
                
                // Check height and Set height
                if (current_pose.pose.position.z <= 0.4)
                    set_localRaw.position.z = takeoff_height;
            }
            else{
                // 
                SetCurrentPose();
                SetCurrentYaw();
                set_localRaw.position.z = takeoff_height;
            }
        }
    }

    // Check armming and chage state
    if (current_state.armed != state_msg->armed)
    {
        if (state_msg->armed)
            ROS_INFO("Now armed");
        else
            ROS_INFO("Now disarmed");
    }

    // Check failsafe and shutdown the code
    if (state_msg->system_status >= 6){
        if (state_msg->system_status <= 8){
            ROS_ERROR("[EMERGENCY] Code Num : %d", state_msg->system_status);
            _Shutdown();
            return;
            
        }
        else{
            ROS_ERROR("[EMERGENCY] Dump Value : %d", state_msg->system_status);
            _Shutdown();
            return;
        }
    }

    current_state = *state_msg;

}

void Drone::CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    current_pose = *pose_msg;
    
    auto e = QuaternionToEuler(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w); 
    current_yaw = e[2];
}

void Drone::CallbackVelocity(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)
{
    Eigen::Vector3d diff_vel;
    
    current_velocity(0) = vel_msg->twist.linear.x;
    current_velocity(1) = vel_msg->twist.linear.y;
    current_velocity(2) = vel_msg->twist.linear.z;
    
    if (current_velocity.norm() >= 2.0)
    {
        ChangeMode("AUTO.LAND");
        ROS_ERROR("[EMERGENCY] High speed (%lf, %lf, %lf)", current_velocity(0), current_velocity(1), current_velocity(2));
        return;
    }
}


void Drone::CallbackPlanner(const mavros_msgs::PositionTarget::ConstPtr& planner_msg)
{
    set_localRaw = *planner_msg;
}

