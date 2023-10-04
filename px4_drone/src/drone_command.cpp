#include <drone.h>
#include <convert.h>

void Drone::Arm()
{
    if (CheckArm(true)) { return; }

    mavros_msgs::CommandBool cmd_arm;
    cmd_arm.request.value = true;

    if (!client_arm.call(cmd_arm))
        ROS_WARN("Can't transfer arming command");
    else
        ROS_INFO("Try arming ...");
}


// Change PX4 mode as parameter
void Drone::ChangeMode(std::string mode)
{
    if (CheckMode(mode)) { return ;}

    mavros_msgs::SetMode cmd_mode;
    cmd_mode.request.base_mode = 0;
    cmd_mode.request.custom_mode = mode;

    if (!client_mode.call(cmd_mode)) { 
        ROS_WARN("Can't tranfer mode command"); 
    }
    else { 
        ROS_INFO("Try to change mode to [%s]", mode.c_str()); 
    }

}


void Drone::SetPlanner()
{
    enable_planner = !enable_planner;
    
    if (enable_planner)
        ROS_INFO("Planner enable");
    else
        ROS_INFO("Planner disable");

    std_msgs::Bool trigger_msg;
    trigger_msg.data = enable_planner;

    pub_planner_trigger.publish(trigger_msg);
}


void Drone::Land()
{
    if (!CheckFlying()) { return; }

    ChangeMode("AUTO.LAND"); // Change mode as AUTO.LAND
}


void Drone::Hover()
{
    ROS_INFO_DELAYED_THROTTLE(5, "hover... [%lf][%.2f, %.2f, %.2f]", RadianToDegree(current_yaw),
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

    pub_setpoint.publish(set_localRaw);
}



// Set local position as current position
void Drone::SetCurrentPose()
{
    set_localRaw.position = current_pose.pose.position;
}

void Drone::SetCurrentYaw()
{
    set_localRaw.yaw = current_yaw;
}


void Drone::MoveLinear(double pose_x, double pose_y, double pose_z)
{
    // Stop
    if (pose_x == 0 && pose_y == 0 && pose_z == 0)
    { SetCurrentPose(); }

    else
    {
        double goal_position[3] = {pose_x, pose_y, pose_z};

        if (pose_x > 0)
        {
            goal_position[0] = pose_x * cos(current_yaw);
            goal_position[1] = pose_x * sin(current_yaw);
        }
        else if (pose_x < 0)
        {
            goal_position[0] = abs(pose_x) * -cos(current_yaw);
            goal_position[1] = abs(pose_x) * -sin(current_yaw);
        }

        if (pose_y > 0)
        {
            goal_position[0] = pose_y * sin(current_yaw);  //-cos(current_yaw + M_PI_2);
            goal_position[1] = pose_y * -cos(current_yaw); //-sin(current_yaw + M_PI_2);
        }
        else if (pose_y < 0)
        {
            goal_position[0] = abs(pose_y) * -sin(current_yaw); // cos(current_yaw + M_PI_2);
            goal_position[1] = abs(pose_y) * cos(current_yaw);  // sin(current_yaw + M_PI_2);
        }

        set_localRaw.position.x += goal_position[0];
        set_localRaw.position.y += goal_position[1];
        set_localRaw.position.z += goal_position[2];

        ROS_INFO("Move linear : %.2f, %.2f, %.2f", set_localRaw.position.x, set_localRaw.position.y, set_localRaw.position.z);

    }
}

void Drone::MoveAngular(double degree)
{
    if (degree == 0)
        SetCurrentYaw();

    else
    {
        double goal_radian = current_yaw + DegreeToRadian(degree);

        if (goal_radian >= M_PI * 2)
            goal_radian -= M_PI * 2;
            
        else if (goal_radian <= 0)
            goal_radian += M_PI * 2;

        set_localRaw.yaw = goal_radian;
        
        ROS_INFO("Move angular : [d: %lf][r: %lf]", RadianToDegree(goal_radian), goal_radian);
    }
}