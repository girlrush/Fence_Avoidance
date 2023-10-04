#include <drone.h>
#include "keyboard.h"

Drone::Drone(const ros::NodeHandle& _nodeHandle):
    nh(_nodeHandle),
    // Mavros
    sub_state(nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &Drone::CallbackState, this)),
    sub_pose(nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &Drone::CallbackPose, this)),
    sub_planner(nh.subscribe<mavros_msgs::PositionTarget>("/planner/waypoint", 10, &Drone::CallbackPlanner, this)),

    pub_setpoint(nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10)),
    pub_planner_trigger(nh.advertise<std_msgs::Bool>("/planner/trigger", 10)),

    client_arm(nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming")),
    client_mode(nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"))

{
    set_localRaw.header.frame_id = "map";
    set_localRaw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
}

Drone::~Drone() { close_keyboard(); }


// Set keyboard teleop mapping
// If you want to add a teleop key, modify this
void Drone::TeleopMapping(std::map<int, std::function<void()>>& keymap)
{
    // Task
    keymap['q'] = std::bind(&Drone::_Shutdown, this);
    keymap['g'] = std::bind(&Drone::Land, this);
    keymap['p'] = std::bind(&Drone::Arm, this);
    keymap['o'] = std::bind(&Drone::ChangeMode, this, "OFFBOARD");

    // Up & Down
    keymap['u'] = std::bind(&Drone::MoveLinear, this, 0, 0, 1);
    keymap['j'] = std::bind(&Drone::MoveLinear, this, 0, 0, -1);

    // Move
    keymap['w'] = std::bind(&Drone::MoveLinear, this, 1, 0, 0);
    keymap['a'] = std::bind(&Drone::MoveLinear, this, 0, -1, 0);
    keymap['s'] = std::bind(&Drone::MoveLinear, this, 0, 0, 0);
    keymap['d'] = std::bind(&Drone::MoveLinear, this, 0, 1, 0);
    keymap['x'] = std::bind(&Drone::MoveLinear, this, -1, 0, 0);

    // Turn
    keymap['1'] = std::bind(&Drone::MoveAngular, this, 10);
    keymap['0'] = std::bind(&Drone::MoveAngular, this, 0);
    keymap['3'] = std::bind(&Drone::MoveAngular, this, -10);

    keymap['m'] = std::bind(&Drone::SetPlanner, this);

}


void Drone::_Run()
{
    if (ReadParam()) 
    { 
        ROS_WARN("Failed to read parameter");
        _Shutdown();
        return;
    }
    if (!CheckConnection())
    {
        ROS_WARN("Failed to connect");
        _Shutdown();
        return; 
    }
    
    TeleopMapping(teleop_map);


    ROS_INFO("Keyboard Teleop Start!");

    init_keyboard();

    while (ros::ok())
    {
        if (_kbhit())
        {
            int ch = _getch();
            if (teleop_map.count(ch) > 0)
            {
                teleop_map[ch]();
            }
        }

        if (current_state.armed)
            Hover();

        ros::spinOnce();
        rate.sleep();
    }
}


void Drone::_Shutdown()
{
    ROS_INFO("Shutdown drone node");
    ros::shutdown();
}