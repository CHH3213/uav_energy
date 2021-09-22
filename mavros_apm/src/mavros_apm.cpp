#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std ;

// a simple callback which to save the current state
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void myDelay(int delaySecs)
{
	int stTime=ros::Time::now().toSec();
	int edTime=stTime+delaySecs;
	int curTime=ros::Time::now().toSec();
//	cout<<"stTime="<<stTime<<endl;
//	cout<<"edTime="<<edTime<<endl;
	while(curTime<=edTime)
	{
		curTime=ros::Time::now().toSec();
	//	cout<<"curTime="<<curTime<<endl;
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
	
	cout<<"offboard start"<<endl;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
	{
        ros::spinOnce();	//	wait for the callback which refresh current state
        rate.sleep();
    }
	cout<<"FCU connected"<<endl;
	myDelay(3);
	
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
	cout<<"send setpoints fin"<<endl;
	myDelay(3);

    mavros_msgs::SetMode offb_set_mode;
    //	offb_set_mode.request.custom_mode = "OFFBOARD";
	offb_set_mode.request.custom_mode = "GUIDED";
	cout<<"set mode fin"<<endl;
	myDelay(3);
	

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	cout<<"arm th"<<endl;
	myDelay(3);

    ros::Time last_request = ros::Time::now();

	//	keep Vehicle armed and mode in "GUIDED" 
    while(ros::ok()){

        //	if( current_state.mode != "OFFBOARD" &&
		if( current_state.mode != "GUIDED" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))	
		{
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
			{
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
		else 
		{
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


         local_pos_pub.publish(pose);

		if( current_state.mode == "GUIDED" &&
            current_state.armed)
		{
			
		}

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

