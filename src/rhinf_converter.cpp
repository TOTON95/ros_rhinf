#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <iostream>

#define PI 3.141592653589793238462

double heading = 0;
double s_t = 0;
std::vector<double> dim;
double vel_x,vel_y,vel_z = 0;
double prev_p_x,prev_p_y,prev_p_z = 0;
std_msgs::Float64MultiArray out_msg_x, ref_msg_x, out_msg_y, ref_msg_y, out_msg_z, ref_msg_z;
ros::Time s0_t, s1_t;
ros::Publisher out_x,out_y,out_z;
ros::Publisher ref_x,ref_y,ref_z;

double stp_x = 0;
double stp_y = 0;

//double a = 0.50;
double a = 1.00;
double prev_vel_x,prev_vel_y,prev_vel_z = 0.00;

struct v_object                                                         //Structure that describes the properties of the object
{
        double _posX,_posY,_posZ;                                       //Position of the drone
        double _errorX,_errorY,_errorZ;                                 //Error of the position of the drone
        double _orX,_orY,_orZ,_orW;                                     //Orientation of the drone
        double _roll,_pitch,_yaw,_yawRAD;                               //Roll,Pitch,Yaw (degrees), Yaw (radians)
        double _cmdX,_cmdY,_cmdZ,_cmdYAW;                               //Command values
        double rot_cmd_x,rot_cmd_y;                                     //Position in the rotated matrix
        double _velX,_velY,_velZ,_velYAW;                               //Velocities
        double abs_x,abs_y;                                             //Absolute position in X and Y
        double angle_res,angle_arc;                                     //Angle resultant, angle of the arc
}drone;

void getDronePos(const geometry_msgs::TransformStamped::ConstPtr& pos)  //Function to obtain the position from the vicon system
{
        drone._posX = pos->transform.translation.x;                     //Position in X
        drone._posY = pos->transform.translation.y;                     //Position in Y
        drone._posZ = pos->transform.translation.z;                     //Position in Z
        drone._orX = pos->transform.rotation.x;                         //Rotation in X
        drone._orY = pos->transform.rotation.y;                         //Rotation in Y
        drone._orZ = pos->transform.rotation.z;                         //Rotation in Z
        drone._orW = pos->transform.rotation.w;                         //Rotation in W

        tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(drone._roll,drone._pitch,drone._yawRAD);               //Get the Roll, Pitch, Yaw (Radians)
        drone._yaw = drone._yawRAD*(180/PI);                            //Convert the Yaw (Radians) into Yaw (Degrees)
        heading = drone._yaw;                                           //Set the heading of the drone

        drone.abs_x = drone._posX;                                      //Set the absolute position of the drone in X
        drone.abs_y = drone._posY;                                      //Set the absolute position of the drone in Y
}

void getSetpointX(const std_msgs::Float64::ConstPtr& msg)
{
	stp_x = msg->data;	
}

void getSetpointY(const std_msgs::Float64::ConstPtr& msg)
{
	stp_y = msg->data;	
}


//Discrete velocity 
void velocity_w_filter()
{
	//Calculate velocity
	vel_x = (1-a)*prev_vel_x+a*(drone._posX - prev_p_x);
	vel_y = (1-a)*prev_vel_y+a*(drone._posY - prev_p_y);
	vel_z = (1-a)*prev_vel_z+a*(drone._posZ - prev_p_z);

	//Set messages
	out_msg_x.data[0] = drone._posX;
	out_msg_x.data[1] = vel_x;
	out_msg_y.data[0] = drone._posY;
	out_msg_y.data[1] = vel_y;
	out_msg_z.data[0] = drone._posZ;
	out_msg_z.data[1] = vel_z;

	ref_msg_x.data[0] = stp_x;
	ref_msg_x.data[1] = 0;
	ref_msg_y.data[0] = stp_y;
	ref_msg_y.data[1] = 0;
	ref_msg_z.data[0] = 0;
	ref_msg_z.data[1] = 0;
	
	//Publish messages
	out_x.publish(out_msg_x);
	ref_x.publish(ref_msg_x);
	out_y.publish(out_msg_y);
	ref_y.publish(ref_msg_y);
	out_z.publish(out_msg_z);
	ref_z.publish(ref_msg_z);

	//Record previous values
	prev_p_x = drone._posX;
	prev_vel_x = vel_x;
	prev_p_y = drone._posY;
	prev_vel_y = vel_y;
	prev_p_z = drone._posZ;
	prev_vel_z = vel_z;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"rhinf_converter"); //Initiates the ROS node
	ros::NodeHandle n;
	ros::Subscriber drone_sub;
	ros::Subscriber ros_ref_x,ros_ref_y;

	if(!n.getParam("sample_time",s_t))
	{
		ROS_ERROR_STREAM("Failed to get sample time. Terminating.");
                ros::shutdown();
                exit(EXIT_FAILURE);
	}

	if(!n.getParam("AN_dim",dim))
	{
		ROS_ERROR_STREAM("Failed to get the state dimensions. Terminating.");
                ros::shutdown();
                exit(EXIT_FAILURE);
	}

	drone_sub = n.subscribe("/vicon/Mambo_5/Mambo_5",1000,getDronePos);
	ros_ref_x = n.subscribe("/setpoint_pos_X",1000,getSetpointX);
	ros_ref_y = n.subscribe("/setpoint_pos_Y",1000,getSetpointY);
	out_x = n.advertise<std_msgs::Float64MultiArray>("/rhinf_st_x",1000);
	ref_x = n.advertise<std_msgs::Float64MultiArray>("/rhinf_ref_x",1000);
	out_y = n.advertise<std_msgs::Float64MultiArray>("/rhinf_st_y",1000);
	ref_y = n.advertise<std_msgs::Float64MultiArray>("/rhinf_ref_y",1000);
	out_z = n.advertise<std_msgs::Float64MultiArray>("/rhinf_st_z",1000);
	ref_z = n.advertise<std_msgs::Float64MultiArray>("/rhinf_ref_z",1000);

	out_msg_x.data.resize((int)dim.back());
	ref_msg_x.data.resize((int)dim.back());
	out_msg_y.data.resize((int)dim.back());
	ref_msg_y.data.resize((int)dim.back());
	out_msg_z.data.resize((int)dim.back());
	ref_msg_z.data.resize((int)dim.back());
	
	out_msg_x.data[0] = drone._posX;
	out_msg_x.data[1] = vel_x;
	prev_p_x = drone._posX;

	out_msg_y.data[0] = drone._posY;
	out_msg_y.data[1] = vel_y;
	prev_p_y = drone._posY;

	out_msg_z.data[0] = drone._posZ;
	out_msg_z.data[1] = vel_z;
	prev_p_z = drone._posZ;

	ref_msg_x.data[0] = stp_x;
	ref_msg_x.data[1] = 0;
	ref_msg_y.data[0] = stp_y;
	ref_msg_y.data[1] = 0;
	ref_msg_z.data[0] = 0;
	ref_msg_z.data[1] = 0;

	out_x.publish(out_msg_x);
	ref_x.publish(ref_msg_x);
	out_y.publish(out_msg_y);
	ref_y.publish(ref_msg_y);
	out_z.publish(out_msg_z);
	ref_z.publish(ref_msg_z);


	while(n.ok())
	{
		ros::spinOnce();
		velocity_w_filter();
		ros::Duration(s_t).sleep();
	}
}
