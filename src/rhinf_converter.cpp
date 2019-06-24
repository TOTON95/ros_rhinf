#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <vector>

#define PI 3.141592653589793238462

double heading = 0;
double s_t = 0;
std::vector<double> dim;
double vel = 0;
double prev_p = 0;
std_msgs::Float64MultiArray out_msg_x, ref_msg_x;
ros::Time s0_t, s1_t;
ros::Publisher out;
ros::Publisher ref_out;

double a = 0.50;
double prev_vel = 0.00;

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

void setOutMsg()
{
	double vel = drone._posX - prev_p;
	out_msg_x.data[0] = drone._posX;
        out_msg_x.data[1] = vel;
	out.publish(out_msg_x);
	ref_out.publish(ref_msg_x);
        prev_p = drone._posX;
}

void velocity_w_filter()
{
	double vel = (1-a)*prev_vel+a*(drone._posX - prev_p);
	out_msg_x.data[0] = drone._posX;
	out_msg_x.data[1] = vel*10;
	out.publish(out_msg_x);
	ref_out.publish(ref_msg_x);
	prev_p = drone._posX;
	prev_vel = vel;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"rhinf_converter"); //Initiates the ROS node
	ros::NodeHandle n;
	ros::Subscriber drone_sub;

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
	out = n.advertise<std_msgs::Float64MultiArray>("/rhinf_st",1000);
	ref_out = n.advertise<std_msgs::Float64MultiArray>("/rhinf_ref",1000);

	out_msg_x.data.resize((int)dim.back());
	ref_msg_x.data.resize((int)dim.back());
	
	out_msg_x.data[0] = drone._posX;
	out_msg_x.data[1] = vel;
	prev_p = drone._posX;

	ref_msg_x.data[0] = 0;
	ref_msg_x.data[1] = 0;

	out.publish(out_msg_x);
	ref_out.publish(ref_msg_x);


	while(n.ok())
	{
		ros::spinOnce();
		//setOutMsg();
		velocity_w_filter();
		ROS_INFO("POS: %lf VEL: %lf",out_msg_x.data[0],out_msg_x.data[1]);
		ros::Duration(s_t).sleep();
	}
}
