#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <iostream>
#include <string>

#define PI 3.141592653589793238462

double heading = 0;
double s_t = 0;
std::vector<double> dim;
double vel = 0;
double prev_p = 0;
std_msgs::Float64MultiArray out_msg, ref_msg;
ros::Time s0_t, s1_t;

//ROS Publishers
ros::Publisher out;
ros::Publisher ref;

//Setpoint value
double stp = 0;

//State value
double st = 0;

//double a = 0.50;
double a = 1.00;
double prev_vel = 0.00;


//Get setpoint
void getSetpoint(const std_msgs::Float64::ConstPtr& msg)
{
    stp = msg->data;	
}

//Get state
void getState(const std_msgs::Float64::ConstPtr& msg)
{
    st = msg->data;	
}

//Discrete velocity 
void velocity_w_filter()
{
    //Calculate velocity
    vel = (1-a)*prev_vel+a*(st - prev_p);

    //Set messages
    out_msg.data[0] = st;
    out_msg.data[1] = vel;

    ref_msg.data[0] = stp;
    ref_msg.data[1] = 0;

    //Publish messages
    out.publish(out_msg);
    ref.publish(ref_msg);

    //Record previous values
    prev_p = st;
    prev_vel = vel;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"rhinf_converter",ros::init_options::AnonymousName); //Initiates the ROS node
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ros::Subscriber ros_ref, ros_st;

    std::string reference_topic;
    std::string state_topic;
    std::string out_reference_topic;
    std::string out_state_topic;

    //Get sample time
    if(!nh.getParam("sample_time",s_t))
    {
        ROS_ERROR_STREAM("Failed to get sample time. Terminating.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    //Get dimensions of state
    if(!nh.getParam("AN_dim",dim))
    {
        ROS_ERROR_STREAM("Failed to get the state dimensions. Terminating.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    //Get name of state topic
    if(!nh.getParam("state_topic",state_topic))
    {
        ROS_ERROR_STREAM("Failed to get the state topic. Terminating.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    //Get name of reference topic
    if(!nh.getParam("reference_topic",reference_topic))
    {
        ROS_ERROR_STREAM("Failed to get the reference topic. Terminating.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    //Get name of out_state topic
    if(!nh.getParam("out_state_topic",out_state_topic))
    {
        ROS_ERROR_STREAM("Failed to get the out_state topic. Terminating.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    //Get name of reference topic
    if(!nh.getParam("out_reference_topic",out_reference_topic))
    {
        ROS_ERROR_STREAM("Failed to get the out_reference topic. Terminating.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }


    ros_st = n.subscribe(state_topic,1000,getState);
    ros_ref = n.subscribe(reference_topic,1000,getSetpoint);
    out = n.advertise<std_msgs::Float64MultiArray>(out_state_topic,1000);
    ref = n.advertise<std_msgs::Float64MultiArray>(out_reference_topic,1000);

    //Wait for incoming messages
    while(ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64>(reference_topic,ros::Duration(1.)))
        ROS_WARN_STREAM("Waiting for first reference message [topic: " + reference_topic+ "]");

    while(ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64>(state_topic,ros::Duration(1.)))
        ROS_WARN_STREAM("Waiting for first state message [topic]");

    out_msg.data.resize((int)dim.back());
    ref_msg.data.resize((int)dim.back());

    out_msg.data[0] = st;
    out_msg.data[1] = vel;
    prev_p = st;

    ref_msg.data[0] = stp;
    ref_msg.data[1] = 0;

    out.publish(out_msg);
    ref.publish(ref_msg);

    ros::Rate r(1/s_t);

    while(n.ok())
    {
        ros::spinOnce();
        velocity_w_filter();
        r.sleep();
    }
}
