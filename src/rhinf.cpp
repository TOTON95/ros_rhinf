#include <ros_rhinf/rhinf.h>

using namespace rh;

rhinfObject::rhinfObject()
{
	ros::NodeHandle node;
	ros::NodeHandle n_priv("~");

	while (ros::ok() && ros::Time(0) == ros::Time::now())
	{
		ROS_INFO("Controller ready and spinning, waiting non-zero");
		sleep(1);	
	}
	
	n_priv.param<double>("downsampling",_downsampling, 17);
	n_priv.param<double>("sample_time", _sample_time,0.001);
	n_priv.param<double>("saturation", _saturation, 1);
	n_priv.param<std::string>("topic_controller", s_ctl, "control_effort");
	n_priv.param<std::string>("topic_state", s_state, "state");
	n_priv.param<std::string>("topic_ref", s_ref, "reference");

	//Print Parameters
	print_param();

	//Init publishers and subscribers
	_control_effort_pub = node.advertise<std_msgs::Float64>(s_ctl, 1);

	ros::Subscriber state_sub = node.subscribe(s_state, 1, &rhinfObject::stateCallback, this);
	ros::Subscriber ref_sub = node.subscribe(s_ref, 1, &rhinfObject::refCallback, this);

	if(!state_sub || !ref_sub)
	{
		ROS_ERROR_STREAM("Subscriber phase failed. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}


	//Wait for incoming messages
	while( ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64>(s_ref, ros::Duration(10.)))
		ROS_WARN_STREAM("Waiting for first reference message");
	while( ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64>(s_state, ros::Duration(10.)))
		ROS_WARN_STREAM("Waiting for first state message");	

	//Infinite cycle until
	while(ros::ok())
	{
		calc();
		ros::spinOnce();

		ros::Duration(_sample_time).sleep();
	}

};

void rhinfObject::calc()
{
	if(need_to_refresh)
	{
		need_to_refresh = false;
	}
}

void rhinfObject::getParams()
{
}

void rhinfObject::print_param()
{
}

void rhinfObject::stateCallback(const std_msgs::Float64& state_msg)
{
	_state = state_msg.data;
	need_to_refresh = true;
}

void rhinfObject::refCallback(const std_msgs::Float64& ref_msg)
{
	_reference = ref_msg.data;
	need_to_refresh = true;
}
