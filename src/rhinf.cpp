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
	
	//Get parameters
	n_priv.param<double>("downsampling",_downsampling, 17);
	n_priv.param<double>("sample_time", _sample_time,0.001);
	n_priv.param<double>("saturation", _saturation, 1);
	n_priv.param<std::string>("topic_controller", s_ctl, "control_effort");
	n_priv.param<std::string>("topic_state", s_state, "state");
	n_priv.param<std::string>("topic_ref", s_ref, "reference");

	if(!n_priv.getParam("AN", an_param) || !n_priv.getParam("AN_dim", d_an_param))
	{
		ROS_ERROR_STREAM("Failed to import AN parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("BN", bn_param) || !n_priv.getParam("BN_dim", d_bn_param))
	{
		ROS_ERROR_STREAM("Failed to import BN parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("FN", bn_param) || !n_priv.getParam("FN_dim", d_fn_param))
	{
		ROS_ERROR_STREAM("Failed to import FN parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("F", bn_param) || !n_priv.getParam("F_dim", d_f_param))
	{
		ROS_ERROR_STREAM("Failed to import F parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("KDIS", bn_param) || !n_priv.getParam("KDIS_dim", d_kdis_param))
	{
		ROS_ERROR_STREAM("Failed to import KDIS parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

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
		//dt calculation
		if(!prev_t.isZero())
		{
			dt = ros::Time::now() - prev_t;
			prev_t = ros::Time::now();

			if(0 == dt.toSec())
			{
				ROS_ERROR("dt is 0, therefore, time hasn't changed at the time: %f", ros::Time::now().toSec());
				return;
			}
		}
		else
		{
			ROS_INFO("prev_t is 0, nothing to change");
			prev_t = ros::Time::now();
			return;
		}
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
