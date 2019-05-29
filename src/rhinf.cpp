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
	n_priv.param<double>("sample_time", _sample_time,0.01);
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

	//Setting sampling time
	sampling_t = ros::Duration(_sample_time);

	//Print Parameters
	print_param();


	//Start controller
	rhinf_ctl ctl;
	setParams(ctl);

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
		sleep_t.sleep();
	}
};

void rhinfObject::calc()
{
	if(need_to_refresh)
	{
		ros::Time init_t = prev_t;
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
			ROS_INFO("prev_t is 0, continue...");
			prev_t = ros::Time::now();
			return;
		}

		Matrix m_state(1,state.size(),vector2array(state));
		Matrix m_ref(1,reference.size(),vector2array(state));



		exec_t = ros::Time::now();
		sleep_t = sampling_t - (exec_t - init_t);
		_it++;
	}
	need_to_refresh = false;
}

void rhinfObject::setParams(rh::rhinf_ctl &ctl)
{
	//Coefficients
	std::vector<std::vector<std_msgs::Float64>> params;
	std::vector<std_msgs::Float64> ros_an;
	std::vector<std_msgs::Float64> ros_bn;
	std::vector<std_msgs::Float64> ros_fn;
	std::vector<std_msgs::Float64> ros_f;
	std::vector<std_msgs::Float64> ros_kdis;

	insert_data(ros_an, an_param);
	insert_data(ros_bn, bn_param);
	insert_data(ros_fn, fn_param);
	insert_data(ros_f, f_param);
	insert_data(ros_kdis, kdis_param);

	params.push_back(ros_an); 
	params.push_back(ros_bn);
	params.push_back(ros_fn);
	params.push_back(ros_f);
	params.push_back(ros_kdis);

	//Dimensions
	std::vector<std::vector<int>> dims;

	dims.push_back(d_an_param);
	dims.push_back(d_bn_param);
	dims.push_back(d_fn_param);
	dims.push_back(d_f_param);
	dims.push_back(d_kdis_param);

	//Controller
	ctl.load_param(params,dims,_saturation,_downsampling);

	std::cout<<std::endl<<"PARAMETERS LOADED"<<std::endl;
}

void rhinfObject::insert_data(std::vector<std_msgs::Float64> ros_vec, std::vector<double> const& vec)
{
	for(int i = 0; i<vec.size();i++)
	{
		std_msgs::Float64 f;
		f.data = vec[i];
		ros_vec.push_back(f);
	}

}

void rhinfObject::print_param()
{
	std::cout<<std::endl<<"RHINF PARAMETERS:"<<std::endl;
	std::cout<<"AN: "<<std::endl;
	explore_vector(an_param);
	std::cout<<"Dimensions: "<<std::endl;
	explore_vector(d_an_param);
	std::cout<<"BN: "<<std::endl;
	explore_vector(bn_param);
	std::cout<<"Dimensions: "<<std::endl;
	explore_vector(d_bn_param);
	std::cout<<"FN: "<<std::endl;
	explore_vector(fn_param);
	std::cout<<"Dimensions: "<<std::endl;
	explore_vector(d_fn_param);
	std::cout<<"F: "<<std::endl;
	explore_vector(f_param);
	std::cout<<"Dimensions: "<<std::endl;
	explore_vector(d_f_param);
	std::cout<<"KDIS: "<<std::endl;
	explore_vector(kdis_param);
	std::cout<<"Dimensions: "<<std::endl;
	explore_vector(d_kdis_param);
	std::cout<<"Sampling time: "<<_sample_time<<std::endl;
	std::cout<<"Downsampling: "<<_downsampling<<std::endl;
	std::cout<<"Saturation: "<<_saturation<<std::endl;
}

void rhinfObject::stateCallback(const std_msgs::Float64MultiArray& state_msg)
{
	_state = state_msg.data;
	need_to_refresh = true;
}

void rhinfObject::refCallback(const std_msgs::Float64MultiArray& ref_msg)
{
	_reference = ref_msg.data;
	need_to_refresh = true;
}
