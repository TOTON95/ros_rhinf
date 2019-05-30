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
	n_priv.param<double>("/downsampling",_downsampling, 17);
	n_priv.param<double>("/sample_time", _sample_time,0.01);
	n_priv.param<double>("/saturation", _saturation, 1);
	n_priv.param<std::string>("/topic_controller", s_ctl, "control_effort");
	n_priv.param<std::string>("/topic_state", s_state, "state");
	n_priv.param<std::string>("/topic_ref", s_ref, "reference");

	if(!n_priv.getParam("/AN", an_param) || !n_priv.getParam("/AN_dim", d_an_param))
	{
		ROS_ERROR_STREAM("Failed to import AN parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("/BN", bn_param) || !n_priv.getParam("/BN_dim", d_bn_param))
	{
		ROS_ERROR_STREAM("Failed to import BN parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("/FN", fn_param) || !n_priv.getParam("/FN_dim", d_fn_param))
	{
		ROS_ERROR_STREAM("Failed to import FN parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("/F", f_param) || !n_priv.getParam("/F_dim", d_f_param))
	{
		ROS_ERROR_STREAM("Failed to import F parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("/KDIS", kdis_param) || !n_priv.getParam("/KDIS_dim", d_kdis_param))
	{
		ROS_ERROR_STREAM("Failed to import KDIS parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	//Setting sampling time
	sampling_t = ros::Duration(_sample_time);

	//Print Parameters
	print_param();


	//Setup controller
	setParams();

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
	while( ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64MultiArray>(s_ref, ros::Duration(10.)))
		ROS_WARN_STREAM("Waiting for first reference message");
	while( ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64MultiArray>(s_state, ros::Duration(10.)))
		ROS_WARN_STREAM("Waiting for first state message");	

	//TEST
	ros::Time s0_t = ros::Time::now();
	//END_TEST

	//Infinite cycle until
	while(ros::ok())
	{
		calc();
		ros::Time s1_t = ros::Time::now();
		ros::Duration d= s1_t-s0_t;
		//ROS_WARN("Duration: %lf",d.toSec());
		s0_t = s1_t;
		ros::spinOnce();
		//ROS_INFO("PASS\n");
		//ros::Duration(0.01).sleep();
	}
};

void rhinfObject::calc()
{
	//if(need_to_refresh)
	//{
		ros::Time init_t = ros::Time::now();
		//std::cout<<"init_t: "<<init_t.toSec()<<std::endl;
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

		
		//std::cout<<"prev_t: "<<prev_t.toSec()-init_t.toSec()<<std::endl;

		//Creating state and reference structure
		Matrix m_state(_state.size(),1,vector2array(_state));
		Matrix m_ref(_reference.size(),1,vector2array(_state));

		//Updating controller
		double output = ctl.update(m_state, m_ref, _it);

		exec_t = ros::Time::now();

		//std::cout<<"Output: "<<output<<std::endl;

		//Send output signal
		ctl_msg.data = output;
		_control_effort_pub.publish(ctl_msg);
			
		//std::cout<<"exec_t: "<<exec_t.toSec()-init_t.toSec()<<std::endl;
		sleep_t = sampling_t - (exec_t - init_t);
		//std::cout<<"sleep_t: "<<sleep_t.toSec()<<std::endl;
		//std::cout<<"_it: "<<_it<<std::endl; 
		_it++;
		
		//TEST
		if(sleep_t.toSec() >= 0)
		{
			sleep_t.sleep();
		}
		else
		{
			ROS_ERROR("Too LONG");
			ros::shutdown();
		}
		//END_TEST

	//}
	//need_to_refresh = false;
}

void rhinfObject::setParams()
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
	
	//TEST
	/*std::cout<<"Size params: "<<params[0].size()<<std::endl;
	for(int i=0;i<params.size();i++)
	{
		for(int j=0;j<params[i].size();j++)
		{
			std::cout<<params[i][j].data<<std::endl;
		}
	}*/
	//END_TEST

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

void rhinfObject::insert_data(std::vector<std_msgs::Float64> &ros_vec, std::vector<double> const& vec)
{
	for(int i = 0; i<vec.size();i++)
	{
		std_msgs::Float64 f;
		f.data = vec[i];
		ros_vec.push_back(f);
		//std::cout<<"Inserted data: "<<f.data<<std::endl;
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
