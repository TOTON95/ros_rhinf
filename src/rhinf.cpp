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

	if(!n_priv.getParam("FN", fn_param) || !n_priv.getParam("FN_dim", d_fn_param))
	{
		ROS_ERROR_STREAM("Failed to import FN parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("F", f_param) || !n_priv.getParam("F_dim", d_f_param))
	{
		ROS_ERROR_STREAM("Failed to import F parameters. Terminating.");
		ros::shutdown();
		exit(EXIT_FAILURE);
	}

	if(!n_priv.getParam("KDIS", kdis_param) || !n_priv.getParam("KDIS_dim", d_kdis_param))
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

	//Debug
//	_dis_pub = node.advertise<std_msgs::Float64MultiArray>("d_dis",1);
//	_disf_pub = node.advertise<std_msgs::Float64MultiArray>("d_disf",1);
//	_disfant_pub = node.advertise<std_msgs::Float64MultiArray>("d_disfant",1);
//	_error_pub = node.advertise<std_msgs::Float64MultiArray>("d_error",1);
//	_error_abs_pub = node.advertise<std_msgs::Float64MultiArray>("d_error_abs",1);
//	_error_exp_pub = node.advertise<std_msgs::Float64MultiArray>("d_error_exp",1);
//	_myUN_pub = node.advertise<std_msgs::Float64>("d_myUN",1);
//	_an_xant_pub = node.advertise<std_msgs::Float64MultiArray>("d_an_xant",1);
//	_bn_uant_pub = node.advertise<std_msgs::Float64MultiArray>("d_bn_uant",1);
//	_x_an_xant_pub = node.advertise<std_msgs::Float64MultiArray>("d_x_an_xant",1);
//	_kdis_dis_pub = node.advertise<std_msgs::Float64MultiArray>("d_kdis_dis",1);
//	_myUN_fn_pub = node.advertise<std_msgs::Float64MultiArray>("d_myUN_fn",1);
//	_f_myUN_fn_pub = node.advertise<std_msgs::Float64MultiArray>("d_f_myUN_fn",1);
//	_m_fmf_e_pub = node.advertise<std_msgs::Float64MultiArray>("d_m_fmf_e",1);

	_dis_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_dis",1);
	_disf_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_disf",1);
	_disfant_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_disfant",1);
	_error_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_error",1);
	_error_abs_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_error_abs",1);
	_error_exp_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_error_exp",1);
	_myUN_pub = n_priv.advertise<std_msgs::Float64>("d_myUN",1);
	_an_xant_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_an_xant",1);
	_bn_uant_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_bn_uant",1);
	_x_an_xant_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_x_an_xant",1);
	_kdis_dis_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_kdis_dis",1);
	_myUN_fn_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_myUN_fn",1);
	_f_myUN_fn_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_f_myUN_fn",1);
	_m_fmf_e_pub = n_priv.advertise<std_msgs::Float64MultiArray>("d_m_fmf_e",1);

	ros::Subscriber state_sub = node.subscribe(s_state, 1, &rhinfObject::stateCallback, this);
	ros::Subscriber ref_sub = node.subscribe(s_ref, 1, &rhinfObject::refCallback, this);
	ros::Subscriber dis_sub = node.subscribe("dis_enable", 1, &rhinfObject::disCallback, this);

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
		Eigen::MatrixXd m_state(2,2), m_ref(2,2);

		//std::cout<<"state_size:"<<_state.size()<<std::endl;
		//std::cout<<"m_state:"<<m_state.size()<<std::endl;

		//Resize them
		m_state.resize(_state.size(), 1);
		m_ref.resize(_reference.size(),1);

		//Retrieve data
		double* _state_data = vector2array(_state);
		double* _reference_data = vector2array(_reference); 

		//Fill them
		for(int i=0;i<_state.size();i++)
		{
			m_state(i,0) = _state_data[i];
			m_ref(i,0) = _reference_data[i];
		}

		//Updating controller
		double output = ctl.update(m_state, m_ref, _it);

        //Enable disturbance estimation?
        ctl.enable_dis(dis_enabled);

		//DEBUG
		std::vector<double> vec;
//		dis_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
//		dis_out.layout.dim[0].size = vec.size();
//		dis_out.layout.dim[0].stride = 1;

        if(dis_enabled)
        {
            //DIS
            vec = ctl.getDis();
            dis_out.data.clear();
            dis_out.data.insert(dis_out.data.end(), vec.begin(), vec.end());

            //DISF
            vec = ctl.getDisf();
            disf_out.data.clear();
            disf_out.data.insert(disf_out.data.end(), vec.begin(), vec.end());

            //DISFANT
            vec = ctl.getDisfant();
            disfant_out.data.clear();
            disfant_out.data.insert(disfant_out.data.end(), vec.begin(), vec.end());

        }
		//ERROR
		vec = ctl.getError();
		error_out.data.clear();
		error_out.data.insert(error_out.data.end(), vec.begin(), vec.end());
		
		//ERROR ABS
		vec = ctl.getErrorAbs();
		error_abs_out.data.clear();
		error_abs_out.data.insert(error_abs_out.data.end(), vec.begin(), vec.end());
		
		//ERROR EXP
		vec = ctl.getErrorExp();
		error_exp_out.data.clear();
		error_exp_out.data.insert(error_exp_out.data.end(), vec.begin(), vec.end());
		
		//MYUN
		myUN_out.data = ctl.getMyUN();
		
		//ANXANT
		vec = ctl.getANXANT();
		an_xant_out.data.clear();
		an_xant_out.data.insert(an_xant_out.data.end(), vec.begin(), vec.end());
		
		//BNUANT
		vec = ctl.getBNUANT();
		bn_uant_out.data.clear();
		bn_uant_out.data.insert(bn_uant_out.data.end(), vec.begin(), vec.end());

        //XANXANT
        vec = ctl.getXANXANT();
        x_an_xant_out.data.clear();
        x_an_xant_out.data.insert(x_an_xant_out.data.end(), vec.begin(), vec.end());


        if(dis_enabled)
        {

            //KDISDIS
            vec = ctl.getKDISDIS();
            kdis_dis_out.data.clear();
            kdis_dis_out.data.insert(kdis_dis_out.data.end(), vec.begin(), vec.end());

        }

		//MyUNFN
		vec = ctl.getMyUNFN();
		myUN_fn_out.data.clear();
		myUN_fn_out.data.insert(myUN_fn_out.data.end(), vec.begin(), vec.end());
		
		//FMyUNFN
		vec = ctl.getFMyUNFN();
		f_myUN_fn_out.data.clear();
		f_myUN_fn_out.data.insert(f_myUN_fn_out.data.end(), vec.begin(), vec.end());
		
		//MFMFE
		vec = ctl.getMFMFE();
		m_fmf_e_out.data.clear();
		m_fmf_e_out.data.insert(m_fmf_e_out.data.end(), vec.begin(), vec.end());
		

		exec_t = ros::Time::now();

	//	std::cout<<"Output: "<<output<<std::endl;

		//Send output signal
		ctl_msg.data = output;
		_control_effort_pub.publish(ctl_msg);


		//DEBUG
        if(dis_enabled)
        {
            _dis_pub.publish(dis_out);
            _disf_pub.publish(disf_out);
            _disfant_pub.publish(disfant_out);
            _kdis_dis_pub.publish(kdis_dis_out);
        }
        
        _error_pub.publish(error_out);
		_error_abs_pub.publish(error_abs_out);
		_error_exp_pub.publish(error_exp_out);
		_myUN_pub.publish(myUN_out);
		_an_xant_pub.publish(an_xant_out);
		_bn_uant_pub.publish(bn_uant_out);
		_x_an_xant_pub.publish(x_an_xant_out);
		_myUN_fn_pub.publish(myUN_fn_out);
		_f_myUN_fn_pub.publish(f_myUN_fn_out);
		_m_fmf_e_pub.publish(m_fmf_e_out);
			
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
void rhinfObject::disCallback(const std_msgs::Bool& dis_msg)
{
	dis_enabled = dis_msg.data;
}
