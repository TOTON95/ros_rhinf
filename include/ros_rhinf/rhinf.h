#ifndef RHINF_H
#define RHINF_H

#include <ros/ros.h>
#include <ros_rhinf/rhinf_ctl.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/time.h>
#include <stdio.h>
#include <string>


namespace rh
{
	class rhinfObject
	{
		public:
			rhinfObject();
		private:
			//Methods
			void calc();
			void setParams();
			void print_param();
			void insert_data(std::vector<std_msgs::Float64> &ros_vec, std::vector<double> const& vec);

			//Callbacks
			void stateCallback(const std_msgs::Float64MultiArray& state_msg);
			void refCallback(const std_msgs::Float64MultiArray& ref_msg);

			//Template
			template <typename T, typename A>
				void explore_vector(std::vector<T,A> const& vec)
				{
					for(int i=0; i<vec.size();i++)
					{
						std::cout<<vec[i]<<" ";
					}
					std::cout<<std::endl;
				}

			//Parameters
			std::vector<double> _state;
			double _control_effort = 0;
			std::vector<double> _reference;
			bool need_to_refresh = false;
			double _downsampling;
			double _sample_time;
			double _saturation;
			std::vector<double> an_param;
			std::vector<double> bn_param;
			std::vector<double> fn_param;
			std::vector<double> f_param;
			std::vector<double> kdis_param;
			std::vector<int> d_an_param;
			std::vector<int> d_bn_param;
			std::vector<int> d_fn_param;
			std::vector<int> d_f_param;
			std::vector<int> d_kdis_param;

			//Time var
			ros::Time prev_t;
			ros::Duration sampling_t;
			ros::Duration dt;
			ros::Time exec_t;
			ros::Duration sleep_t;
			unsigned long int _it = 0;

			//Controller
			rh::rhinf_ctl ctl;
			

			//ROS I/O
			ros::Publisher _control_effort_pub;
			std::string s_ctl, s_state, s_ref;

			std_msgs::Float64 ctl_msg;
			std_msgs::Float64MultiArray state, reference;
	};	
}
#endif
