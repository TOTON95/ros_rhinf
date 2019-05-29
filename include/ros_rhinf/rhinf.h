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
			void calc();
			void getParams();
			void print_param();

			void stateCallback(const std_msgs::Float64& state_msg);
			void refCallback(const std_msgs::Float64& ref_msg);

			double _state;
			double _control_effort = 0;
			double _reference = 0;
			bool need_to_refresh = false;
			double _downsampling;
			double _sample_time;
			double _saturation;

			ros::Time prev_t;
			ros::Duration dt;

			ros::Publisher _control_effort_pub;
			std::string s_ctl, s_state, s_ref;

			std_msgs::Float64 ctl_msg, state_msg;	
	};	
}
#endif
