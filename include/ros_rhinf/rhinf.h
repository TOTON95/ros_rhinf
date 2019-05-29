#ifndef RHINF_H
#define RHINF_H

#include <ros/ros.h>
#include <ros_rhinf/rhinf_ctl.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
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

			double _state;
			double _control_effort = 0;
			double _setpoint = 0;
			bool need_to_refresh = false;

			ros::Time prev_t;
			ros::Duration dt;

			ros::Publisher _control_effort_pub;
			std::string s_ctl, s_state, s_ref;

			std_msgs::Float64 ctl_msg, state_msg;	
	};	
}
#endif
