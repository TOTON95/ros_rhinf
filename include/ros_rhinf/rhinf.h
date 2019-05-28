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
	};	
}
#endif
