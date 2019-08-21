#ifndef _RHINF_CTL_
#define _RHINF_CTL_

#include <iostream>
#include <cmath>
#include <string>
#include <ros_rhinf/matrix.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <bits/stdc++.h>

#include <complex>
#define EIGEN_USE_BLAS
#include <Eigen/Core>

namespace rh
{
	class rhinf_ctl
	{
		public:
			rhinf_ctl();
			~rhinf_ctl();
			double update(Eigen::MatrixXd &state, Eigen::MatrixXd &reference,int t);
			bool load_param(std::vector<std::vector<std_msgs::Float64>> params,std::vector<std::vector<int>> dims, double umax, int ds);

		private:
			Eigen::MatrixXd an, bn, fn, f, kdis;
			Eigen::MatrixXd xant;
			double uant = 0;
			double umax = 1;
			int downsampling = 1;

			//Velocity estimation
			double win_width = 50.00;
			double a = 0.50;

			float snrm2(int N, float *a, int inca) {return snrm2_(&N,a,&inca);};
			double* dgemm(int _numRowA , int _numColA , double* _A , int _numRowB , int _numColB , double* _B , int& _numRowC , int& _numColC);
			double usat(double umax, double u);
			double* extract_data(std::vector<std_msgs::Float64> d); 
			int* extract_dims(std::vector<std::vector<int>> dims);
			void set_data(double* data, Eigen::MatrixXd &mat);
	};
}
#endif
