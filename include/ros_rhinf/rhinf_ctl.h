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

			std::vector<double>& getDis();
			std::vector<double>& getDisf();
			std::vector<double>& getDisfant();
			std::vector<double>& getError();
			std::vector<double>& getErrorAbs();
			std::vector<double>& getErrorExp();
			double getMyUN();
			std::vector<double>& getANXANT();
			std::vector<double>& getBNUANT();
			std::vector<double>& getXANXANT();
			std::vector<double>& getKDISDIS();
			std::vector<double>& getMyUNFN();
			std::vector<double>& getFMyUNFN();
			std::vector<double>& getMFMFE();

            //Enable disturbance calculation
            void enable_dis(bool b);


		private:
			Eigen::MatrixXd an, bn, fn, f, kdis;
			Eigen::MatrixXd xant;
			double uant = 0;
			double uant2 = 0;
			double umax = 1;
			Eigen::MatrixXd disfant;
			int downsampling = 1;

			//Velocity estimation
			double win_width = 50.00;
			//double a = 0.50;
			double a = 1.00;
			double b = 0.05;

            //dis
            bool calculate_disturbance = false;

			//Calculations matrices
			Eigen::MatrixXd error;
			Eigen::MatrixXd error_abs;
			Eigen::MatrixXd error_exp;
			Eigen::MatrixXd i_d;
			Eigen::MatrixXd an_xant;
			Eigen::MatrixXd bn_uant;
			Eigen::MatrixXd x_an_xant;
			Eigen::MatrixXd dis;
			Eigen::MatrixXd disf;
			Eigen::MatrixXd kdis_dis;
			Eigen::MatrixXd myUN_fn;
			Eigen::MatrixXd f_myUN_fn;
			Eigen::MatrixXd m_fmf_e;
			Eigen::MatrixXd u;

			//Expose data
			std::vector<double> dis_out;
			std::vector<double> disf_out;
			std::vector<double> disfant_out;
			std::vector<double> error_out;
			std::vector<double> error_abs_out;
			std::vector<double> error_exp_out;
			double myUN_out;
			std::vector<double> an_xant_out;
			std::vector<double> bn_uant_out;
			std::vector<double> x_an_xant_out;
			std::vector<double> kdis_dis_out;
			std::vector<double> myUN_fn_out;
			std::vector<double> f_myUN_fn_out;
			std::vector<double> m_fmf_e_out;

			float snrm2(int N, float *a, int inca) {return snrm2_(&N,a,&inca);};
			double* dgemm(int _numRowA , int _numColA , double* _A , int _numRowB , int _numColB , double* _B , int& _numRowC , int& _numColC);
			double usat(double umax, double u);
			double* extract_data(std::vector<std_msgs::Float64> d); 
			int* extract_dims(std::vector<std::vector<int>> dims);
			void set_data(double* data, Eigen::MatrixXd &mat);
	};
}
#endif
