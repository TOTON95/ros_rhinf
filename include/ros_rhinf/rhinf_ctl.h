#ifndef _RHINF_CTL_
#define _RHINF_CTL_

#include <iostream>
#include <cmath>
#include <string>
#include <ros_rhinf/matrix.h>
#include <std_msgs/Float64.h>
#include <vector>

extern "C"
{
	float snrm2_(const int *N, const float *a, const int *inca);
	void dgemm_(const char *tA, const char *tB, const int *M, const int *N, const int *K, const double *alpha, const double *A, const int *lda, const double *B, const int *ldb, const double *beta, const double *C, const int *ldc);
}

namespace rh
{
	class rhinf_ctl
	{
		public:
			rhinf_ctl();
			~rhinf_ctl();
			double update(rh::Matrix &state, rh::Matrix &reference, int t);
			bool load_param(std::vector<std::vector<std_msgs::Float64>> params, double umax);
		private:
			rh::Matrix an, bn, fn, f, kdis;
			rh::Matrix xant;
			double uant = 0;
			double umax = 1;

			float snrm2(int N, float *a, int inca) {return snrm2_(&N,a,&inca);};
			double* dgemm(int _numRowA , int _numColA , double* _A , int _numRowB , int _numColB , double* _B , int& _numRowC , int& _numColC);
			double usat(double umax, double u);
	};
}

#endif
