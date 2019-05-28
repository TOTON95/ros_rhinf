#include <ros_rhinf/rhinf_ctl.h>


rh::rhinf_ctl::rhinf_ctl()
{
}

rh::rhinf_ctl::~rhinf_ctl()
{
}

bool rh::rhinf_ctl::update(double state, double reference, int t)
{
}

bool rh::rhinf_ctl::load_param(std::vector<std_msgs::Float64> params)
{
}

double* rh::rhinf_ctl::dgemm(int _numRowA , int _numColA , double* _A , int _numRowB , int _numColB , double* _B , int& _numRowC , int& _numColC)
{
}

double rh::rhinf_ctl::usat(double umax, double u)
{
}
