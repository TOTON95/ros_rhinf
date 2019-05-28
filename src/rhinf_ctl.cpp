#include <ros_rhinf/rhinf_ctl.h>


rh::rhinf_ctl::rhinf_ctl()
{
}

rh::rhinf_ctl::~rhinf_ctl()
{
}

double rh::rhinf_ctl::update(rh::Matrix &state, rh::Matrix &reference, int t)
{
	if (t == 0)
        {
                double *_data = new double[2];
                _data[0] = 0; _data[1] = 0;
                xant = rh::Matrix(2,1,_data);
                uant = 0;
        }
        if (t%17==0)
	{
		rh::Matrix error = rh::Matrix::sub_matrix(state, reference);
	}
}

bool rh::rhinf_ctl::load_param(std::vector<std_msgs::Float64> params)
{
}

double* rh::rhinf_ctl::dgemm(int _numRowA , int _numColA , double* _A , int _numRowB , int _numColB , double* _B , int& _numRowC , int& _numColC)
{
	if( _numColA != _numRowB )
        {
                std::cout << "number of columns of Matrix A is NOT equal to num of rows of Matrix B" << std::endl;
                return NULL;
        }

        _numRowC = _numRowA;
        _numColC = _numColB;

        char trans = 'N';
        double alpha = 1.0 , beta = 0.0;
        double *C = new double [ _numRowC * _numColC ];
        int lda = _numRowA;
        int ldb = _numRowB;
        int ldc = _numRowC;
        dgemm_(&trans, &trans, &_numRowA , &_numColB , &_numColA , &alpha , _A , &lda , _B , &ldb , &beta , C , &ldc );

        return C;
}

double rh::rhinf_ctl::usat(double umax, double u)
{
	double n_u = u;
        if(n_u > umax)
                n_u = umax;
        if(n_u < -umax)
                n_u = -umax;
        return n_u;

}
