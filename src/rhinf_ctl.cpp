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
		/*rh::Matrix error = rh::sub_matrix(state,reference);
		double *e = rh::Matrix::matrix_to_array(error);
                float *e_abs = new float[error.getRows()*error.getCols()];
                for(int j=0; j<(error.getRows()*error.getCols()); j++)
                {
                        e_abs[j] = -1.1*abs((float)e[j]);
                }
                float *e_exp = new float[error.getRows()*error.getCols()];
                for(int j=0; j<(error.getRows()*error.getCols()); j++)
                {
                        e_exp[j] = exp(e_abs[j]) - 1;
		}
		double myUN = (double)(-3.6386 * snrm2(error.getRows()*error.getCols(),e_exp,1));
		double *_an = rh::Matrix::matrix_to_array(an);
		double *_xant = rh::Matrix::matrix_to_array(xant);
		int an_xant_r;
                int an_xant_c;*/
	}
}

bool rh::rhinf_ctl::load_param(std::vector<std::vector<std_msgs::Float64>> params, double _umax)
{
	std::vector<std_msgs::Float64> AN,BN,FN,F,KDIS;
	
	AN = params[0];
	BN = params[1];
	FN = params[2];
	F = params[3];
	KDIS = params[4];

	umax = _umax;
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
