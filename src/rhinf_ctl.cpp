#include <ros_rhinf/rhinf_ctl.h>


rh::rhinf_ctl::rhinf_ctl()
{
	
}

rh::rhinf_ctl::~rhinf_ctl()
{
}

double rh::rhinf_ctl::update(Eigen::MatrixXd &state, Eigen::MatrixXd &reference, int t)
{
	if (t == 0)
        {
                xant = Eigen::MatrixXd(2,1);
		xant << 0,0;
                uant = 0;
        }
        if (t%downsampling==0)
	{
	//	std::cout<<"State: \n"<<state<<std::endl;
		std::cout<<"Ref: \n"<<reference<<std::endl;
	//	std::cout<<"Xant: \n"<<xant<<std::endl;

		Eigen::MatrixXd error = state - reference;
		std::cout<<"Err: \n"<<error<<std::endl;

		Eigen::MatrixXd error_abs = error.cwiseAbs();
	//	std::cout<<"Err_abs: \n"<<error.cwiseAbs()<<std::endl;

		Eigen::MatrixXd error_exp;
		error_exp.resize(error.rows(),error.cols());
		error_exp = error_abs.array().exp().matrix();
	//	std::cout<<"Err_exp: \n"<<error_exp<<std::endl;

		double myUN = (double)(-3.6386 * error_exp.squaredNorm());
	//	std::cout<<"MYUN: "<<myUN<<std::endl;

		//An@xant
		Eigen::MatrixXd an_xant =  an*xant;
	//	std::cout<<"An@xant: \n"<<an_xant<<std::endl;

		//Bn*uant
		Eigen::MatrixXd bn_uant = bn*usat(umax,uant);
	//	std::cout<<"bn_uant: \n"<<bn_uant<<std::endl;

		//x-an_xant
		Eigen::MatrixXd x_an_xant = state-an_xant;
	//	std::cout<<"x_an_xant: \n"<<x_an_xant<<std::endl;

		//dls
		Eigen::MatrixXd dis = x_an_xant-bn_uant;
	//	std::cout<<"dis: \n"<<dis<<std::endl;

		//Kdls@dls
		Eigen::MatrixXd kdis_dis = kdis*dis;
	//	std::cout<<"kdis_dis: \n"<<kdis_dis<<std::endl;

		////myUN*Fn
		Eigen::MatrixXd myUN_fn = myUN*fn;
	//	std::cout<<"myUN_fn: \n"<<myUN_fn<<std::endl;

		////F+myUN_fn
		Eigen::MatrixXd f_myUN_fn = f;
	//	std::cout<<"f_myUN_fn: \n"<<f_myUN_fn<<std::endl;

		//f_myUN_fn @ e
		Eigen::MatrixXd m_fmf_e = f_myUN_fn*error;
	//	std::cout<<"m_fmf_e: \n"<<m_fmf_e<<std::endl;

		//m_fmf_e + kdls@dls
		Eigen::MatrixXd u = m_fmf_e + kdis_dis;
		//Eigen::MatrixXd u = m_fmf_e;
	//	std::cout<<"u: \n"<<u<<std::endl;

		uant = usat(umax,u(0,0));
	//	std::cout<<"U_sat: "<<uant<<std::endl;
		xant = state;
                return uant;
	}
	else
	{
		xant = state;
		return usat(umax,uant);
	}
}

bool rh::rhinf_ctl::load_param(std::vector<std::vector<std_msgs::Float64>> params,std::vector<std::vector<int>> dims, double _umax, int ds)
{
	std::vector<std_msgs::Float64> AN,BN,FN,F,KDIS;

	AN = params[0];
	BN = params[1];
	FN = params[2];
	F = params[3];
	KDIS = params[4];

	double *_AN,*_BN,*_FN,*_F,*_KDIS;
	
	_AN = rh::rhinf_ctl::extract_data(AN);
	_BN = rh::rhinf_ctl::extract_data(BN);
	_FN = rh::rhinf_ctl::extract_data(FN);
	_F = rh::rhinf_ctl::extract_data(F);
	_KDIS = rh::rhinf_ctl::extract_data(KDIS);

	int *dim = extract_dims(dims);
	
	an.resize(dim[0],dim[1]); 
	bn.resize(dim[2],dim[3]);
	fn.resize(dim[4],dim[5]);
	f.resize(dim[6],dim[7]);
	kdis.resize(dim[8],dim[9]);

	set_data(_AN, an);
	set_data(_BN, bn);
	set_data(_FN, fn);
	set_data(_F, f);
	set_data(_KDIS, kdis);

	//TEST
	std::cout<<"AN: \n"<<an<<std::endl;
	std::cout<<"BN: \n"<<bn<<std::endl;
	std::cout<<"FN: \n"<<fn<<std::endl;
	std::cout<<"F: \n"<<f<<std::endl;
	std::cout<<"KDIS: \n"<<kdis<<std::endl;
	//END_TEST
	
	umax = _umax;
	downsampling = ds;
}

double* rh::rhinf_ctl::extract_data(std::vector<std_msgs::Float64> d)
{
	double *data = new double[d.size()];
	for(int i=0;i<d.size();i++)
	{
		data[i] = d.at(i).data;
	}	
	return data;
}

void rh::rhinf_ctl::set_data(double* data, Eigen::MatrixXd &mat)
{
	int aux = 0;
	for(int i=0;i<mat.rows();i++)
		for(int j=0;j<mat.cols();j++)
		{
			mat(i,j) = data[aux];
		       	//std::cout<<"Inserted data II: "<<data[aux]<<std::endl;	
			aux++;
		}
}


int* rh::rhinf_ctl::extract_dims(std::vector<std::vector<int>> dims)
{
	int* dimensions = new int[dims.size()*2];
	int c = 0;
	for(int i=0; i<dims.size(); i++)
	{
		for(int j = 0; j<dims[i].size(); j++)
		{
			dimensions[c] = dims[i][j];
			c++;	
		}
	}
	return dimensions;
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
	/*int lda = ( _numRowA >= _numColA ) ? _numRowA : _numColA;
	int ldb = ( _numRowB >= _numColB ) ? _numRowB : _numColB;
	int ldc = ( _numRowC >= _numColC ) ? _numRowC : _numColC;*/
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
