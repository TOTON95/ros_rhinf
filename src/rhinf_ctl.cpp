#include <ros_rhinf/rhinf_ctl.h>


rh::rhinf_ctl::rhinf_ctl()
{
}

rh::rhinf_ctl::~rhinf_ctl()
{
}

std::vector<double>& rh::rhinf_ctl::getDis()
{
	return dis_out;
}
std::vector<double>& rh::rhinf_ctl::getDisf()
{
	return disf_out;
}
std::vector<double>& rh::rhinf_ctl::getDisfant()
{
	return disfant_out;
}
std::vector<double>& rh::rhinf_ctl::getError()
{
	return error_out;
}
std::vector<double>& rh::rhinf_ctl::getErrorAbs()
{
	return error_abs_out;
}
std::vector<double>& rh::rhinf_ctl::getErrorExp()
{
	return error_exp_out;
}
double rh::rhinf_ctl::getMyUN()
{
	return myUN_out;
}
std::vector<double>& rh::rhinf_ctl::getANXANT()
{
	return an_xant_out;
}
std::vector<double>& rh::rhinf_ctl::getBNUANT()
{
	return bn_uant_out;
}
std::vector<double>& rh::rhinf_ctl::getXANXANT()
{
	return x_an_xant_out;
}
std::vector<double>& rh::rhinf_ctl::getKDISDIS()
{
	return kdis_dis_out;
}
std::vector<double>& rh::rhinf_ctl::getMyUNFN()
{
	return myUN_fn_out;
}
std::vector<double>& rh::rhinf_ctl::getFMyUNFN()
{
	return f_myUN_fn_out;
}
std::vector<double>& rh::rhinf_ctl::getMFMFE()
{
	return m_fmf_e_out;
}

double rh::rhinf_ctl::update(Eigen::MatrixXd &state, Eigen::MatrixXd &reference, int t)
{
	if (t == 0)
        {
                xant = Eigen::MatrixXd(2,1);
		xant << 0,0;
                uant = 0;
		uant2 = 0;
		disfant = Eigen::MatrixXd(2,1);
		disfant << 0,0;
        }
        if (t%downsampling==0)
	{

	//	std::cout<<"State: \n"<<state<<std::endl;
	//	std::cout<<"Ref: \n"<<reference<<std::endl;
	//	std::c	std::cout<<"Xant: \n"<<xant<<std::endl;

		error = state - reference;
		std::vector<double> temp_error(error.data(),error.data()+error.rows()*error.cols());
		error_out = temp_error;
		/* std::cout<<"Err: \n"<<error<<std::endl; */

		error_abs = error.cwiseAbs();
		error_abs = -1.1 * error_abs;
		std::vector<double> temp_error_abs(error_abs.data(),error_abs.data()+error_abs.rows()*error_abs.cols());
		error_abs_out = temp_error_abs;
		/* std::cout<<"Err_abs: \n"<<error.cwiseAbs()<<std::endl; */

		error_exp.resize(error.rows(),error.cols());
		error_exp = error_abs.array().exp().matrix();
		i_d.resize(error.rows(),error.cols());
		i_d.setOnes();
		error_exp = error_exp - i_d;
		std::vector<double> temp_error_exp(error_exp.data(),error_exp.data()+error_exp.rows()*error_exp.cols());
		error_exp_out = temp_error_exp;
		/* std::cout<<std::endl<<error_exp<<std::endl; */
		/* std::cout<<"Err_exp: \n"<<error_exp<<std::endl; */

		double myUN = (double)(-3.6386 * error_exp.squaredNorm());
		myUN_out = myUN;
	//	std::cout<<"MYUN: "<<myUN<<std::endl;

		//An@xant
		an_xant = an*xant;
		std::vector<double> temp_an_xant(an_xant.data(),an_xant.data()+an_xant.rows()*an_xant.cols());
		an_xant_out = temp_an_xant;
		/* std::cout<<"An@xant: \n"<<an_xant<<std::endl; */

		//Bn*uant
		bn_uant = bn*uant2;
		std::vector<double> temp_bn_uant(bn_uant.data(),bn_uant.data()+bn_uant.rows()*bn_uant.cols());
		bn_uant_out = temp_bn_uant;
		/* std::cout<<"bn_uant: \n"<<bn_uant<<std::endl; */

		//x-an_xant
		x_an_xant = state-an_xant;
		std::vector<double> temp_x_an_xant(x_an_xant.data(),x_an_xant.data()+x_an_xant.rows()*x_an_xant.cols());
		x_an_xant_out = temp_x_an_xant;
		/* std::cout<<"x_an_xant: \n"<<x_an_xant<<std::endl; */

		//dls
		dis = x_an_xant-bn_uant;
		disf = (1-b)*disfant+b*dis;
		std::vector<double> temp_dis(dis.data(),dis.data()+dis.rows()*dis.cols());
		dis_out = temp_dis;
		std::vector<double> temp_disf(disf.data(),disf.data()+disf.rows()*disf.cols());
		disf_out = temp_disf;
		/* std::cout<<"dis: \n"<<disf<<std::endl; */

		//Saving last disturbance
		disfant = disf;
		std::vector<double> temp_disfant(disfant.data(),disfant.data()+disfant.rows()*disfant.cols());
		disfant_out = temp_disfant;

		//Kdls@dls
		kdis_dis = kdis*disf;
		std::vector<double> temp_kdis_dis(kdis_dis.data(),kdis_dis.data()+kdis_dis.rows()*kdis_dis.cols());
		kdis_dis_out = temp_kdis_dis;
		/* std::cout<<"kdis_dis: \n"<<kdis_dis<<std::endl; */

		////myUN*Fn
		myUN_fn = myUN*fn;
		std::vector<double> temp_myUN_fn(myUN_fn.data(),myUN_fn.data()+myUN_fn.rows()*myUN_fn.cols());
		myUN_fn_out = temp_myUN_fn;
		/* std::cout<<"myUN_fn: \n"<<myUN_fn<<std::endl; */

		////F+myUN_fn
		f_myUN_fn = f+myUN_fn;
		std::vector<double> temp_f_myUN_fn(f_myUN_fn.data(),f_myUN_fn.data()+f_myUN_fn.rows()*f_myUN_fn.cols());
		f_myUN_fn_out = temp_f_myUN_fn;
		/* std::cout<<"f_myUN_fn: \n"<<f_myUN_fn<<std::endl; */

		//f_myUN_fn @ e
		m_fmf_e = f_myUN_fn*error;
		std::vector<double> temp_m_fmf_e(m_fmf_e.data(),m_fmf_e.data()+m_fmf_e.rows()*m_fmf_e.cols());
		m_fmf_e_out = temp_m_fmf_e;
		/* std::cout<<"m_fmf_e: \n"<<m_fmf_e<<std::endl; */

		//m_fmf_e + kdls@dls
		u = m_fmf_e + kdis_dis;
		//Eigen::MatrixXd u = m_fmf_e;
	//	std::cout<<"u: \n"<<u<<std::endl;


		//Save u
		double u_out = usat(umax,u(0,0));
	//	std::cout<<"U_sat: "<<uant<<std::endl;

		//Save uant2
		uant2 = uant;

		//Save last values
		xant = state;
		uant = u_out;

	//	std::cout<<"Uant: "<<uant<<std::endl;
	//	std::cout<<"Uant2: "<<uant2<<std::endl;

                return u_out;
	}
	else
	{
		xant = state;
		return uant;
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

/*double* rh::rhinf_ctl::dgemm(int _numRowA , int _numColA , double* _A , int _numRowB , int _numColB , double* _B , int& _numRowC , int& _numColC)
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
	//int lda = ( _numRowA >= _numColA ) ? _numRowA : _numColA;
	//int ldb = ( _numRowB >= _numColB ) ? _numRowB : _numColB;
	//int ldc = ( _numRowC >= _numColC ) ? _numRowC : _numColC;
	int lda = _numRowA;
	int ldb = _numRowB;
	int ldc = _numRowC;
	dgemm_(&trans, &trans, &_numRowA , &_numColB , &_numColA , &alpha , _A , &lda , _B , &ldb , &beta , C , &ldc );

	return C;

}*/

double rh::rhinf_ctl::usat(double umax, double u)
{
	double n_u = u;
	if(n_u > umax)
	{
		n_u = umax;
	}
	if(n_u < -umax)
	{
		n_u = -umax;
	}
	return n_u;
}
