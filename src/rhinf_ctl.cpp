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
        if (t%downsampling==0)
	{
		rh::print_matrix(state,"state");
		rh::print_matrix(reference,"ref");
		rh::print_matrix(xant,"xant");

		rh::Matrix error = rh::sub_matrix(state,reference);
		rh::print_matrix(error,"error");
		
		double aux = 0;

		double *e = rh::matrix_to_array(error);
                float *e_abs = new float[error.getRows()*error.getCols()];
		std::cout<<"ABS:";
                for(int j=0; j<(error.getRows()*error.getCols()); j++)
                {
                        e_abs[j] = -1.1*abs((float)e[j]);
		       	std::cout<<" "<<e_abs[j]<<" ";	
                }
		std::cout<<std::endl<<"EXP:";
                float *e_exp = new float[error.getRows()*error.getCols()];
                for(int j=0; j<(error.getRows()*error.getCols()); j++)
                {
                        e_exp[j] = exp(e_abs[j]) - 1;
		       	std::cout<<" "<<e_exp[j]<<" ";	
		}
		std::cout<<std::endl;
		double myUN = (double)(-3.6386 * snrm2(error.getRows()*error.getCols(),e_exp,1));
		std::cout<<"MYUN: "<<myUN;
		double *_an = rh::matrix_to_f_array(an);
		double *_xant = rh::matrix_to_array(xant);
		int an_xant_r;
                int an_xant_c;

		//An@xant
                double *_r = rh::rhinf_ctl::dgemm(an.getRows() , an.getCols() , _an , xant.getRows() , xant.getCols() , _xant , an_xant_r , an_xant_c);
		rh::Matrix an_xant = rh::Matrix(an_xant_r,an_xant_c,_r,true);
		
		//TEST
		rh::print_matrix(an_xant,"an_xant");
		std::cout<<"_r"<<std::endl;
		std::cout<<"ROWS xant: "<<an_xant_r<<" COLS xant: "<<an_xant_c<<std::endl;
		for(int i=0;i<(an_xant_r*an_xant_c);i++)
		{
			std::cout<<_r[i]<<" ";
		}
		std::cout<<std::endl;
		//END_TEST


		//Bn*uant
                double *bn_uant = new double[bn.getRows()*bn.getCols()];
                double *_bn = rh::matrix_to_array(bn);
                for(int o=0;o<(bn.getRows()*bn.getCols());o++)
                {
                        bn_uant[o] = _bn[o]*usat(umax,uant);
                }
		rh::Matrix m_bn_uant = rh::Matrix(bn.getRows(),bn.getCols(),bn_uant);
		rh::print_matrix(m_bn_uant,"m_bn_uant");

		
		//x-an_xant
		rh::Matrix x_an_xant = rh::sub_matrix(state,an_xant);
		rh::print_matrix(x_an_xant,"x_an_xant");


		//dls
		rh::Matrix dis = rh::sub_matrix(x_an_xant,m_bn_uant);
		rh::print_matrix(dis,"dls");

		print_matrix(kdis,"KDIS");

		//Kdls@dls
                double *_kdis = rh::matrix_to_f_array(kdis);
                double *_dis = rh::matrix_to_f_array(dis);
                int kdis_dis_r;
                int kdis_dis_c;

		double *_q = rh::rhinf_ctl::dgemm(kdis.getRows() , kdis.getCols() , _kdis , dis.getRows() , dis.getCols() , _dis , kdis_dis_r , kdis_dis_c);
		rh::Matrix m_kdis_dis = rh::Matrix(kdis_dis_r,kdis_dis_c,_q,true);
		rh::print_matrix(m_kdis_dis,"m_kdis_dis");

		//myUN*Fn
                double *_fn = rh::matrix_to_array(fn);
                double *myUN_fn = new double[fn.getRows()*fn.getCols()];
                for(int i=0;i<(fn.getRows()*fn.getCols());i++)
                        myUN_fn[i] = _fn[i]*myUN;
		rh::Matrix m_myUN_fn = rh::Matrix(fn.getRows(),fn.getCols(),myUN_fn);
		rh::print_matrix(m_myUN_fn,"m_myUN_fn");

		//F+myUN_fn
		rh::Matrix f_myUN_fn = rh::sum_matrix(f,m_myUN_fn);
		rh::print_matrix(f_myUN_fn,"f_myUN_fn");

		//f_myUN_fn @ e
                double *_f_myUN_fn = rh::matrix_to_f_array(f_myUN_fn);
                double *_error = rh::matrix_to_f_array(error);
                int fmf_r,fmf_c;
                double *_s = rh::rhinf_ctl::dgemm((double)f_myUN_fn.getRows() , (double)f_myUN_fn.getCols() , _f_myUN_fn , error.getRows() , error.getCols() , _error , fmf_r , fmf_c);
		rh::Matrix m_fmf_e = rh::Matrix(fmf_r,fmf_c,_s,true);
		rh::print_matrix(m_fmf_e,"m_fmf_e");

		//m_fmf_e + kdls@dls
		rh::Matrix u = rh::sum_matrix(m_fmf_e , m_kdis_dis);
		rh::print_matrix(u,"u");

		uant = usat(umax,u.getData()[0][0]);
		std::cout<<"U_sat: "<<uant<<std::endl;
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
	
	an = rh::Matrix(dim[0],dim[1],_AN);
	bn = rh::Matrix(dim[2],dim[3],_BN);
	fn = rh::Matrix(dim[4],dim[5],_FN);
	f = rh::Matrix(dim[6],dim[7],_F);
	kdis = rh::Matrix(dim[8],dim[9],_KDIS);

	//TEST
	/*rh::print_matrix(an,"AN");
	rh::print_matrix(bn,"BN");
	rh::print_matrix(fn,"FN");
	rh::print_matrix(f,"F");
	rh::print_matrix(kdis,"KDIS");*/
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
