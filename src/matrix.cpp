//Coded by Alexis Guijarro 2019
#include <ros_rhinf/matrix.h>

rh::Matrix::Matrix(){}

rh::Matrix::~Matrix(){}

rh::Matrix::Matrix(int rows, int cols, double *da, bool fortran)
{
	m = rows;	
	n = cols;
	double **d = new double*[rows];
        for (int i = 0;i<rows;i++)
        {
                d[i] = new double[cols];
        }
	int counter = 0;
	if(fortran)
	{
		for(int i = 0; i<cols; i++)
		{
			for(int j=0; j<rows; j++)
			{
				d[i][j] = da[counter];
				counter++;
			}
		}
	}
	else
	{
		for (int i = 0; i<rows;i++)
		{
			for(int j=0;j<cols;j++)
			{
				d[i][j] = da[counter];
				counter++;
			}
		}
	}
	data = d;
}

rh::Matrix rh::sum_matrix(rh::Matrix &A, rh::Matrix &B)
{
	if(A.getRows() == B.getRows() && A.getCols() == B.getCols())
	{
		double *result = new double[A.getRows() *A.getCols()];
		int c = 0;
		for(int i=0; i < A.getRows();i++)
		{
			for(int j=0;j < A.getCols();j++)
			{
				result[c] = A.getData()[i][j] + B.getData()[i][j];
				c++;
			}
		}
		rh::Matrix r = rh::Matrix(A.getRows(),A.getCols(),result);
		return r;
	}
	else
		throw std::runtime_error("Input shapes must match");

}

rh::Matrix rh::sub_matrix(rh::Matrix &A, rh::Matrix &B)
{
	if(A.getRows() == B.getRows() && A.getCols() == B.getCols())
	{
		double *result = new double[A.getRows()*A.getCols()];
		int c = 0;
		for(int i=0; i < A.getRows();i++)
		{
			for(int j=0;j < A.getCols();j++)
			{
				result[c] = A.getData()[i][j] - B.getData()[i][j];
				c++;
			}
		}
		rh::Matrix r = rh::Matrix(A.getRows(),A.getCols(),result);
		return r;
	}
	else
		throw std::runtime_error("Input shapes must match");
}

void rh::print_matrix(rh::Matrix &A, std::string title)
{
	std::cout<<std::endl;
	std::cout<<title<<std::endl;
	std::cout<<std::endl;
	for (int i=0;i<A.getRows();i++)
	{
		for(int j=0;j<A.getCols();j++)
		{
			std::cout<<A.getData()[i][j]<<" ";
		}
		std::cout<<std::endl;
	}
	std::cout<<std::endl;
}

double* rh::matrix_to_f_array(rh::Matrix &A)
{
	double *result = new double[A.getRows()*A.getCols()];
	int c = 0;
	for (int i = 0; i<A.getCols(); i++)
	{
		for(int j=0; j<A.getRows();j++)
		{
			result[c] = A.getData()[j][i];
			c++;
		}
	}
	return result;
}

double* rh::matrix_to_array(rh::Matrix &A)
{
	double *result = new double[A.getRows()*A.getCols()];
	int c = 0;
	std::cout<<A.getRows()<<std::endl;
	std::cout<<A.getCols()<<std::endl;
	for(int i = 0; i<A.getRows();i++)
	{
		for(int j=0; j<A.getCols();j++)
		{
			result[c] = A.getData()[i][j];
			c++;
		}
	}
	return result;
}

double* rh::vector2array(std::vector<double> const& vec)
{
	double* result = new double[vec.size()];
	for(int i=0;i<vec.size();i++)
		result[i] = vec[i];
	return result;
}

