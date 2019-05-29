#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <string>
#include <vector>
#include <iostream>

namespace rh
{
	class Matrix
	{
		public:
			Matrix();
			Matrix(int rows, int cols, double *da, bool fortran=false);
			int getRows() {return m;}
			int getCols() {return n;}
			double** getData() {return data;}
			void setRows(int rows) {m = rows;}
			void setCols(int cols) {n = cols;}	
			void setData(double** d) {data = d;}
                        void print_matrix(Matrix &A, std::string title);
			~Matrix();
		private:
			double **data = new double*[1];
			int m = 1;
			int n = 1;
	};
	Matrix sum_matrix(Matrix &A, Matrix &B);
	Matrix sub_matrix(Matrix &A, Matrix &B);
	double *matrix_to_f_array(Matrix &A);
	double *matrix_to_array(Matrix &A);
	double *vector2array(std::vector<double> const& vec);
}

#endif
