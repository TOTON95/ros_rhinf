#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <string>
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
                        Matrix sum_matrix(Matrix &A, Matrix &B);
                        Matrix sub_matrix(Matrix &A, Matrix &B);
                        void print_matrix(Matrix &A, std::string title);
                        double *matrix_to_f_array(Matrix &A);
                        double *matrix_to_array(Matrix &A);
			~Matrix();
		private:
			double **data;
			int m;
			int n;
	};
}

#endif
