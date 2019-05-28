#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <string>

namespace rh
{
	class Matrix
	{
		public:
			Matrix(int m, int n, double **data, bool fortran=false);
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
