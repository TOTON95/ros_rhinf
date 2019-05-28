#include <ros_rhinf/matrix.h>


rh::Matrix::Matrix()
{
}

rh::Matrix::Matrix(int m, int n, double **data, bool fortran)
{
}

rh::Matrix::~Matrix()
{
}

rh::Matrix rh::Matrix::sum_matrix(rh::Matrix &A, rh::Matrix &B)
{
}

rh::Matrix rh::Matrix::sub_matrix(rh::Matrix &A, rh::Matrix &B)
{
}

void print_matrix(rh::Matrix &A, std::string title)
{
}

double* rh::Matrix::matrix_to_f_array(rh::Matrix &A)
{
}

double* rh::Matrix::matrix_to_array(rh::Matrix &A)
{
}
