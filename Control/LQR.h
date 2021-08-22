#ifndef _LQR_
#define _LQR_

#include <iostream>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <math.h>

template <int Dim>
using Vector = Eigen::Matrix<double, Dim, 1>;

template <int Dim>
using SymmetricMatrix = Eigen::Matrix<double, Dim, Dim>;

template <int rDim, int cDim>
using Matrix = Eigen::Matrix<double, rDim, cDim>;


template <int xDim, int uDim>
inline bool computeK(Vector<xDim> &K, const Matrix<xDim, xDim> &A, const Matrix<uDim, xDim> &B, const Matrix<xDim, xDim> &QCost, const Matrix<uDim, uDim> &RCost, const double &tolerance, const int max_iter) {
	//Q: Cost of State
	//R: Cost of Control
	//Discrete Version
	//https://github.com/TakaHoribe/Riccati_Solver/blob/master/riccati_solver.cpp
	MRatrix<xDim, xDim> P = Q;
	Matrix<xDim, xDim> P_next;

	Matrix<xDim, xDim> AT = A.transpose();
	Matrix<xDim, xDim> BT = B.transpose();
	Matrix<xDim, xDim> RI = R.inverse();

	double diff;

	for (int i = 0; i < max_iter; i++) {
		P_next = AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
		diff = fabs((P_next - P).maxCoeff());
		P = P_next;
		if (diff < tolerance) {
			std::cout << "Iteration mumber = " << i << std::endl;
			K = R.inverse() * BT * P;
			return true;
		}
	}
	return false;
}

#endif