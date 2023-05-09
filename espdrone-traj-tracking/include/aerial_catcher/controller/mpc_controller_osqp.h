/*
 * Copyright 2022 Wang Siqiang, NROS, HITSZ, China
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __MPC_CONTROLLER_OSQP_H__
#define __MPC_CONTROLLER_OSQP_H__
#include "OsqpEigen/OsqpEigen.h"

#include <iostream>

#include <Eigen/Eigen>
#include <stdio.h>
#include <chrono>
#include <aerial_catcher/predictor/uam_param.h>

class mpc_controller_osqp
{
private:
    uam_param param;
    int mpcWindow = 20;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 12, 12> a;
    Eigen::Matrix<double, 12, 4> b;

    // allocate the constraints vector
    Eigen::Matrix<double, 12, 1> xMax;
    Eigen::Matrix<double, 12, 1> xMin;
    Eigen::Matrix<double, 4, 1> uMax;
    Eigen::Matrix<double, 4, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 12> Q;
    Eigen::DiagonalMatrix<double, 4> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 12, 1> x0;
    Eigen::Matrix<double, 12, 1> xRef;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

public:
    mpc_controller_osqp(/* args */);
    ~mpc_controller_osqp();
    void example();
    void setDynamicsMatrices(Eigen::Matrix<double, 12, 12> &a, Eigen::Matrix<double, 12, 4> &b);
    void setInequalityConstraints(Eigen::Matrix<double, 12, 1> &xMax, Eigen::Matrix<double, 12, 1> &xMin,
                                  Eigen::Matrix<double, 4, 1> &uMax, Eigen::Matrix<double, 4, 1> &uMin);
    void setWeightMatrices(Eigen::DiagonalMatrix<double, 12> &Q, Eigen::DiagonalMatrix<double, 4> &R);
    void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 12> &Q, const Eigen::DiagonalMatrix<double, 4> &R, int mpcWindow,
                            Eigen::SparseMatrix<double> &hessianMatrix);
    void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 12> &Q, const Eigen::Matrix<double, 12, 1> &xRef, int mpcWindow,
                             Eigen::VectorXd &gradient);
    void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 12, 12> &dynamicMatrix, const Eigen::Matrix<double, 12, 4> &controlMatrix,
                                     int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix);
    void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 12, 1> &xMax, const Eigen::Matrix<double, 12, 1> &xMin,
                                      const Eigen::Matrix<double, 4, 1> &uMax, const Eigen::Matrix<double, 4, 1> &uMin,
                                      const Eigen::Matrix<double, 12, 1> &x0,
                                      int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
    void updateConstraintVectors(const Eigen::Matrix<double, 12, 1> &x0,
                                 Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
    double getErrorNorm(const Eigen::Matrix<double, 12, 1> &x,
                        const Eigen::Matrix<double, 12, 1> &xRef);
};


#endif //__MPC_CONTROLLER_OSQP_H__