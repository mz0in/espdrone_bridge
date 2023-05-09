/*
* iterativeLQR based quadrotor path planner
*
* Copyright (c) 2015 Haoyao Chen, HITSZ, Shenzhen, China / ASL ETHZ, Switzerland
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

#include <stdio.h>
//#include <tchar.h>

#include "aerial_catcher/iterativeLQR.h"

// Set dimensions
#define X_DIM 12
#define U_DIM 4
#define DIM 3

struct Obstacle {
	Matrix_ac<DIM> pos;
	double radius;
	size_t dim;
};

size_t ell;
SymmetricMatrix<X_DIM> Q;
double rotCost;
Matrix_ac<X_DIM> xGoal, xStart;
SymmetricMatrix<U_DIM> R;
Matrix_ac<U_DIM> uNominal;
double obstacleFactor;
double scaleFactor;

std::vector<Matrix_ac<X_DIM>> xLocal;
std::vector<SymmetricMatrix<X_DIM>> Wp;
double timeStateFactor;
std::vector<double> timeLocal;

std::vector<Obstacle> obstacles;
Matrix_ac<DIM> bottomLeft;
Matrix_ac<DIM> topRight;
double robotRadius;
double dt;

double gravity;         // gravity,   m/s^2
Matrix_ac<3> eX, eY, eZ;   // unit vectors

// quadrotor constants
double mass;            // mass, kg
SymmetricMatrix<3> inertia;    // moment of inertia matrix 
double momentConst;     // ratio between force and moment of rotor
double dragConst;       // ratio between speed and opposite drag force
double rollMomentConst; // ratio between addition roll moment and the rotor translational velocity perpendicular to the rotor axis
double length;          // distance between center and rotor, m
double minForce;        // minimum force of rotor, N
double maxForce;        // maximum force of rotor, N
double motorConst;      // motor constant
// derivative constants
SymmetricMatrix<3> invInertia; // inverse of intertia matrix

inline Matrix_ac<3,3> skewSymmetric(const Matrix_ac<3>& vector) {
  Matrix_ac<3,3> result = zeros<3,3>();
  result(0,1) = -vector[2]; result(0,2) = vector[1];
  result(1,0) = vector[2];  result(1,2) = -vector[0];
  result(2,0) = -vector[1]; result(2,1) = vector[0];

  return result;
}

/// body 3-2-1 Euler angle/ world 1-2-3 Euler angle
/// roll along x-axis; then pitch along y-axis; final, yaw along z-axis (world 1-2-3)
/// equavilently, yaw long z-axis, pitch along y'-axis, and finally roll along x''-axis (body 3-2-1)
inline Matrix_ac<3,3> rpyToRotationMatrix(const float roll, const float pitch, const float yaw) {
  Matrix_ac<3,3> R, Rr, Rp, Ry;
  Rr = zeros<3,3>();
  Rp = zeros<3,3>();
  Ry = zeros<3,3>();

  Rr(0,0) = 1;
  Rr(1,1) = cos(roll);	Rr(1,2) =-sin(roll);
  Rr(2,1) = sin(roll);	Rr(2,2) = cos(roll);

  Rp(0,0) = cos(pitch); Rp(0,2) =sin(pitch);
  Rp(1,1) = 1;
  Rp(2,0) = -sin(pitch);Rp(2,2) = cos(pitch);
  
  Ry(0,0) = cos(yaw);	Ry(0,1) =-sin(yaw);
  Ry(1,0) = sin(yaw);	Ry(1,1) = cos(yaw);
  Ry(2,2) = 1;

  R = Ry * Rp * Rr;
  return R;
  //std::cout<<"cal:"<<std::endl<< R<<std::endl;
}

// Obstacle-cost term in local cost function
inline double obstacleCost(const Matrix_ac<X_DIM>& x) {
  double cost = 0;
  Matrix_ac<DIM> pos = x.subMatrix<DIM>(0,0);

  for (size_t i = 0; i < obstacles.size(); ++i) {
    Matrix_ac<DIM> d = pos - obstacles[i].pos;
    d[obstacles[i].dim] = 0;
    double dist = sqrt(scalar(~d*d)) - robotRadius - obstacles[i].radius;
    cost += obstacleFactor * exp(-scaleFactor*dist);
  }
  for (size_t i = 0; i < DIM; ++i) {
    double dist = (pos[i] - bottomLeft[i]) - robotRadius;
    cost += obstacleFactor * exp(-scaleFactor*dist);
  }
  for (size_t i = 0; i < DIM; ++i) {
    double dist = (topRight[i] - pos[i]) - robotRadius;
    cost += obstacleFactor * exp(-scaleFactor*dist);
  }
  return cost;	
}

inline void quadratizeObstacleCost(const Matrix_ac<X_DIM>& x, SymmetricMatrix<X_DIM>& Q, Matrix_ac<X_DIM>& q) {
  SymmetricMatrix<DIM> QObs = zeros<DIM>();
  Matrix_ac<DIM> qObs = zero<DIM>();

  Matrix_ac<DIM> pos = x.subMatrix<DIM>(0,0);

  for (size_t i = 0; i < obstacles.size(); ++i) {
    Matrix_ac<DIM> d = pos - obstacles[i].pos;
    d[obstacles[i].dim] = 0;
    double distr = sqrt(scalar(~d*d));
    d /= distr;
    double dist = distr - robotRadius - obstacles[i].radius;

    Matrix_ac<DIM> n = zero<DIM>();
    n[obstacles[i].dim] = 1.0;
    Matrix_ac<DIM> d_ortho = skewSymmetric(n)*d;

    double a0 = obstacleFactor * exp(-scaleFactor*dist);
    double a1 = -scaleFactor*a0;
    double a2 = -scaleFactor*a1;

    double b2 = a1 / distr;

    QObs += a2*SymProd(d,~d) + b2*SymProd(d_ortho,~d_ortho);
    qObs += a1*d;
  }
  for (size_t i = 0; i < DIM; ++i) {
    double dist = (pos[i] - bottomLeft[i]) - robotRadius;

    Matrix_ac<DIM> d = zero<DIM>();
    d[i] = 1.0;

    double a0 = obstacleFactor * exp(-scaleFactor*dist);
    double a1 = -scaleFactor*a0;
    double a2 = -scaleFactor*a1;

    QObs += a2*SymProd(d,~d);
    qObs += a1*d;
  }
  for (size_t i = 0; i < DIM; ++i) {
    double dist = (topRight[i] - pos[i]) - robotRadius;

    Matrix_ac<DIM> d = zero<DIM>();
    d[i] = -1.0;

    double a0 = obstacleFactor * exp(-scaleFactor*dist);
    double a1 = -scaleFactor*a0;
    double a2 = -scaleFactor*a1;

    QObs += a2*SymProd(d,~d);
    qObs += a1*d;
  }
  regularize(QObs);
  Q.insert(0, QObs + Q.subSymmetricMatrix<3>(0));
  q.insert(0,0, qObs - QObs*x.subMatrix<3>(0,0) + q.subMatrix<3>(0,0));
}

// Time-dependent-state cost term in local cost function
inline double timeDependentStateCost(const Matrix_ac<X_DIM>& x, const size_t& t) {
  /// time dependent state
  double cost = 0;
  double tp;
  Matrix_ac<X_DIM> pos(x);
  Matrix_ac<X_DIM> pos_d; 
  SymmetricMatrix<X_DIM> Wpi;

  for(int i=0; i<timeLocal.size(); i++) {
    tp = timeLocal[i];
    pos_d = xLocal[i]; 
    Wpi = Wp[i];
    cost += 0.5 * scalar(~(pos - pos_d)*Wpi*(pos - pos_d)*sqrt(timeStateFactor / 6.28) * exp(- timeStateFactor /2 * (t * dt - tp) * (t * dt - tp))); 
  }
  return cost;	
}

inline void quadratizeTimeDependentStateCost(const Matrix_ac<X_DIM>& x, SymmetricMatrix<X_DIM>& Q, Matrix_ac<X_DIM>& q, const size_t& t) {
  SymmetricMatrix<X_DIM> QObs = zeros<X_DIM>();
  Matrix_ac<X_DIM> qObs = zero<X_DIM>();

  Matrix_ac<X_DIM> pos(x);
  Matrix_ac<X_DIM> pos_d; 
  SymmetricMatrix<X_DIM> Wpi;
  double tp; 
  double a;
  for(int i=0; i<timeLocal.size(); i++) {
    tp = timeLocal[i];
    pos_d = xLocal[i]; 
    a = sqrt(timeStateFactor / 6.28) * exp(- timeStateFactor /2 * (t * dt - tp) * (t * dt - tp)); 
    Wpi = Wp[i];
    QObs = a * Wpi;
    qObs = a * Wpi * (pos - pos_d);

    regularize(QObs);
    Q.insert(0, QObs + Q);
    q.insert(0,0, qObs - QObs*x + q);
  }
}

// Local cost-function c_t(x_t, u_t)
inline double ct(const Matrix_ac<X_DIM>& x, const Matrix_ac<U_DIM>& u, const size_t& t) {
  double cost = 0;
  if (t == 0) {
    cost += scalar(~(x - xStart)*Q*(x - xStart));
  } else {
    cost += obstacleCost(x);
  }
  cost += scalar(~(u - uNominal)*R*(u - uNominal));
  cost += timeDependentStateCost(x, t);
  return cost;
}

inline void quadratizeCost(const Matrix_ac<X_DIM>& x, const Matrix_ac<U_DIM>& u, const size_t& t, Matrix_ac<U_DIM,X_DIM>& Pt, SymmetricMatrix<X_DIM>& Qt, SymmetricMatrix<U_DIM>& Rt, Matrix_ac<X_DIM>& qt, Matrix_ac<U_DIM>& rt, const size_t& iter) {
  /*Qt = hessian1(x, u, t, c); 
    Pt = ~hessian12(x, u, t, c);
    Rt = hessian2(x, u, t, c);
    qt = jacobian1(x, u, t, c) - Qt*x - ~Pt*u;
    rt = jacobian2(x, u, t, c) - Pt*x - Rt*u;*/
  if (t == 0) {
    Qt = Q;
    qt = -(Q*xStart);
  } else {
    Qt = zeros<X_DIM>(); 
    qt = zero<X_DIM>();

    if (iter < 1) {
      Qt.insert(6,rotCost*identity<3>());
    }

    quadratizeObstacleCost(x, Qt, qt);
    quadratizeTimeDependentStateCost(x, Qt, qt, t);
  }

  Rt = R;
  rt = -(R*uNominal);
  Pt = zeros<U_DIM,X_DIM>();
}

// Final cost function c_\ell(x_\ell)
inline double cell(const Matrix_ac<X_DIM>& x) {
  double cost = 0;
  cost += scalar(~(x - xGoal)*Q*(x - xGoal));
  return cost;
}

inline void quadratizeFinalCost(const Matrix_ac<X_DIM>& x, SymmetricMatrix<X_DIM>& Qell, Matrix_ac<X_DIM>& qell, const size_t& iter) {
  /*Qell = hessian(x, cell); 
    qell = jacobian(x, cell) - Qell*x;*/

  Qell = Q;
  qell = -(Q*xGoal);
}

/// Wrap the angle
inline void angleWrap(double &angle) {
  double c, s;
  c = cos(angle);
  s = sin(angle);
  angle = atan2(s, c);
}

// Continuous-time dynamics \dot{x} = f(x,u)
/// state: position, velocity, Euler angle, angular rate
inline Matrix_ac<X_DIM> f(const Matrix_ac<X_DIM>& x, const Matrix_ac<U_DIM>& u) {
  Matrix_ac<X_DIM> xDot;

  Matrix_ac<3> p = x.subMatrix<3,1>(0,0);
  Matrix_ac<3> v = x.subMatrix<3,1>(3,0);
  Matrix_ac<3> rpy = x.subMatrix<3,1>(6,0);
  Matrix_ac<3> w = x.subMatrix<3,1>(9,0); //body angle rate

  // \dot{p} = v
  xDot.insert(0, 0, v);

  Matrix_ac<3,3> R_WB, skew;  //R_WB transform body coordinates to world coordinates
  Matrix_ac<3> f_R_ez;
  Matrix_ac<3> R_ez;

  R_WB = rpyToRotationMatrix(rpy[0], rpy[1], rpy[2]);

  R_ez = R_WB * eZ;
  f_R_ez = (u[0] + u[1] + u[2] + u[3]) * R_ez;
  //std::cout<<"F_R_ez"<<f_R_ez<<std::endl;
  
  Matrix_ac<3,3> skew_Rez;
  Matrix_ac<3,3> skew_w;
  Matrix_ac<3,3> skew_Rez2;
  Matrix_ac<3,3> RWB_skeww;
  skew_Rez = skewSymmetric(R_ez);
  skew_w  = skewSymmetric(w);
  skew_Rez2 = skew_Rez * skew_Rez;
  RWB_skeww = R_WB * skew_w;

  Matrix_ac<3> wv_orth = zero<3>();
  Matrix_ac<3> v_p = zero<3>();
  Matrix_ac<3> w_p = zero<3>();
  Matrix_ac<3> d_i = zero<3>();
  // \dot{v} = [0,0,-g]^T + Rez*[0,0,(f_1 + f_2 + f_3 + f_4) / m]^T - sum_i(1-4) (v_p); 
  d_i[0] = length;  d_i[1] = 0;       d_i[2] = 0.01; 
  wv_orth = sqrt(u[0]/motorConst) * (- skew_Rez2) * (v + RWB_skeww * d_i); 
  v_p = v_p + dragConst * wv_orth; 
  w_p = w_p + rollMomentConst * wv_orth; 
  d_i[0] = 0;       d_i[1] = length;  d_i[2] = 0.01; 
  wv_orth = sqrt(u[1]/motorConst) * (- skew_Rez2) * (v + RWB_skeww * d_i); 
  v_p = v_p + dragConst * wv_orth; 
  w_p = w_p + rollMomentConst * wv_orth; 
  d_i[0] = -length; d_i[1] = 0;       d_i[2] = 0.01; 
  wv_orth = sqrt(u[2]/motorConst) * (- skew_Rez2) * (v + RWB_skeww * d_i); 
  v_p = v_p + dragConst * wv_orth; 
  w_p = w_p + rollMomentConst * wv_orth; 
  d_i[0] = 0;       d_i[1] = -length; d_i[2] = 0.01; 
  wv_orth = sqrt(u[3]/motorConst) * (- skew_Rez2) * (v + RWB_skeww * d_i); 
  v_p = v_p + dragConst * wv_orth; 
  w_p = w_p + rollMomentConst * wv_orth; 

  xDot.insert(3, 0, - gravity * eZ + f_R_ez / mass - v_p/mass);

  ///rpy rate here equals to world angle rate
  xDot.insert(6, 0, R_WB * w); //w wrt. body frame

  //// \dot{w} = J^{-1}*([l*(f_2 - f_4), l*(f_3 - f_1), (f_1 - f_2 + f_3 - f_4)*k_M]^T - [w]*J*w)
  xDot.insert(9, 0, invInertia * (length*(u[1] - u[3])*eX + length*(u[2] - u[0])*eY + (u[0] - u[1] + u[2] - u[3]) * momentConst*eZ - w_p - skew_w * inertia * w));

  return xDot;
}

// Discrete-time dynamics x_{t+1} = g(x_t, u_t)
inline Matrix_ac<X_DIM> g(const Matrix_ac<X_DIM>& x, const Matrix_ac<U_DIM>& u) {
  Matrix_ac<X_DIM> result;
  Matrix_ac<X_DIM> k1 = f(x, u);
  Matrix_ac<X_DIM> k2 = f(x + 0.5*dt*k1, u);
  Matrix_ac<X_DIM> k3 = f(x + 0.5*dt*k2, u);
  Matrix_ac<X_DIM> k4 = f(x + dt*k3, u);

  result = x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
  for(int i=0; i<3; i++)
    angleWrap(result[i+6]); //x[6],x[7],x[8] are Euler angle
  
  return result;

  //		Matrix_ac<X_DIM> k4 = f(x, u);
  //		return x + dt*(k4);
}

/// Return x_dot
inline Matrix_ac<X_DIM> getXdot(const Matrix_ac<X_DIM>& x, const Matrix_ac<U_DIM>& u) {

  Matrix_ac<X_DIM> x_dot;
  Matrix_ac<X_DIM> k1 = f(x, u);
  Matrix_ac<X_DIM> k2 = f(x + 0.5*dt*k1, u);
  Matrix_ac<X_DIM> k3 = f(x + 0.5*dt*k2, u);
  Matrix_ac<X_DIM> k4 = f(x + dt*k3, u);

  x_dot = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
  return x_dot;
}

// Discrete-time inverse dynamics x_t = \bar{g}(x_{t+1}, u_t)
inline Matrix_ac<X_DIM> gBar(const Matrix_ac<X_DIM>& x, const Matrix_ac<U_DIM>& u) {

  Matrix_ac<X_DIM> result;
  
  Matrix_ac<X_DIM> k1 = f(x, u);
  Matrix_ac<X_DIM> k2 = f(x - 0.5*dt*k1, u);
  Matrix_ac<X_DIM> k3 = f(x - 0.5*dt*k2, u);
  Matrix_ac<X_DIM> k4 = f(x - dt*k3, u);

  result = x - (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
  for(int i=0; i<3; i++)
    angleWrap(result[i+6]); //x[6],x[7],x[8] are Euler angle
        
  return result;
  //		Matrix_ac<X_DIM> k4 = f(x, u);
  //		return x - dt*(k4);
}
