/*
 * iterativeLQR.h
 * iterativeLQR Library
 *
 * Copyright (c) 2013 University of Utah.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the authors <berg@cs.utah.edu> or the Technology
 * and Venture Commercialization office at the University of Utah
 * <801-581-7792> <http://tvc.utah.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * Utah. The software program and documentation are supplied "as is," without
 * any accompanying services from the University of Utah or the authors. The
 * University of Utah and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF UTAH OR THE AUTHORS BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
 * INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE UNIVERSITY OF UTAH OR THE AUTHORS HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE UNIVERSITY OF UTAH AND THE AUTHORS SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY OF NON-INFRINGEMENT.
 * THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
 * UTAH AND THE AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
 * UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <berg@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg
 * School of Computing
 * 50 S. Central Campus Drive,
 * Salt Lake City, UT 84112
 * United States of America
 *
 * <http://arl.cs.utah.edu/research/extendedlqr/>
 */

#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include "matrix.h"

template <size_t xDim>
void regularize(SymmetricMatrix<xDim> &Q)
{
  SymmetricMatrix<xDim> D;
  Matrix_ac<xDim, xDim> V;
  jacobi(Q, V, D); // V is eigenvector，D is eigenvalues
  for (size_t i = 0; i < xDim; ++i)
  {
    if (D(i, i) < 0)
    {
      D(i, i) = 0;
    }
  }
  Q = SymProd(V, D * ~V); // remove the eigenvalues below zero
}

static const double DEFAULTSTEPSIZE = 0.0009765625;

template <size_t aDim, typename T>
inline Matrix_ac<aDim> jacobian(const Matrix_ac<aDim> &a, const T &b, double (*f)(const Matrix_ac<aDim> &, const T &), double jStep = DEFAULTSTEPSIZE)
{
  Matrix_ac<aDim> A;
  Matrix_ac<aDim> ar(a), al(a);
  for (size_t i = 0; i < aDim; ++i)
  {
    ar[i] += jStep;
    al[i] -= jStep;
    A[i] = (f(ar, b) - f(al, b)) / (2 * jStep);
    ar[i] = al[i] = a[i];
  }
  return A;
}

template <size_t aDim>
inline Matrix_ac<aDim> jacobian(const Matrix_ac<aDim> &a, double (*f)(const Matrix_ac<aDim> &), double jStep = DEFAULTSTEPSIZE)
{
  Matrix_ac<aDim> A;
  Matrix_ac<aDim> ar(a), al(a);
  for (size_t i = 0; i < aDim; ++i)
  {
    ar[i] += jStep;
    al[i] -= jStep;
    A[i] = (f(ar) - f(al)) / (2 * jStep);
    ar[i] = al[i] = a[i];
  }
  return A;
}

template <size_t aDim, typename T, size_t yDim>
inline Matrix_ac<yDim, aDim> jacobian1(const Matrix_ac<aDim> &a, const T &b, Matrix_ac<yDim> (*f)(const Matrix_ac<aDim> &, const T &), double jStep = DEFAULTSTEPSIZE)
{
  Matrix_ac<yDim, aDim> A;
  Matrix_ac<aDim> ar(a), al(a);
  for (size_t i = 0; i < aDim; ++i)
  {
    ar[i] += jStep;
    al[i] -= jStep;
    A.insert(0, i, (f(ar, b) - f(al, b)) / (2 * jStep));
    ar[i] = al[i] = a[i];
  }
  return A;
}

template <size_t aDim, typename T1, typename T2>
inline Matrix_ac<aDim> jacobian1(const Matrix_ac<aDim> &a, const T1 &b, const T2 &c, double (*f)(const Matrix_ac<aDim> &, const T1 &, const T2 &), double jStep = DEFAULTSTEPSIZE)
{
  Matrix_ac<aDim> A;
  Matrix_ac<aDim> ar(a), al(a);
  for (size_t i = 0; i < aDim; ++i)
  {
    ar[i] += jStep;
    al[i] -= jStep;
    A[i] = (f(ar, b, c) - f(al, b, c)) / (2 * jStep);
    ar[i] = al[i] = a[i];
  }
  return A;
}

template <typename T1, size_t bDim, size_t yDim>
inline Matrix_ac<yDim, bDim> jacobian2(const T1 &a, const Matrix_ac<bDim> &b, Matrix_ac<yDim> (*f)(const T1 &, const Matrix_ac<bDim> &), double jStep = DEFAULTSTEPSIZE)
{
  Matrix_ac<yDim, bDim> B;
  Matrix_ac<bDim> br(b), bl(b);
  for (size_t i = 0; i < bDim; ++i)
  {
    br[i] += jStep;
    bl[i] -= jStep;
    // B.insert(0, i, (f(a, br) - f(a, bl)) / (2 * jStep));
    Matrix_ac<yDim> tmp = (f(a, br) - f(a, bl)) / (2 * jStep);
    if (isnan(tmp[0]))
    {
      tmp = f(a, br) / jStep;
    }
    B.insert(0, i, tmp);
    br[i] = bl[i] = b[i];
  }
  return B;
}

template <typename T1, size_t bDim, typename T2>
inline Matrix_ac<bDim> jacobian2(const T1 &a, const Matrix_ac<bDim> &b, const T2 &c, double (*f)(const T1 &, const Matrix_ac<bDim> &, const T2 &), double jStep = DEFAULTSTEPSIZE)
{
  Matrix_ac<bDim> B;
  Matrix_ac<bDim> br(b), bl(b);
  for (size_t i = 0; i < bDim; ++i)
  {
    br[i] += jStep;
    bl[i] -= jStep;
    B[i] = (f(a, br, c) - f(a, bl, c)) / (2 * jStep);
    br[i] = bl[i] = b[i];
  }
  return B;
}

template <size_t aDim, typename T>
inline SymmetricMatrix<aDim> hessian(const Matrix_ac<aDim> &a, const T &t, double (*f)(const Matrix_ac<aDim> &, const T &), double jStep = DEFAULTSTEPSIZE)
{
  SymmetricMatrix<aDim> Q;

  double p = f(a, t);

  // diag(Q)
  Matrix_ac<aDim> ar(a), al(a);
  for (size_t i = 0; i < aDim; ++i)
  {
    ar[i] += jStep;
    al[i] -= jStep;
    Q(i, i) = (f(al, t) - 2.0 * p + f(ar, t)) / (jStep * jStep);
    ar[i] = al[i] = a[i];
  }

  Matrix_ac<aDim> atr(a), atl(a), abr(a), abl(a);
  for (size_t i = 1; i < aDim; ++i)
  {
    atr[i] += jStep;
    atl[i] -= jStep;
    abr[i] += jStep;
    abl[i] -= jStep;
    for (size_t j = 0; j < i; ++j)
    {
      atr[j] += jStep;
      atl[j] += jStep;
      abr[j] -= jStep;
      abl[j] -= jStep;
      Q(i, j) = (f(abl, t) + f(atr, t) - f(atl, t) - f(abr, t)) / (4.0 * jStep * jStep);
      atr[j] = atl[j] = abr[j] = abl[j] = a[j];
    }
    atr[i] = atl[i] = abr[i] = abl[i] = a[i];
  }
  return Q;
}

template <size_t aDim>
inline SymmetricMatrix<aDim> hessian(const Matrix_ac<aDim> &a, double (*f)(const Matrix_ac<aDim> &), double jStep = DEFAULTSTEPSIZE)
{
  SymmetricMatrix<aDim> Q;

  double p = f(a);

  // diag(Q)
  Matrix_ac<aDim> ar(a), al(a);
  for (size_t i = 0; i < aDim; ++i)
  {
    ar[i] += jStep;
    al[i] -= jStep;
    Q(i, i) = (f(al) - 2.0 * p + f(ar)) / (jStep * jStep);
    ar[i] = al[i] = a[i];
  }

  Matrix_ac<aDim> atr(a), atl(a), abr(a), abl(a);
  for (size_t i = 1; i < aDim; ++i)
  {
    atr[i] += jStep;
    atl[i] -= jStep;
    abr[i] += jStep;
    abl[i] -= jStep;
    for (size_t j = 0; j < i; ++j)
    {
      atr[j] += jStep;
      atl[j] += jStep;
      abr[j] -= jStep;
      abl[j] -= jStep;
      Q(i, j) = (f(abl) + f(atr) - f(atl) - f(abr)) / (4.0 * jStep * jStep);
      atr[j] = atl[j] = abr[j] = abl[j] = a[j];
    }
    atr[i] = atl[i] = abr[i] = abl[i] = a[i];
  }
  return Q;
}

template <size_t aDim, typename T1, typename T2>
inline SymmetricMatrix<aDim> hessian1(const Matrix_ac<aDim> &a, const T1 &t1, const T2 &t2, double (*f)(const Matrix_ac<aDim> &, const T1 &, const T2 &), double jStep = DEFAULTSTEPSIZE)
{
  SymmetricMatrix<aDim> Q;

  double p = f(a, t1, t2);

  // diag(Q)
  Matrix_ac<aDim> ar(a), al(a);
  for (size_t i = 0; i < aDim; ++i)
  {
    ar[i] += jStep;
    al[i] -= jStep;
    Q(i, i) = (f(al, t1, t2) - 2.0 * p + f(ar, t1, t2)) / (jStep * jStep);
    ar[i] = al[i] = a[i];
  }

  Matrix_ac<aDim> atr(a), atl(a), abr(a), abl(a);
  for (size_t i = 1; i < aDim; ++i)
  {
    atr[i] += jStep;
    atl[i] -= jStep;
    abr[i] += jStep;
    abl[i] -= jStep;
    for (size_t j = 0; j < i; ++j)
    {
      atr[j] += jStep;
      atl[j] += jStep;
      abr[j] -= jStep;
      abl[j] -= jStep;
      Q(i, j) = (f(abl, t1, t2) + f(atr, t1, t2) - f(atl, t1, t2) - f(abr, t1, t2)) / (4.0 * jStep * jStep);
      atr[j] = atl[j] = abr[j] = abl[j] = a[j];
    }
    atr[i] = atl[i] = abr[i] = abl[i] = a[i];
  }
  return Q;
}

template <typename T1, size_t aDim, typename T2>
inline SymmetricMatrix<aDim> hessian2(const T1 &t1, const Matrix_ac<aDim> &a, const T2 &t2, double (*f)(const T1 &, const Matrix_ac<aDim> &, const T2 &), double jStep = DEFAULTSTEPSIZE)
{
  SymmetricMatrix<aDim> Q;

  double p = f(t1, a, t2);

  // diag(Q)
  Matrix_ac<aDim> ar(a), al(a);
  for (size_t i = 0; i < aDim; ++i)
  {
    ar[i] += jStep;
    al[i] -= jStep;
    Q(i, i) = (f(t1, al, t2) - 2.0 * p + f(t1, ar, t2)) / (jStep * jStep);
    ar[i] = al[i] = a[i];
  }

  Matrix_ac<aDim> atr(a), atl(a), abr(a), abl(a);
  for (size_t i = 1; i < aDim; ++i)
  {
    atr[i] += jStep;
    atl[i] -= jStep;
    abr[i] += jStep;
    abl[i] -= jStep;
    for (size_t j = 0; j < i; ++j)
    {
      atr[j] += jStep;
      atl[j] += jStep;
      abr[j] -= jStep;
      abl[j] -= jStep;
      Q(i, j) = (f(t1, abl, t2) + f(t1, atr, t2) - f(t1, atl, t2) - f(t1, abr, t2)) / (4.0 * jStep * jStep);
      atr[j] = atl[j] = abr[j] = abl[j] = a[j];
    }
    atr[i] = atl[i] = abr[i] = abl[i] = a[i];
  }
  return Q;
}

template <size_t aDim, size_t bDim, typename T2>
inline Matrix_ac<aDim, bDim> hessian12(const Matrix_ac<aDim> &a, const Matrix_ac<bDim> &b, const T2 &t2, double (*f)(const Matrix_ac<aDim> &, const Matrix_ac<bDim> &, const T2 &), double jStep = DEFAULTSTEPSIZE)
{
  Matrix_ac<aDim, bDim> Q;

  Matrix_ac<aDim> atr(a), atl(a), abr(a), abl(a);
  Matrix_ac<bDim> btr(b), btl(b), bbr(b), bbl(b);

  for (size_t i = 0; i < aDim; ++i)
  {
    atr[i] += jStep;
    atl[i] -= jStep;
    abr[i] += jStep;
    abl[i] -= jStep;
    for (size_t j = 0; j < bDim; ++j)
    {
      btr[j] += jStep;
      btl[j] += jStep;
      bbr[j] -= jStep;
      bbl[j] -= jStep;
      Q(i, j) = (f(abl, bbl, t2) + f(atr, btr, t2) - f(atl, btl, t2) - f(abr, bbr, t2)) / (4.0 * jStep * jStep);
      btr[j] = btl[j] = bbr[j] = bbl[j] = b[j];
    }
    atr[i] = atl[i] = abr[i] = abl[i] = a[i];
  }
  return Q;
}

template <size_t xDim, size_t uDim>
inline void iterativeLQR(const size_t &ell,                                                      // number of step
                         const Matrix_ac<xDim> &initState,                                       // initial states
                         const Matrix_ac<uDim> &uNominal,                                        //
                         Matrix_ac<xDim> (*g)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &), // discrete-time dynamic
                         void (*quadratizeFinalCost)(const Matrix_ac<xDim> &, SymmetricMatrix<xDim> &, Matrix_ac<xDim> &, const size_t &),
                         double (*cell)(const Matrix_ac<xDim> &),
                         void (*quadratizeCost)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &, const size_t &, Matrix_ac<uDim, xDim> &, SymmetricMatrix<xDim> &, SymmetricMatrix<uDim> &, Matrix_ac<xDim> &, Matrix_ac<uDim> &, const size_t &),
                         double (*ct)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &, const size_t &),
                         std::vector<Matrix_ac<uDim, xDim>> &L,
                         std::vector<Matrix_ac<uDim>> &l,
                         bool vis,
                         size_t &iter)
{
  size_t maxIter = 1000;

  L.resize(ell, zeros<uDim, xDim>());
  l.resize(ell, uNominal);

  std::vector<Matrix_ac<xDim>> xHat(ell + 1, zero<xDim>());    // states after last iteration
  std::vector<Matrix_ac<xDim>> xHatNew(ell + 1, zero<xDim>()); // new states after every iteration
  std::vector<Matrix_ac<uDim>> uHat(ell);                      // control after last iteration
  std::vector<Matrix_ac<uDim>> uHatNew(ell);                   // new control after every iteration

  double oldCost = -log(0.0);

  for (iter = 0; iter < maxIter; ++iter)
  {
    double newCost;
    double alpha = 1.0;

    do
    {
      newCost = 0;

      // beginning point
      xHatNew[0] = initState;
      for (size_t t = 0; t < ell; ++t)
      {
        uHatNew[t] = (1.0 - alpha) * uHat[t] + L[t] * (xHatNew[t] - (1.0 - alpha) * xHat[t]) + alpha * l[t]; // paper:to implement line search, alpha is the line search parameter
        //                std::cout << "t: "<< t << " uHatNew:   " <<   ~uHatNew[t] ;
        xHatNew[t + 1] = g(xHatNew[t], uHatNew[t]);
        //                std::cout << "xHatNew[t]  :  " <<   ~(xHatNew[t]) ;
        //                std::cout << "xHatNew[t+1]:  " <<   ~(xHatNew[t+1]) ;
        newCost += ct(xHatNew[t], uHatNew[t], t); // the cost for time t
      }
      //            std::cout << "times: "<< iter << " newCost:" <<   newCost << std::endl;
      newCost += cell(xHatNew[ell]); // the cost for the last point

      alpha *= 0.5;
    } while (!(newCost < oldCost || fabs((oldCost - newCost) / newCost) < 1e-4));

    xHat = xHatNew;
    uHat = uHatNew;

    if (vis)
    {
      std::cerr << "Iter: " << iter << " Alpha: " << 2 * alpha << " Rel. progress: " << (oldCost - newCost) / newCost << " Cost: " << newCost << " Time step: " << exp(xHat[0][xDim - 1]) << std::endl;
    }
    double thres = fabs((oldCost - newCost) / newCost);
    if (thres < 1e-4)
    {
      std::cerr << "l 331 of iterativeLQR.h " << fabs((oldCost - newCost) / newCost) << std::endl;
      if (thres == 0)
      {
        std::cerr << "cnmmmm" << std::endl;
      }

      return;
    }
    else
    {
      std::cerr << "l 335 of iterativeLQR.h " << fabs((oldCost - newCost) / newCost) << std::endl;
    }

    oldCost = newCost;

    // pass
    SymmetricMatrix<xDim> S;
    Matrix_ac<xDim> s;

    quadratizeFinalCost(xHat[ell], S, s, iter);

    for (size_t t = ell - 1; t != -1; --t)
    { // iteration from last point
      const Matrix_ac<xDim, xDim> A = jacobian1(xHat[t], uHat[t], g);
      const Matrix_ac<xDim, uDim> B = jacobian2(xHat[t], uHat[t], g);
      const Matrix_ac<xDim> c = xHat[t + 1] - A * xHat[t] - B * uHat[t];

      Matrix_ac<uDim, xDim> P;
      SymmetricMatrix<xDim> Q;
      SymmetricMatrix<uDim> R;
      Matrix_ac<xDim> q;
      Matrix_ac<uDim> r;

      quadratizeCost(xHat[t], uHat[t], t, P, Q, R, q, r, iter); // get P Q R q r
      //            std::cout << "B=\n" << B << std::endl;
      //            std::cout << "S=\n" << A << std::endl;
      //            std::cout << "R=\n" << R << std::endl;
      //            std::cout << "P=\n" << P << std::endl;
      const Matrix_ac<uDim, xDim> C = ~B * S * A + P;
      const SymmetricMatrix<xDim> D = SymProd(~A, S * A) + Q;
      const SymmetricMatrix<uDim> E = SymProd(~B, S * B) + R;
      const Matrix_ac<xDim> d = ~A * (s + S * c) + q;
      const Matrix_ac<uDim> e = ~B * (s + S * c) + r;

      //            std::cout << "E=\n" << E << std::endl;
      //            std::cout << "C=\n" << C << std::endl;
      // std::cout << "l 454 of iterativeLQR.h " << std::endl;
      L[t] = -(E % C);
      l[t] = -(E % e);
      //            std::cout << "L=\n" << L[t] << std::endl;
      //            std::cout << "l=\n" << l[t] << std::endl;
      S = D + SymProd(~C, L[t]);
      s = d + ~C * l[t];
    }
  }
}

template <size_t xDim, size_t uDim>
inline bool iterativeLQRR(const size_t &ell,                                                      // number of step
                          const Matrix_ac<xDim> &initState,                                       // initial states
                          const Matrix_ac<uDim> &uNominal,                                        //
                          Matrix_ac<xDim> (*g)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &), // discrete-time dynamic
                          void (*quadratizeFinalCost)(const Matrix_ac<xDim> &, SymmetricMatrix<xDim> &, Matrix_ac<xDim> &, const size_t &),
                          double (*cell)(const Matrix_ac<xDim> &),
                          void (*quadratizeCost)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &, const size_t &, Matrix_ac<uDim, xDim> &, SymmetricMatrix<xDim> &, SymmetricMatrix<uDim> &, Matrix_ac<xDim> &, Matrix_ac<uDim> &, const size_t &),
                          double (*ct)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &, const size_t &),
                          std::vector<Matrix_ac<uDim, xDim>> &L,
                          std::vector<Matrix_ac<uDim>> &l,
                          bool vis,
                          size_t &iter)
{
  size_t maxIter = 1000;

  L.resize(ell, zeros<uDim, xDim>());
  l.resize(ell, uNominal);

  std::vector<Matrix_ac<xDim>> xHat(ell + 1, zero<xDim>());    // states after last iteration
  std::vector<Matrix_ac<xDim>> xHatNew(ell + 1, zero<xDim>()); // new states after every iteration
  std::vector<Matrix_ac<uDim>> uHat(ell);                      // control after last iteration
  std::vector<Matrix_ac<uDim>> uHatNew(ell);                   // new control after every iteration

  double oldCost = -log(0.0);

  for (iter = 0; iter < maxIter; ++iter)
  {
    double newCost;
    double alpha = 1.0;

    do
    {
      newCost = 0;

      // beginning point
      xHatNew[0] = initState;
      for (size_t t = 0; t < ell; ++t)
      {
        uHatNew[t] = (1.0 - alpha) * uHat[t] + L[t] * (xHatNew[t] - (1.0 - alpha) * xHat[t]) + alpha * l[t]; // paper:to implement line search, alpha is the line search parameter
        //                std::cout << "t: "<< t << " uHatNew:   " <<   ~uHatNew[t] ;
        xHatNew[t + 1] = g(xHatNew[t], uHatNew[t]);
        //                std::cout << "xHatNew[t]  :  " <<   ~(xHatNew[t]) ;
        //                std::cout << "xHatNew[t+1]:  " <<   ~(xHatNew[t+1]) ;
        newCost += ct(xHatNew[t], uHatNew[t], t); // the cost for time t
      }
      //            std::cout << "times: "<< iter << " newCost:" <<   newCost << std::endl;
      newCost += cell(xHatNew[ell]); // the cost for the last point

      alpha *= 0.5;
    } while (!(newCost < oldCost || fabs((oldCost - newCost) / newCost) < 1e-4));

    xHat = xHatNew;
    uHat = uHatNew;

    if (vis)
    {
      std::cerr << "Iter: " << iter << " Alpha: " << 2 * alpha << " Rel. progress: " << (oldCost - newCost) / newCost << " Cost: " << newCost << " Time step: " << exp(xHat[0][xDim - 1]) << std::endl;
    }
    double thres = fabs((oldCost - newCost) / newCost);
    if (thres < 1e-4)
    {
      // std::cerr << "l 331 of iterativeLQR.h " << fabs((oldCost - newCost) / newCost) << std::endl;
      if (thres == 0)
      {
        std::cerr << "cnmmmm" << std::endl;
        return false;
      }
      return true;
    }
    // else
    // {
    //   std::cerr << "l 335 of iterativeLQR.h " << fabs((oldCost - newCost) / newCost) << std::endl;
    // }

    oldCost = newCost;

    // pass
    SymmetricMatrix<xDim> S;
    Matrix_ac<xDim> s;

    quadratizeFinalCost(xHat[ell], S, s, iter);

    for (size_t t = ell - 1; t != -1; --t)
    { // iteration from last point
      const Matrix_ac<xDim, xDim> A = jacobian1(xHat[t], uHat[t], g);
      const Matrix_ac<xDim, uDim> B = jacobian2(xHat[t], uHat[t], g);
      const Matrix_ac<xDim> c = xHat[t + 1] - A * xHat[t] - B * uHat[t];

      Matrix_ac<uDim, xDim> P;
      SymmetricMatrix<xDim> Q;
      SymmetricMatrix<uDim> R;
      Matrix_ac<xDim> q;
      Matrix_ac<uDim> r;

      quadratizeCost(xHat[t], uHat[t], t, P, Q, R, q, r, iter); // get P Q R q r
      //            std::cout << "B=\n" << B << std::endl;
      //            std::cout << "S=\n" << A << std::endl;
      //            std::cout << "R=\n" << R << std::endl;
      //            std::cout << "P=\n" << P << std::endl;
      const Matrix_ac<uDim, xDim> C = ~B * S * A + P;
      const SymmetricMatrix<xDim> D = SymProd(~A, S * A) + Q;
      const SymmetricMatrix<uDim> E = SymProd(~B, S * B) + R;
      const Matrix_ac<xDim> d = ~A * (s + S * c) + q;
      const Matrix_ac<uDim> e = ~B * (s + S * c) + r;
      if (isnan(E[0]))
      {
        std::cout << "l 572 of iterativeLQR.h " << std::endl;
        // SymProd is product, shouldn't give NAN

        std::cout << "B" << B << std::endl;
        if (isnan(B[0]))
        {
          std::cout << "l 578 of iterativeLQR.h " << std::endl;
          std::cout << "xHat[t]" << xHat[t] << std::endl;
          std::cout << "uHat[t]" << uHat[t] << std::endl;
          std::cout << "g" << g << std::endl;
          Matrix_ac<xDim, uDim> B0;
          Matrix_ac<uDim> br(uHat[t]), bl(uHat[t]);
          std::cout << "check jacobian" << std::endl;
          for (size_t i = 0; i < uDim; ++i)
          {
            br[i] += DEFAULTSTEPSIZE;
            bl[i] -= DEFAULTSTEPSIZE;
            B0.insert(0, i, (g(xHat[t], br) - g(xHat[t], bl)) / (2 * DEFAULTSTEPSIZE));
            std::cout << (g(xHat[t], br) - g(xHat[t], bl)) / (2 * DEFAULTSTEPSIZE)<< "\t" << g(xHat[t], br) <<"\t" << g(xHat[t], bl)  << std::endl;
            br[i] = bl[i] = uHat[t][i];
          }

        }
        // B = jacobian2(xHat[t], uHat[t], g); should not give nan
        std::cout << "S" << S << std::endl;
        if (isnan(S[0]))
        {
          std::cout << "l 587 of iterativeLQR.h " << std::endl;
          std::cout << "xHat[ell]" << xHat[ell] << std::endl;
          std::cout << "iter" << iter << std::endl;
        }
        
        // quadratizeFinalCost(xHat[ell], S, s, iter);
        std::cout << "R" << R << std::endl;
        if (isnan(R[0]))
        {
          std::cout << "l 596 of iterativeLQR.h " << std::endl;
        }
        
        // quadratizeCost(xHat[t], uHat[t], t, P, Q, R, q, r, iter);
      }

      //            std::cout << "E=\n" << E << std::endl;
      //            std::cout << "C=\n" << C << std::endl;
      // std::cout << "l 572 of iterativeLQR.h " << std::endl;
      L[t] = -(E % C);
      l[t] = -(E % e);
      //            std::cout << "L=\n" << L[t] << std::endl;
      //            std::cout << "l=\n" << l[t] << std::endl;
      S = D + SymProd(~C, L[t]);
      s = d + ~C * l[t];
    }
  }
  return false;
}

template <size_t xDim, size_t uDim>
inline double extendedLQR(const size_t &ell, const Matrix_ac<xDim> &startState, const Matrix_ac<uDim> &uNominal, Matrix_ac<xDim> (*g)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &), Matrix_ac<xDim> (*gBar)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &), void (*quadratizeFinalCost)(const Matrix_ac<xDim> &, SymmetricMatrix<xDim> &, Matrix_ac<xDim> &, const size_t &), double (*cell)(const Matrix_ac<xDim> &), void (*quadratizeCost)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &, const size_t &, Matrix_ac<uDim, xDim> &, SymmetricMatrix<xDim> &, SymmetricMatrix<uDim> &, Matrix_ac<xDim> &, Matrix_ac<uDim> &, const size_t &), double (*ct)(const Matrix_ac<xDim> &, const Matrix_ac<uDim> &, const size_t &), std::vector<Matrix_ac<uDim, xDim>> &L, std::vector<Matrix_ac<uDim>> &l, bool vis, size_t &iter)
{
  size_t maxIter = 1000;

  // Initialization
  L.resize(ell, zeros<uDim, xDim>());
  l.resize(ell, uNominal);

  std::vector<SymmetricMatrix<xDim>> S(ell + 1, zeros<xDim>());
  std::vector<Matrix_ac<xDim>> s(ell + 1, zero<xDim>());
  std::vector<SymmetricMatrix<xDim>> SBar(ell + 1);
  std::vector<Matrix_ac<xDim>> sBar(ell + 1);

  double oldCost = -log(0.0);
  Matrix_ac<xDim> xHat = startState;

  SBar[0] = zeros<xDim>();
  sBar[0] = zero<xDim>();

  for (iter = 0; iter < maxIter; ++iter)
  {
    // forward pass
    for (size_t t = 0; t < ell; ++t)
    {
      const Matrix_ac<uDim> uHat = L[t] * xHat + l[t];
      const Matrix_ac<xDim> xHatPrime = g(xHat, uHat);

      const Matrix_ac<xDim, xDim> ABar = jacobian1(xHatPrime, uHat, gBar);
      const Matrix_ac<xDim, uDim> BBar = jacobian2(xHatPrime, uHat, gBar);
      const Matrix_ac<xDim> cBar = xHat - ABar * xHatPrime - BBar * uHat;

      Matrix_ac<uDim, xDim> P;
      SymmetricMatrix<xDim> Q;
      SymmetricMatrix<uDim> R;
      Matrix_ac<xDim> q;
      Matrix_ac<uDim> r;

      quadratizeCost(xHat, uHat, t, P, Q, R, q, r, iter);

      const SymmetricMatrix<xDim> SBarQ = SBar[t] + Q;
      const Matrix_ac<xDim> sBarqSBarQcBar = sBar[t] + q + SBarQ * cBar;

      const Matrix_ac<uDim, xDim> CBar = ~BBar * SBarQ * ABar + P * ABar;
      const SymmetricMatrix<xDim> DBar = SymProd(~ABar, SBarQ * ABar);
      const SymmetricMatrix<uDim> EBar = SymProd(~BBar, SBarQ * BBar) + R + SymSum(P * BBar);
      const Matrix_ac<xDim> dBar = ~ABar * sBarqSBarQcBar;
      const Matrix_ac<uDim> eBar = ~BBar * sBarqSBarQcBar + r + P * cBar;
      // std::cout << "l 631 of iterativeLQR.h " << std::endl;
      L[t] = -(EBar % CBar);
      l[t] = -(EBar % eBar);

      SBar[t + 1] = DBar + SymProd(~CBar, L[t]);
      sBar[t + 1] = dBar + ~CBar * l[t];

      xHat = -((S[t + 1] + SBar[t + 1]) % (s[t + 1] + sBar[t + 1]));
    }

    // backward pass
    quadratizeFinalCost(xHat, S[ell], s[ell], iter);
    xHat = -((S[ell] + SBar[ell]) % (s[ell] + sBar[ell]));

    for (size_t t = ell - 1; t != -1; --t)
    {
      const Matrix_ac<uDim> uHat = L[t] * xHat + l[t];
      const Matrix_ac<xDim> xHatPrime = gBar(xHat, uHat);

      const Matrix_ac<xDim, xDim> A = jacobian1(xHatPrime, uHat, g);
      const Matrix_ac<xDim, uDim> B = jacobian2(xHatPrime, uHat, g);
      const Matrix_ac<xDim> c = xHat - A * xHatPrime - B * uHat;

      Matrix_ac<uDim, xDim> P;
      SymmetricMatrix<xDim> Q;
      SymmetricMatrix<uDim> R;
      Matrix_ac<xDim> q;
      Matrix_ac<uDim> r;

      quadratizeCost(xHatPrime, uHat, t, P, Q, R, q, r, iter);

      const Matrix_ac<uDim, xDim> C = ~B * S[t + 1] * A + P;
      const SymmetricMatrix<xDim> D = SymProd(~A, S[t + 1] * A) + Q;
      const SymmetricMatrix<uDim> E = SymProd(~B, S[t + 1] * B) + R;
      const Matrix_ac<xDim> d = ~A * (s[t + 1] + S[t + 1] * c) + q;
      const Matrix_ac<uDim> e = ~B * (s[t + 1] + S[t + 1] * c) + r;
      // std::cout << "l 667 of iterativeLQR.h " << std::endl;
      L[t] = -(E % C);
      l[t] = -(E % e);

      S[t] = D + SymProd(~C, L[t]);
      s[t] = d + ~C * l[t];

      xHat = -((S[t] + SBar[t]) % (s[t] + sBar[t]));
    }

    // compute cost
    double newCost = 0;
    Matrix_ac<xDim> x = xHat;
    for (size_t t = 0; t < ell; ++t)
    {
      Matrix_ac<uDim> u = L[t] * x + l[t];
      newCost += ct(x, u, t);
      x = g(x, u);
    }
    newCost += cell(x);

    if (vis)
    {
      std::cerr << "Iter: " << iter << " Rel. progress: " << (oldCost - newCost) / newCost << " Cost: " << newCost << " Time step: " << exp(xHat[xDim - 1]) << std::endl;
    }
    if (fabs((oldCost - newCost) / newCost) < 1e-4 || fabs(newCost) < 10)
    {
      ++iter;
      return exp(xHat[xDim - 1]);
    }
    oldCost = newCost;
  }

  return exp(xHat[xDim - 1]);
}
