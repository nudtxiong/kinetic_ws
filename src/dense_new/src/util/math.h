#ifndef MATH_H 
#define MATH_H

#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace Eigen;

class Math
{
  public:

    Matrix3d Skew(const Vector3d& w)
    {
      Matrix3d W;
      W <<     0, -w(2),  w(1),
            w(2),     0, -w(0),
           -w(1),  w(0),     0;
      return W;
    }

    MatrixXd JacobianF(const Matrix3d& R, const Vector3d& a, const Vector3d& w, double dt)
    {
      MatrixXd F = MatrixXd::Identity(9,9);
      F.block<3,3>(0,3) += Matrix3d::Identity() * dt;
      F.block<3,3>(3,6) += -R * Skew(a) * dt;
      F.block<3,3>(6,6) += -Skew(w) * dt;
      return F;
    }

    MatrixXd JacobianG(const Matrix3d& R, const Vector3d& a, const Vector3d& w, double dt)
    {
      MatrixXd G = MatrixXd::Zero(9,6);
      G.block<3,3>(3,0) += -R * dt;
      G.block<3,3>(6,3) += -Matrix3d::Identity() * dt;
      return G;
    }

    VectorXd ResidualImu(const Vector3d& p1, const Vector3d& v1, const Matrix3d& R1,
                         const Vector3d& p2, const Vector3d& v2, const Matrix3d& R2,
                         const Vector3d& g,  double dt,
                         const Vector3d& dp, const Vector3d& dv, const Matrix3d& dR)
    {
      VectorXd      r = VectorXd::Zero(9);
      r.segment<3>(0) = ( R1.transpose() * (p2 - p1 + g * dt * dt / 2.0) - v1 * dt ) - dp;
      r.segment<3>(3) = ( R1.transpose() * (R2 * v2 + g * dt) - v1 ) - dv;
      r.segment<3>(6) = Quaterniond(dR.transpose() * R1.transpose() * R2).vec() * 2.0;
      return r;
    }

    MatrixXd JacobianImu(const Vector3d& p1, const Vector3d& v1, const Matrix3d& R1,
                         const Vector3d& p2, const Vector3d& v2, const Matrix3d& R2,
                         const Vector3d& g, double dt)
    {
      MatrixXd J = MatrixXd::Zero(9, 18);
      J.block<3,3>(0,0)  = -R1.transpose();
      J.block<3,3>(0,3)  = -Matrix3d::Identity() * dt;
      J.block<3,3>(3,3)  = -Matrix3d::Identity();
      J.block<3,3>(0,6)  = Skew( R1.transpose() * (p2 - p1 + g * dt * dt / 2.0) );
      J.block<3,3>(3,6)  = Skew( R1.transpose() * (R2 * v2 + g * dt) );
      J.block<3,3>(6,6)  = -R2.transpose() * R1;
      J.block<3,3>(0,9)  = R1.transpose();
      J.block<3,3>(3,12) = R1.transpose() * R2;
      J.block<3,3>(3,15) = -R1.transpose() * R2 * Skew(v2);
      J.block<3,3>(6,15) = Matrix3d::Identity();
      return J;
    }

    VectorXd ResidualCamera(const Vector3d& p1, const Matrix3d& R1, 
                            const Vector3d& p2, const Matrix3d& R2, 
                            double d1, const Vector2d& uv1, 
                            const Vector2d& uv2)
    {
      Matrix3d Ric;
      Ric << 0, -1,  0,
             0,  0, -1,
             1,  0,  0;
      Vector3d uv1h = Vector3d(uv1(0), uv1(1), 1);
      Vector3d f = Ric * R2.transpose() * (p1 - p2 + d1 * R1 * Ric.transpose() * uv1h);
      Vector2d r = Vector2d(f(0) / f(2), f(1) / f(2)) - uv2;
      return r;
    }

    MatrixXd JacobianCamera(const Vector3d& p1, const Matrix3d& R1, 
                            const Vector3d& p2, const Matrix3d& R2, 
                            double d1, const Vector2d& uv1)
    {
      MatrixXd J = MatrixXd::Zero(2, 19); 
      Matrix3d Ric;
      Ric << 0, -1,  0,
             0,  0, -1,
             1,  0,  0;
      Vector3d uv1h = Vector3d(uv1(0), uv1(1), 1);
      // Projection
      MatrixXd H = MatrixXd::Zero(2, 3);
      Vector3d f = Ric * R2.transpose() * (p1 - p2 + d1 * R1 * Ric.transpose() * uv1h);
      H(0,0)     = 1.0 / f(2);
      H(0,2)     = -f(0) / (f(2) * f(2));
      H(1,1)     = 1.0 / f(2);
      H(1,2)     = -f(1) / (f(2) * f(2));
      // Feature frame transform
      MatrixXd JF         = MatrixXd::Zero(3, 19);
      JF.block<3,3>(0,0)  = R2.transpose();
      JF.block<3,3>(0,6)  = -R2.transpose() * R1 * Skew( d1 * Ric.transpose() * uv1h ); 
      JF.block<3,3>(0,9)  = -R2.transpose();
      JF.block<3,3>(0,15) = Skew( R2.transpose() * (p1 - p2 + d1 * R1 * Ric.transpose() * uv1h) );
      JF.block<3,1>(0,18) = R2.transpose() * R1 * Ric.transpose() * uv1h;
      // Assemble
      J = H * Ric * JF;
      return J;
    }

    VectorXd ResidualDenseTracking(const Vector3d& pi, const Matrix3d& Ri,
                            const Vector3d& pj, const Matrix3d& Rj,
                            const Vector3d& Tij, const Matrix3d& Rij )
    {
      VectorXd r = VectorXd::Zero(9);
      r.segment<3>(0) = Rj.transpose()*(pi - pj) - Tij;
      r.segment<3>(6) = Quaterniond(Rij.transpose() * Rj.transpose() * Ri).vec() * 2.0;

      return  r;
    }

    MatrixXd JacobianDenseTracking(const Vector3d& pi, const Matrix3d& Ri,
                            const Vector3d& pj, const Matrix3d& Rj )
    {
      MatrixXd J = MatrixXd::Zero(9, 18);
      J.block<3,3>(0,0)  = Rj.transpose();
      J.block<3,3>(0,9)  = -Rj.transpose();
      J.block<3,3>(0,15) = Skew( Rj.transpose()*(pi-pj) ) ;
      J.block<3,3>(6,6)  = Matrix3d::Identity();
      J.block<3,3>(6,15) = -Ri.transpose()*Rj ;
      return J;
    }
};
#endif
