#ifndef __MARGINALIZATION_H
#define __MARGINALIZATION_H

#include "Eigen/Dense"
#include "Eigen/SparseCore"
#include "Eigen/SparseCholesky"
#include "settings.h"
#include "globalFuncs.h"

using namespace Eigen;
using namespace std;

struct MARGINALIZATION
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MatrixXd Ap;
  VectorXd bp ;
	int size;

	MARGINALIZATION()
	{
    Ap = MatrixXd::Zero(variablesNumInState * slidingWindowSize, variablesNumInState * slidingWindowSize);
    bp = VectorXd::Zero(variablesNumInState * slidingWindowSize);
		size = 0;
	}
	~MARGINALIZATION(){
	}

	void initPrior()
	{
		size = 1;
		Ap.block(0, 0, variablesNumInState, variablesNumInState).setIdentity();
    		Ap.block(0, 0, 3, 3) *= SQ(1000000);
    		Ap.block(3, 3, 3, 3) *= SQ(1000000);
    		Ap.block(6, 6, 3, 3) *= SQ(1000000);
    bp.segment(0, variablesNumInState) = VectorXd::Zero(variablesNumInState);
	}

	void marginalizeOutTheEnd()
	{
		//Schur complement
    MatrixXd BD_inv = Ap.block(0, STATE_SZ(size - 1), STATE_SZ(size - 1), variablesNumInState)
			* Ap.block(STATE_SZ(size - 1), STATE_SZ(size - 1), variablesNumInState, variablesNumInState).inverse();
		Ap.block(0, 0, STATE_SZ(size - 1), STATE_SZ(size - 1)) -= 
			BD_inv*Ap.block(STATE_SZ(size - 1), 0, variablesNumInState, STATE_SZ(size - 1) );
		bp.segment(0, STATE_SZ(size - 1)) -= 
			BD_inv*bp.segment(STATE_SZ(size - 1), variablesNumInState );

		//set the rest zero
		bp.segment( STATE_SZ(size - 1), variablesNumInState).setZero() ;
		Ap.block(0, STATE_SZ(size - 1), STATE_SZ(size) , variablesNumInState).setZero();
		Ap.block( STATE_SZ(size - 1), 0, variablesNumInState, STATE_SZ(size) ).setZero();

		size-- ;
	}

	void popFrontState()
	{
		//swap cols 
		Ap.block(0, STATE_SZ(size - 2), STATE_SZ(size), variablesNumInState ).swap( 
			Ap.block(0, STATE_SZ(size - 1), STATE_SZ(size), variablesNumInState ));

		//swap rows
		bp.segment(STATE_SZ(size - 2), variablesNumInState).swap(
			bp.segment(STATE_SZ(size - 1), variablesNumInState));
		Ap.block(STATE_SZ(size - 2), 0, variablesNumInState, STATE_SZ(size) ).swap( 
			Ap.block(STATE_SZ(size - 1), 0, variablesNumInState, STATE_SZ(size) ) );

		marginalizeOutTheEnd();
	}

	void popEndState()
	{
		//swap cols 
    MatrixXd tmp = Ap.block(0, 0, STATE_SZ(size), variablesNumInState);
		for (int i = 1; i < size; i++) {
			Ap.block(0, STATE_SZ(i - 1), STATE_SZ(size), variablesNumInState) 
				= Ap.block(0, STATE_SZ(i), STATE_SZ(size), variablesNumInState );
		}
		Ap.block(0, STATE_SZ(size - 1), STATE_SZ(size), variablesNumInState ) = tmp;

		//swap rows
		tmp = Ap.block(0, 0, variablesNumInState, STATE_SZ(size) );
    VectorXd tmpb = bp.segment(0, variablesNumInState );
		for (int i = 1; i < size; i++){
			Ap.block(STATE_SZ(i - 1), 0, variablesNumInState, STATE_SZ(size) ) 
				= Ap.block(STATE_SZ(i), 0, variablesNumInState, STATE_SZ(size) );
			bp.segment(STATE_SZ(i - 1), variablesNumInState) = bp.segment(STATE_SZ(i), variablesNumInState);
		}
		Ap.block(STATE_SZ(size - 1), 0, variablesNumInState, STATE_SZ(size) ) = tmp;
		bp.segment(STATE_SZ(size - 1), variablesNumInState) = tmpb;

		marginalizeOutTheEnd();
//    ROS_WARN("[Pop out end]") ;
//    Ap.block(0, 0, 3, 3) += Matrix3d::Identity()* 1.0 / SQ(0.0001);
//    Ap.block(3, 3, 3, 3) += Matrix3d::Identity()* 1.0 / SQ(0.0001);
//    Ap.block(6, 6, 3, 3) += Matrix3d::Identity()* 1.0 / SQ(0.01/180*PI);
	}
};

#endif
