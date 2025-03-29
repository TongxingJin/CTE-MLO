/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ESEKFOM_EKF_HPP
#define ESEKFOM_EKF_HPP


#include <vector>
#include <cstdlib>

#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "../mtk/types/vect.hpp"
#include "../mtk/types/SOn.hpp"
#include "../mtk/types/S2.hpp"
#include "../mtk/startIdx.hpp"
#include "../mtk/build_manifold.hpp"
#include "util.hpp"
#include "common_lib.hpp"

namespace esekfom {

using namespace Eigen;

template<typename S, typename M, int measurement_noise_dof = M::DOF>
struct share_datastruct
{
	bool valid;
	bool converge;
	M z;
	Eigen::Matrix<typename S::scalar, M::DOF, S::DOF> h_x;
};

template<typename T>
struct dyn_share_datastruct
{
	bool valid;
	bool converge;
	Eigen::Matrix<T, Eigen::Dynamic, 1> z;
	Eigen::Matrix<T, Eigen::Dynamic, 1> h;
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
};

template<typename T>
struct dyn_runtime_share_datastruct
{
	bool valid;
	bool converge;
	//Z z;
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
};

template<typename state, int process_noise_dof, typename input = state, typename measurement=state, int measurement_noise_dof=0>
class esekf{

	typedef esekf self;
	enum{
		n = state::DOF, m = state::DIM, l = measurement::DOF
	};

public:
	
	typedef typename state::scalar scalar_type;
	typedef Matrix<scalar_type, n, n> cov;
	typedef Matrix<scalar_type, m, n> cov_;
	typedef Matrix<scalar_type, n, 1> vectorized_state;
	typedef Matrix<scalar_type, m, 1> flatted_state;
	typedef flatted_state processModel(state &);
	typedef Eigen::Matrix<scalar_type, m, n> processMatrix1(state &);
	typedef Eigen::Matrix<scalar_type, m, process_noise_dof> processMatrix2(state &);
	typedef Eigen::Matrix<scalar_type, process_noise_dof, process_noise_dof> processnoisecovariance;
	typedef measurement measurementModel(state &, bool &);
	//typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, 1> measurementModel_dyn_share(state &,  dyn_share_datastruct<scalar_type> &);
	typedef void measurementModel_dyn_share(state &,  dyn_share_datastruct<scalar_type> &);
	typedef Eigen::Matrix<scalar_type ,l, n> measurementMatrix1(state &, bool&);

	esekf(const state &x = state(),
		const cov  &P = cov::Identity()): x_(x), P_(P){
	};


	void init_dyn_share(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in, measurementModel_dyn_share h_dyn_share_in, int maximum_iteration, scalar_type limit_vector[n])
	{
		f = f_in;
		f_x = f_x_in;
		f_w = f_w_in;
		h_dyn_share = h_dyn_share_in;

		maximum_iter = maximum_iteration;
		for(int i=0; i<n; i++)
		{
			limit[i] = limit_vector[i];
		}

		x_.build_S2_state();
		x_.build_SO3_state();
		x_.build_vect_state();
	}

	void predict(double &dt, processnoisecovariance &Q) {
		flatted_state f_ = f(x_);
		cov_ f_x_ = f_x(x_);
		Matrix<scalar_type, m, process_noise_dof> f_w_ = f_w(x_);
		x_.oplus(f_, dt);
	
		F_x1 = cov::Identity();
		F_x1 += f_x_*dt;//f_x_final * dt;
		Eigen::Matrix<double, 3, 1> omg;
		omg(0,0) = f_(3);
		omg(1,0) = f_(4);
		omg(2,0) = f_(5);
		F_x1.template block<3, 3>(3, 3) = Exp(omg,-dt); //BCH
		P_ = (F_x1) * P_ * (F_x1).transpose() + (dt * f_w_) * Q * (dt * f_w_).transpose();
	}	

	void predict_pub(double &dt, processnoisecovariance &Q) {
		x_pub_ = x_;
		flatted_state f_ = f(x_pub_);
		cov_ f_x_ = f_x(x_pub_);
		Matrix<scalar_type, m, process_noise_dof> f_w_ = f_w(x_pub_);
		x_pub_.oplus(f_, dt);
	
		cov F_x1_ = cov::Identity();
		F_x1_ += f_x_*dt;//f_x_final * dt;
		Eigen::Matrix<double, 3, 1> omg;
		omg(0,0) = f_(3);
		omg(1,0) = f_(4);
		omg(2,0) = f_(5);
		F_x1_.template block<3, 3>(3, 3) = Exp(omg,-dt); //BCH
		P_pub_ = F_x1_ * P_ * F_x1_.transpose() + (dt * f_w_) * Q * (dt * f_w_).transpose();
	}	

	void update_iterated_dyn_share_modified(double &solve_time) {		
		dyn_share_datastruct<scalar_type> dyn_share;
		dyn_share.valid = true;
		dyn_share.converge = true;
		
		for(int i=0; i<maximum_iter; i++) {
			dyn_share.valid = true;	
			h_dyn_share(x_, dyn_share);
			if(! dyn_share.valid) continue;

			Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_ = dyn_share.h_x;
			
			double solve_start = omp_get_wtime();
			vectorized_state dx;
			cov P_temp = P_.inverse();
			Eigen::Matrix<scalar_type, 15, 15> HTH = h_x_.transpose() * h_x_; 
			P_temp. template block<15, 15>(0, 0) += HTH;

			P_ = P_temp.inverse();
			Matrix<scalar_type, n, 1> dx_ = P_. template block<n, 15>(0, 0) * h_x_.transpose() * dyn_share.h;
			x_.boxplus(dx_);
			dyn_share.converge = true;
			solve_time += omp_get_wtime() - solve_start;

			for(int i = 0; i < n ; i++) {
				if(std::fabs(dx_[i]) > limit[i]) {
					dyn_share.converge = false;
					break;
				}
			}
			if (dyn_share.converge) break;
		}
	}

	void change_x(state &input_state)
	{
		x_ = input_state;
		if((!x_.vect_state.size())&&(!x_.SO3_state.size())&&(!x_.S2_state.size()))
		{
			x_.build_S2_state();
			x_.build_SO3_state();
			x_.build_vect_state();
		}
	}

	void change_P(cov &input_cov)
	{
		P_ = input_cov;
	}

	const state& get_x() const {
		return x_;
	}
	const cov& get_P() const {
		return P_;
	}
	const state& get_x_pub() const {
		return x_pub_;
	}
	const cov& get_P_pub() const {
		return P_pub_;
	}
private:
	state x_;
	state x_pub_;
	cov P_;
	cov P_pub_;
	cov F_x1 = cov::Identity();

	processModel *f;
	processMatrix1 *f_x;
	processMatrix2 *f_w;

	measurementModel *h;
	measurementMatrix1 *h_x;

	measurementModel_dyn_share *h_dyn_share;

	int maximum_iter = 0;
	scalar_type limit[n];
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace esekfom

#endif //  ESEKFOM_EKF_HPP
