#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;

MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos)) 	//0
((SO3, rot)) 	//3
((vect3, vel)) 	//6
((vect3, omg)) 	//9
((vect3, acc))	//12
);

MTK_BUILD_MANIFOLD(process_noise_ikfom,
((vect3, ng))
((vect3, na))
);

MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001);
	return cov;
}

Eigen::Matrix<double, 15, 1> get_f(state_ikfom &s) {
	Eigen::Matrix<double, 15, 1> res = Eigen::Matrix<double, 15, 1>::Zero();
	vect3 a_w = s.rot*s.acc;
	for(int i = 0; i < 3; i++ ) {
		res(i) = s.vel[i];
		res(i + 3) = s.omg[i];
		res(i + 6) = a_w[i];
	}
	return res;
}

Eigen::Matrix<double, 15, 15> df_dx(state_ikfom &s) {
	Eigen::Matrix<double, 15, 15> cov = Eigen::Matrix<double, 15, 15>::Zero();
	cov.template block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(6, 3) = -s.rot.toRotationMatrix()*MTK::hat((s.acc));
	cov.template block<3, 3>(6, 12) = s.rot.toRotationMatrix(); 
	cov.template block<3, 3>(3, 9) = Eigen::Matrix3d::Identity(); 
	return cov;
}

Eigen::Matrix<double, 15, 6> df_dw(state_ikfom &s) {
	Eigen::Matrix<double, 15, 6> cov = Eigen::Matrix<double, 15, 6>::Zero();
	cov.template block<3, 3>(12, 3) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();
	return cov;
}

#endif