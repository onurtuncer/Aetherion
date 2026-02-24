

#pragma once 

template<class S> Eigen::Matrix<S, 3, 3> skew(const Eigen::Matrix<S, 3, 1>& v) {
	Eigen::Matrix<S, 3, 3> M;
	M << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	return M;
}