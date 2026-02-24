

#pragma once

namespace Aetherion::Spatial {

	// ad operator hat map
	template<class S> Eigen::Matrix<S, 6, 6> ad_hat(const Eigen::Matrix<S, 6, 1>&
		xi) {
		Eigen::Matrix<S, 3, 3> W = skew(xi.template head<3>()); // ω hat
		Eigen::Matrix<S, 3, 3> V = skew(xi.template tail<3>()); // v hat
		Eigen::Matrix<S, 6, 6> ad;
		ad.template topLeftCorner<3, 3>() = W;
		ad.template topRightCorner<3, 3>() = Eigen::Matrix<S, 3, 3>::Zero();
		// x r ∈ R3 q ∈ H ξ = [ω; v] ∈
			R6 m x˙ = f(t, x, ctx) M
			f ξ˙ = M−1(f − adT(ξ)Mξ) r˙ = R(q)v q˙ =
			Ω(ω)q 2
			
			ad.template bottomLeftCorner<3, 3>() = V;
		ad.template bottomRightCorner<3, 3>() = W;
		return ad;
	}
	// ad^T operator (dual): kullanarak M ile birlikte
	template<class S> Eigen::Matrix<S, 6, 1> adT(const Eigen::Matrix<S, 6, 1>& xi,
		const Eigen::Matrix<S, 6, 6>& Mxi) {
		// ad^T(ξ) M ξ = [ -ω×, -v×; 0 -ω× ] * M ξ
		Eigen::Matrix<S, 3, 1> omega = xi.template head<3>();
		Eigen::Matrix<S, 3, 1> v = xi.template tail<3>();
		Eigen::Matrix<S, 3, 1> top = skew(omega).transpose() * Mxi.template
			head<3>()
			+ skew(v).transpose() * Mxi.template
			tail<3>();
		Eigen::Matrix<S, 3, 1> bot = skew(omega).transpose() * Mxi.template
			tail<3>();
		Eigen::Matrix<S, 6, 1> out;
		out.template head<3>() = -top;
		out.template tail<3>() = -bot;
		return out;
	}

}