#pragma once
#include "opencv2/opencv.hpp"

#include "AHCPlaneSeg.hpp"

namespace ahc {

struct IntegralStats {
	cv::Mat Sx, Sy, Sz;
	cv::Mat Sxx, Syy, Szz;
	cv::Mat Sxy, Syz, Sxz;
	cv::Mat N;

	IntegralStats() {}

	template<typename Image3D>
	IntegralStats(const Image3D& points) { computeIntegral(points); }

	template<typename Image3D>
	void computeIntegral(const Image3D& points) {
		const int W = points.width();
		const int H = points.height();

		reset(W, H);

		cv::Mat * const pI[10] = { &Sx, &Sy, &Sz, &Sxx, &Syy, &Szz, &Sxy, &Syz, &Sxz, &N };

		for (int i = 0; i < H; ++i) {//row
			for (int j = 0; j < W; ++j) {//col
				double x=0, y=0, z=0;
				bool valid = points.get(i, j, x, y, z);
				if (!valid) { x = y = z = 0; }

				double data[10] = { x, y, z, x*x, y*y, z*z, x*y, y*z, x*z, valid ? 1.0 : 0.0 };
				for (int k = 0; k < 10; ++k) {
					cv::Mat& I = *pI[k];
					double Iup = i > 0 ? I.at<double>(i - 1, j) : 0;
					double Ileft = j > 0 ? I.at<double>(i, j - 1) : 0;
					double Iupleft = i > 0 && j > 0 ? I.at<double>(i - 1, j - 1) : 0;
					I.at<double>(i, j) = data[k] + Ileft + Iup - Iupleft;
				}
			}
		}
	}

	//compute plane using all points in [imin,imax]x[jmin,jmax]
	inline void compute(const int imax, const int imin, const int jmax, const int jmin,
		double center[3], double normal[3], double& mse, double& curvature, ahc::PlaneSeg::Stats& stats) const
	{
		const int W = Sx.cols;
		const int H = Sx.rows;
		assert(0 <= imin && imin <= imax && imax < H && 0 <= jmin && jmin <= jmax && jmax < W);

		const cv::Mat * const pI[10] = { &Sx, &Sy, &Sz, &Sxx, &Syy, &Szz, &Sxy, &Syz, &Sxz, &N };
		double n;
		double * const data[10] = {
			&stats.sx, &stats.sy, &stats.sz,
			&stats.sxx, &stats.syy, &stats.szz,
			&stats.sxy, &stats.syz, &stats.sxz,
			&n };

		for (int k = 0; k < 10; ++k) {
			const cv::Mat& I = *pI[k];

			//a:(imin,jmin), b:(imax,jmin), c:(imin,jmax), d:(imax,jmax)
			double Ia = 0, Ib = 0, Ic = 0, Id = 0;
			if (imin > 0) {
				if (jmin > 0) {
					Ia = I.at<double>(imin - 1, jmin - 1);
					Ib = I.at<double>(imax, jmin - 1);
				}
				Ic = I.at<double>(imin - 1, jmax);
			}
			Id = I.at<double>(imax, jmax);
			*data[k] = Id + Ia - Ib - Ic;
		}
		stats.N = n;

		if (n >= 4)
			stats.compute(center, normal, mse, curvature);
		else {
			mse = DBL_MAX;
			curvature = DBL_MAX;
		}
	}

	inline void reset(const int width, const int height) {
		Sx.create(height, width, CV_64FC1);
		Sy.create(height, width, CV_64FC1);
		Sz.create(height, width, CV_64FC1);
		Sxx.create(height, width, CV_64FC1);
		Syy.create(height, width, CV_64FC1);
		Szz.create(height, width, CV_64FC1);
		Sxy.create(height, width, CV_64FC1);
		Syz.create(height, width, CV_64FC1);
		Sxz.create(height, width, CV_64FC1);
		N.create(height, width, CV_64FC1);
	}
};

}