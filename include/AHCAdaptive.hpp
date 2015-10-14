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
	void computeIntegral(const Image3D& points) { //TODO: this takes ~30ms for VGA size, maybe we should do block integral instead of pixel-wise integral
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

struct Block {
	const int imin, imax, jmin, jmax;
	bool valid;

	ahc::PlaneSeg::Stats stats;
	double normal[3];
	double center[3];
	double mse;
	double curvature;

	Block() : imin(-1), imax(-1), jmin(-1), jmax(-1), valid(false) {}
	Block(int i0, int i1, int j0, int j1) : imin(i0), imax(i1), jmin(j0), jmax(j1), valid(true), mse(DBL_MAX), curvature(DBL_MAX) {}
	Block(const Block& other) : imin(other.imin), imax(other.imax), jmin(other.jmin), jmax(other.jmax), valid(other.valid),
		stats(other.stats), mse(other.mse), curvature(other.curvature)
	{
		std::copy(other.normal, other.normal + 3, normal);
		std::copy(other.center, other.center + 3, center);
	}

	inline operator cv::Rect() const { return cv::Rect(jmin, imin, IndexWidth(), IndexHeight()); }

	inline void computeOn(const IntegralStats& intStats) {
		intStats.compute(imax, imin, jmax, jmin, center, normal, mse, curvature, stats);
	}

	inline int IndexWidth() const { return jmax - jmin + 1; }
	inline int IndexHeight() const { return imax - imin + 1; }
	inline cv::Point IndexCenter() const { return cv::Point((jmax + jmin) / 2, (imax + imin) / 2); }

	inline bool canSplit(const int minBlockSize = 2) const {
		return IndexHeight() >= 2 * minBlockSize
			&& IndexWidth() >= 2 * minBlockSize;
	}

	inline bool needSplit(const double mse_threshold) const {
		return mse > mse_threshold;
	}

	inline void setFlag(const int id, cv::Mat1i& flag) const {
		const int width = IndexWidth();
		const int height = IndexHeight();
		flag(cv::Rect(jmin, imin, width, 1)) = id;
		flag(cv::Rect(jmin, imin, 1, height)) = id;
		flag(cv::Rect(jmax, imin, 1, height)) = id;
		flag(cv::Rect(jmin, imax, width, 1)) = id;
	}

	inline void split2(std::vector<Block>& output) {
		valid = false;
		if (IndexWidth() >= IndexHeight()) { //split j
			const int jmid_low = (jmin + jmax) / 2;
			const int jmid_high = jmid_low + 1;
			output.push_back(Block(imin, imax, jmin, jmid_low));
			output.push_back(Block(imin, imax, jmid_high, jmax));
		} else { //split i
			const int imid_low = (imin + imax) / 2;
			const int imid_high = imid_low + 1;
			output.push_back(Block(imin, imid_low, jmin, jmax));
			output.push_back(Block(imid_high, imax, jmin, jmax));
		}
	}

	inline void split4(std::vector<Block>& output) {
		valid = false;
		const int imid_low = (imin + imax) / 2;
		const int imid_high = imid_low + 1;
		const int jmid_low = (jmin + jmax) / 2;
		const int jmid_high = jmid_low + 1;

		output.push_back(Block(imin, imid_low, jmin, jmid_low));
		output.push_back(Block(imid_high, imax, jmin, jmid_low));
		output.push_back(Block(imid_high, imax, jmid_high, jmax));
		output.push_back(Block(imin, imid_low, jmid_high, jmax));
	}

	inline PlaneSeg::Ptr newPlaneSeg(const int rid) const {
		PlaneSeg::Ptr ret = new PlaneSeg(rid, mse, center, normal, curvature, stats);
		return ret;
	}
};

}