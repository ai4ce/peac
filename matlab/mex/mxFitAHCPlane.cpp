//
// Copyright 2014 Mitsubishi Electric Research Laboratories All
// Rights Reserved.
//
// Permission to use, copy and modify this software and its
// documentation without fee for educational, research and non-profit
// purposes, is hereby granted, provided that the above copyright
// notice, this paragraph, and the following three paragraphs appear
// in all copies.
//
// To request permission to incorporate this software into commercial
// products contact: Director; Mitsubishi Electric Research
// Laboratories (MERL); 201 Broadway; Cambridge, MA 02139.
//
// IN NO EVENT SHALL MERL BE LIABLE TO ANY PARTY FOR DIRECT,
// INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
// LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
// DOCUMENTATION, EVEN IF MERL HAS BEEN ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGES.
//
// MERL SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND MERL HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE,
// SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
#include "mex.h"
#include "matrix.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <fstream>
#include <iomanip>

//opencv
#include "opencv2/opencv.hpp"

#include "AHCPlaneFitter.hpp"

struct MatlabImage3D {
	const double* src; //pointer to a matlab matrix of xyz ( WxHx3 )
	const int w;
	const int h;
	const int yStride;
	const int zStride;

	MatlabImage3D(const double* pSrc, const int width, const int height)
		: src(pSrc), w(width), h(height), yStride(w*h), zStride(2*w*h)
	{}
	
	inline int width() const { return w; }
	inline int height() const { return h; }
	
	bool get(const int row, const int col, double &x, double &y, double &z) const {
		const int pixIdx = col * h + row; //note matlab data is in col-major

		z=src[pixIdx+zStride];
		if(mxIsNaN(z)) return false;
		x=src[pixIdx];
		y=src[pixIdx+yStride];
		
		return true;
	}
};//MatlabImage3D

typedef ahc::PlaneFitter<MatlabImage3D> MatlabKinectPlaneFitter;
MatlabKinectPlaneFitter gFitter;

void reportParams() {
	mexPrintf("[mxFitAHCPlane] parameters:\n");
#define reportVar(var,flag,type) mexPrintf("%25s = " #flag "\n", #var, (type)gFitter.var)
	reportVar(params.depthSigma,%g,double);
	reportVar(params.stdTol_merge,%f,double);
	reportVar(params.stdTol_init,%f,double);
	reportVar(params.similarityTh_merge,%f,double);
	reportVar(params.similarityTh_refine,%f,double);
	reportVar(params.angle_far,%f,double);
	reportVar(params.angle_near,%f,double);
	reportVar(params.z_near,%f,double);
	reportVar(params.z_far,%f,double);
	reportVar(params.depthAlpha,%f,double);
	reportVar(params.depthChangeTol,%f,double);
	reportVar(params.initType,%d,int);

	reportVar(windowWidth,%d,int);
	reportVar(windowHeight,%d,int);
	reportVar(maxStep,%d,int);
	reportVar(minSupport,%d,int);
	reportVar(doRefine,%d,int);
	reportVar(erodeType,%d,int);
#undef reportVar
}

enum FitterParams {
	FP_depthSigma=0,
	FP_stdTol_merge,
	FP_stdTol_init,
	FP_similarityTh_merge,
	FP_similarityTh_refine,
	FP_angle_far,
	FP_angle_near,
	FP_z_near,
	FP_z_far,
	FP_windowWidth,
	FP_windowHeight,
	FP_maxStep,
	FP_minSupport,
	FP_doRefine,
	FP_depthAlpha,
	FP_depthChangeTol,
	FP_erodeType,
	FP_initType,
	FP_drawCoarseBorder,
	FP_TOTAL
};
void setParams(const int *param_keys, const double *param_vals, const int nParams) {
	for(int i=0; i<nParams; ++i) {
		const int key = param_keys[i];
		const double val=param_vals[i];
		switch(key) {
		case FP_depthSigma: gFitter.params.depthSigma=val; break;
		case FP_stdTol_merge: gFitter.params.stdTol_merge=val; break;
		case FP_stdTol_init: gFitter.params.stdTol_init=val; break;
		case FP_similarityTh_merge: gFitter.params.similarityTh_merge=val; break;
		case FP_similarityTh_refine: gFitter.params.similarityTh_refine=val; break;
		case FP_angle_far: gFitter.params.angle_far=val; break;
		case FP_angle_near: gFitter.params.angle_near=val; break;
		case FP_z_near: gFitter.params.z_near=val; break;
		case FP_z_far: gFitter.params.z_far=val; break;
		case FP_depthAlpha: gFitter.params.depthAlpha=val; break;
		case FP_depthChangeTol: gFitter.params.depthChangeTol=val; break;
		case FP_initType: gFitter.params.initType=(ahc::InitType)(int)val; break;
		
		case FP_windowWidth: gFitter.windowWidth=(int)val; break;
		case FP_windowHeight: gFitter.windowHeight=(int)val; break;
		case FP_maxStep: gFitter.maxStep=(int)val; break;
		case FP_minSupport: gFitter.minSupport=(int)val; break;
		case FP_doRefine: gFitter.doRefine=(val!=0); break;
		case FP_drawCoarseBorder: gFitter.drawCoarseBorder=(val!=0); break;
		case FP_erodeType: gFitter.erodeType=(ahc::ErodeType)(int)val; break;
		default:
			mexPrintf("[mxFitAHCPlane.setFitterParams warn] "
			"ignore invalid parameter pair (%d,%f)\n",
			(int)key, val);
		}
	}
	reportParams();
}

void rowMajorIdx2colMajorIdx(std::vector<int>& ids, const int w, const int h) {
	for(int i=0; i<(int)ids.size(); ++i) {
		int &id = ids[i];
		const int c=id%w;
		const int r=id/w;
		id = c*h+r+1; //matlab use 1-based col-major index
	}
}

/* The matlab mex function */
// ret=mxFitAHCPlane(xyz): run plane fitting
// mxFitAHCPlane(param_keys, param_vals): reset parameters
// mxFitAHCPlane(): report current parameters
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
	if (nrhs==0) {
		reportParams();
		return;
	}
	if (nrhs==2) {
		if (!mxIsClass(prhs[0],"int32") || !mxIsDouble(prhs[1])) {
			mexErrMsgIdAndTxt("MATLAB:mxFitAHCPlane:argin",
				"When calling mxFitAHCPlane(param_keys, param_vals),"
				"param_keys must be of type int32 and "
				"param_vals must be of type double!");
		}
		const int* keys = (const int*)mxGetData(prhs[0]);
		const double* vals = mxGetPr(prhs[1]);
		const int nParams_key = (int)mxGetNumberOfElements(prhs[0]);
		const int nParams_val = (int)mxGetNumberOfElements(prhs[1]);
		if(nParams_key!=nParams_val) {
			mexErrMsgIdAndTxt("MATLAB:mxFitAHCPlane:argin",
				"When calling mxFitAHCPlane(param_keys, param_vals),"
				"the two arguments should have same length!");
		}
		setParams(keys, vals, nParams_key);
		return;
	}
	if (nrhs>1) {
		mexErrMsgIdAndTxt("MATLAB:mxFitAHCPlane:argin",
			"Too much input parameters!");
	}

	const int nDims = mxGetNumberOfDimensions(prhs[0]);
	const mwSize* dims = mxGetDimensions(prhs[0]);
	const int h=dims[0];
	const int w=dims[1];

	if (nDims!=3 || dims[2]!=3) {
		mexErrMsgIdAndTxt("MATLAB:mxFitAHCPlane:argin",
			"When calling ret=mxFitAHCPlane(xyz), xyz should be of size WxHx3!");
	}
	if (nlhs==0) {
		mexPrintf("[mxFitAHCPlane warn] no output parameter, skip.");
		return;
	}

	MatlabImage3D mi3d((double*)mxGetPr(prhs[0]), w, h);
	std::vector<std::vector<int>> ret;
	gFitter.run(&mi3d, &ret);

	mxArray *output = mxCreateCellMatrix((mwSize)ret.size(),1);
	plhs[0]=output;
	for(int i=0; i<(int)ret.size(); ++i) {
		std::vector<int>& plid=ret[i];
		rowMajorIdx2colMajorIdx(plid, w, h);
		mxArray *out=mxCreateNumericMatrix((int)plid.size(), 1, mxINT32_CLASS, mxREAL);
		mxSetCell(output, i, out);
		int* p = (int*)mxGetData(out);
		memcpy(p, &plid[0], sizeof(int)*plid.size());
	}
}//mexFunction