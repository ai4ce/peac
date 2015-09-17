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

#include "stdlib.h"

#include "eig33sym.hpp"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
	if (nlhs==0) return;
	if (nlhs>2) {
		mexErrMsgIdAndTxt("MATLAB:mxEig33Sym:argout",
			"Please call [s,V]=mxEig33Sym(K) with at most two output arguments!");
	}
	if (nrhs!=1) {
		mexErrMsgIdAndTxt("MATLAB:mxEig33Sym:argin",
			"Please call mxEig33Sym(K) with one and only one input argument!");
	}
	if (!mxIsDouble(prhs[0])) {
		mexErrMsgIdAndTxt("MATLAB:mxEig33Sym:argin",
			"Please call mxEig33Sym(K) with double-typed K!");
	}
	if (mxGetM(prhs[0])!=3 || mxGetN(prhs[0])!=3) {
		mexErrMsgIdAndTxt("MATLAB:mxEig33Sym:argin",
			"Please call mxEig33Sym(K) with K of size 3x3!");
	}

	double K[3][3]={0}, V[3][3]={0};
	const double *pin=mxGetPr(prhs[0]);
	memcpy((void*)K, pin, sizeof(double)*9); //since K is symmetric, col-major doesn't matter

	plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
	double *s = mxGetPr(plhs[0]);
	if (!LA::eig33sym(K,s,V)) {
		mexPrintf("[mxEig33Sym error] LA::eig33sys failed!\n");
		return;
	}

	if (nlhs>1) {
		plhs[1] = mxCreateDoubleMatrix(3,3,mxREAL);
		double *v = mxGetPr(plhs[1]);
		for(int c=0, cnt=0; c<3; ++c)
			for(int r=0; r<3; ++r, ++cnt)
				*(v+cnt) = V[r][c];
	}
}