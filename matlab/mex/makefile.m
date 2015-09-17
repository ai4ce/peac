%
% Copyright 2014 Mitsubishi Electric Research Laboratories All
% Rights Reserved.
%
% Permission to use, copy and modify this software and its
% documentation without fee for educational, research and non-profit
% purposes, is hereby granted, provided that the above copyright
% notice, this paragraph, and the following three paragraphs appear
% in all copies.
%
% To request permission to incorporate this software into commercial
% products contact: Director; Mitsubishi Electric Research
% Laboratories (MERL); 201 Broadway; Cambridge, MA 02139.
%
% IN NO EVENT SHALL MERL BE LIABLE TO ANY PARTY FOR DIRECT,
% INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
% LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
% DOCUMENTATION, EVEN IF MERL HAS BEEN ADVISED OF THE POSSIBILITY OF
% SUCH DAMAGES.
%
% MERL SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
% "AS IS" BASIS, AND MERL HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE,
% SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
%
function makefile(OpenCV_Lib, OpenCV_Include, Eigen_Include)
% compile the mex functions
% OpenCV_Lib: directory to opencv lib files
% OpenCV_Include: directory to opencv include files
% Eigen_Include: directory to boost include files

AHC_Include=fullfile(fileparts(mfilename('fullpath')),'..','..','include');

if nargin<1
  %note: my matlab is 64bit, so must link to 64bit libraries
  OpenCV_Lib='D:\lib\opencv_249\build\x64\vc10\lib';
  OpenCV_Include='D:\lib\opencv_249\build\include\';
  Eigen_Include='D:\lib\eigen\';
end

if ~exist(OpenCV_Lib,'dir')
  OpenCV_Lib = uigetdir([],'Please select a valid OpenCV_Lib dir');
end
if ~exist(OpenCV_Include,'dir')
  OpenCV_Include = uigetdir([],'Please select a valid OpenCV_Include dir');
end
if ~exist(Eigen_Include,'dir')
  Eigen_Include = uigetdir([],'Please select a valid Eigen_Include dir');
end

fprintf('OpenCV_Lib=%s\n',OpenCV_Lib);
fprintf('OpenCV_Include=%s\n',OpenCV_Include);
fprintf('Eigen_Include=%s\n',Eigen_Include);

mex('-v', '-O', ['-L' OpenCV_Lib],...
  '-lopencv_core249',...
  ['-I' AHC_Include],...
  ['-I' OpenCV_Include],...
  ['-I' Eigen_Include],...
  'mxFitAHCPlane.cpp');

mex('-v', '-O',...
  ['-I' AHC_Include],...
  ['-I' Eigen_Include],...
  'mxEig33Sym.cpp');
end