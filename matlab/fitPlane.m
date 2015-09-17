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
function [center,majorAxis,majorAxisVar]=fitPlane(Vxyz)
% fit plane from Vxyz using PCA
% Vxyz<3xn>: points expressed in world coordinate system
% center<3x1>: center of mass
% majorAxis<3x3>: from pca usually, majorAxis(:,1) is the x-direction in
% world coordinate system, majorAxis(:,2) is y-direction, majorAxis(:,3) is
% z-direction

center=mean(Vxyz,2);
Vnew = bsxfun(@minus, Vxyz, center);
K=Vnew*Vnew';
[zyxVar,zyxDir]=mxEig33Sym(K);
majorAxis=zyxDir(:,[3,2,1]);
viewDir = center/norm(center); %assume viewpoint is at (0,0,0)'
if dot(viewDir,majorAxis(:,3))>0
  majorAxis(:,3)=-majorAxis(:,3); %ensure <zdir,viewdir> <0
end
if det(majorAxis)<0 %ensure rotation matrix
  majorAxis(:,2)=-majorAxis(:,2);
end
if nargout>2
  majorAxisVar=zyxVar([3,2,1])/size(Vxyz,2);
end
end