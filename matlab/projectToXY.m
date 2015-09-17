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
function Vxy=projectToXY(Vxyz,majorAxis)
% project Vxyz to x-y plane using the major axis
% Vxyz<3xn>: points expressed in world coordinate system
% majorAxis<3x3>: from pca usually, majorAxis(:,1) is the x-direction in
% world coordinate system, majorAxis(:,2) is y-direction, majorAxis(:,3) is
% z-direction

Vxy=majorAxis'*Vxyz;
Vxy=Vxy(1:2,:);
end