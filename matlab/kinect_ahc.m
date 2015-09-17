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
function [xyz,rgb,depth,mbs]=kinect_ahc(varargin)
% call kinect_ahc('rgb'); to see our fitAHCPlane run in real-time
% depends on Kinect.m

viewRGB=false;
viewXYZ=false;
viewDepth=false;
for i=1:nargin
  if strcmpi(varargin{i},'rgb')
    viewRGB=true;
  elseif strcmpi(varargin{i},'xyz')
    viewXYZ=true;
  elseif strcmpi(varargin{i},'depth')
    viewDepth=true;
  end
end
if ~viewRGB && ~viewXYZ && ~viewDepth
  xyz=[]; rgb=[]; depth=[]; mbs=[];
  return;
end

hd = Kinect;
[xyz,rgb,depth]=hd.grab;

ahcParams = getDefaultAHCFitterparams;
ahcParams.windowWidth=8;
ahcParams.windowHeight=8;
ahcParams.mergeMSETolerance=10;
% ahcParams.initMSETolerance=5;
% ahcParams.depthSigmaFactor=9e-7;
setAHCPlaneFitterParams(ahcParams);
[h,w]=size(depth);

hfRGB=[];
hfXYZ=[];
hfD=[];
if viewRGB
  figure;
  hfRGB=imshow(rgb);
end
if viewXYZ
  figure;
  [x,y,z]=splitXYZ(xyz);
  hfXYZ=plot3(x,y,z,'.');
  axis equal;
  grid on;
end
if viewDepth
  figure;
  hfD=imshow(depth); colormap('jet');
end

  function ret=rgbNotDone
    ret = viewRGB && ~isempty(hfRGB) && ishghandle(hfRGB);
  end
  function ret=xyzNotDone
    ret = viewXYZ && ~isempty(hfXYZ) && ishghandle(hfXYZ);
  end
  function ret=depthNotDone
    ret = viewDepth && ~isempty(hfD) && ishghandle(hfD);
  end
  function ret=notDone
    ret= rgbNotDone...
      || xyzNotDone...
      || depthNotDone;
  end

while notDone
  [xyz,rgb,depth]=hd.grab;
  mbs=fitAHCPlane(xyz);
  seg=createSegImg(mbs,w,h);
  rgb=(rgb+seg)/2;
  if rgbNotDone
    set(hfRGB,'CDATA',rgb);
  end
  if xyzNotDone
    [x,y,z]=splitXYZ(xyz);
    set(hfXYZ,'XDATA',x);
    set(hfXYZ,'YDATA',y);
    set(hfXYZ,'ZDATA',z);
  end
  if depthNotDone
    set(hfD,'CDATA',depth);
  end
  drawnow;
end
end