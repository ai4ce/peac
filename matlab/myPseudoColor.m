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
function map=myPseudoColor(ncolors)
map=[0,0,0;
     255, 0, 0;
		 255, 255, 0;
		 100, 20, 50;
		 0, 30, 255;
		 10, 255, 60;
		 80, 10, 100;
		 0, 255, 200;
		 10, 60, 60;
		 255, 0, 128;
		 60, 128, 128]/255.0;
if ncolors>length(map)
  nMore=ncolors-length(map);
  map=[map;rand(nMore,3)];
elseif ncolors<length(map)
  map=map(1:ncolors,:);
end
end
