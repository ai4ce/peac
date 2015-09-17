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
function kinect_record(fname)
if nargin<1
  fname=uiputfile;
end

msgbox('Click image to start record and click it again to stop and quit!');

hd = Kinect;
[~,rgb,depth]=hd.grab;

startRecord=false;
  function myfunc(~,~)
    if startRecord %if started
      fprintf('Record End, saved as %s\n',fname);
      hd.recordStop();
      close(hfRGB);
    else %not started yet
      startRecord=true; %let's start
      fprintf('Record Start...\n');
      hd.recordStart(fname);
    end
  end

hfRGB=figure;
rgb=uint8((double(rgb)+double(mono2rgb(depth)))/2);
hRGB=imshow(rgb);
set(hRGB,'ButtonDownFcn',@myfunc);

  function ret=rgbNotDone
    ret = ~isempty(hRGB) && ishghandle(hRGB);
  end
  function ret=notDone
    ret= rgbNotDone;
  end

while notDone
  [~,rgb,depth]=hd.grab;
  rgb=uint8((double(rgb)+double(mono2rgb(depth)))/2);
  if rgbNotDone
    set(hRGB,'CDATA',rgb);
  end
  drawnow;
end
end