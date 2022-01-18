function [translation, rotScale] = combTrafo9(xOld, xNew)
% combTrafo9 combines the given transformation parameters into a 
% translation and Rotation/Scale matrix.
% -------------------------------------------------------------------------
% Autor: Johannes Ernst
% -------------------------------------------------------------------------
% Input:    x_old           = old/prior transformation parameters 
%                             (mx,my,mz, rx,ry,rz in (rad), cx,cy,cz)
%           x_new           = new/updated transformation parameters 
%                             (mx,my,mz, rx,ry,rz in (rad), cx,cy,cz)
% Output:   translation     = new scaled and rotated translation (X,Y,Z)
%           rotScale        = combination of rotation and scale matrices

% Combine scale and rotation M*Rz*Ry*Rx
rotScaleOld = [xOld(1) 0 0; 0 xOld(2) 0; 0 0 xOld(3)]*...
              [cos(xOld(6)) -sin(xOld(6)) 0; sin(xOld(6)) cos(xOld(6)) 0; 0 0 1]*...
              [cos(xOld(5)) 0 sin(xOld(5)); 0 1 0; -sin(xOld(5)) 0 cos(xOld(5))]*...
              [1 0 0; 0 cos(xOld(4)) -sin(xOld(4)); 0 sin(xOld(4)) cos(xOld(4))];
rotScaleNew = [xNew(1) 0 0; 0 xNew(2) 0; 0 0 xNew(3)]*...
              [cos(xNew(6)) -sin(xNew(6)) 0; sin(xNew(6)) cos(xNew(6)) 0; 0 0 1]*...
              [cos(xNew(5)) 0 sin(xNew(5)); 0 1 0; -sin(xNew(5)) 0 cos(xNew(5))]*...
              [1 0 0; 0 cos(xNew(4)) -sin(xNew(4)); 0 sin(xNew(4)) cos(xNew(4))];          

% Calculate translation
translation = xNew(7:9) + rotScaleNew*xOld(7:9);

% Calculate rotScale
rotScale = rotScaleNew*rotScaleOld;
end

