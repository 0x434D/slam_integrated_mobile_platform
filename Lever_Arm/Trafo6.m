function PB = Trafo6(PA,x)
% Trafo6 fuehrt eine 6-Parameter Transformation fuer die gegebenen Punkte 
% mit den Transformationsparametern in x aus.
% -------------------------------------------------------------------------
% Autor: Johannes Ernst
% -------------------------------------------------------------------------
% Die Transformationsparameter sind Rotation um x-Achse, Rotation um
% y-Achse, Rotation um z-Achse (je in [rad]), Offset in X, Offset in Y und 
% Offset in Z (in dieser Reihenfolge). Die Reihenfolge der Drehung bei der
% Transformation ist Rz*Ry*Rx mit den Rotationsmatrizen von Wikipedia, 
% welche eine math. positive Drehrichtung beschreiben (geg. Uhrzeigersinn):
% https://en.wikipedia.org/wiki/Rotation_matrix
% -------------------------------------------------------------------------
% Input:    PA = Punkte im System A [X1 Y1 Z1; X2 ...]
%           x  = Transformationsparameter [rx; ry; rz; cx; cy; cz]
% Outut:    PB = Transformierte Punkte im System B [X1 Y1 Z1; X2 ...]

% Initialisieren der Loesungsmatrix
PB = zeros(size(PA));

% Gleichungssystem 6-Parameter Transformation mit Rotationsreihenfolge Rz*Ry*Rx
syms rx ry rz cx cy cz XA YA ZA
XB = @(rx,ry,rz,cx,cy,cz,XA,YA,ZA) cx+((cos(ry)*cos(rz))*XA+(sin(rx)*sin(ry)*cos(rz)-cos(rx)*sin(rz))*YA+(cos(rx)*sin(ry)*cos(rz)+sin(rx)*sin(rz))*ZA);
YB = @(rx,ry,rz,cx,cy,cz,XA,YA,ZA) cy+((cos(ry)*sin(rz))*XA+(sin(rx)*sin(ry)*sin(rz)+cos(rx)*cos(rz))*YA+(cos(rx)*sin(ry)*sin(rz)-sin(rx)*cos(rz))*ZA);
ZB = @(rx,ry,rz,cx,cy,cz,XA,YA,ZA) cz+(    (-sin(ry))   *XA+            (sin(rx)*cos(ry))            *YA+            (cos(rx)*cos(ry))            *ZA);

% Alle Punkte im neuen System bestimmen
for i = 1:size(PA,1)
    PB(i,:) = [XB(x(1),x(2),x(3),x(4),x(5),x(6),PA(i,1),PA(i,2),PA(i,3))...
               YB(x(1),x(2),x(3),x(4),x(5),x(6),PA(i,1),PA(i,2),PA(i,3))... 
               ZB(x(1),x(2),x(3),x(4),x(5),x(6),PA(i,1),PA(i,2),PA(i,3))];
end
end