function x = Est6Trafo3D(PA, PB, xinit, t)
% Est6Trafo3D bestimmt die Parameter einer 6-Parameter Transformation fuer 
% gegebene identische Punkte in zwei Systemen (Kl.-Quadrate-Ausgleichung).
% -------------------------------------------------------------------------
% Autor: Johannes Ernst
% -------------------------------------------------------------------------
% Die 6 unbekannten Parameter sind Rotation um x-Achse, Rotation um
% y-Achse, Rotation um z-Achse, Offset in X, Offset in Y und Offset in Z 
% (in dieser Reihenfolge). Die Reihenfolge der Drehung bei der 
% Transformation ist Rz*Ry*Rx mit den Rotationsmatrizen von Wikipedia, 
% welche eine math. positive Drehrichtung beschreiben (geg. Uhrzeigersinn):
% https://en.wikipedia.org/wiki/Rotation_matrix
%
% Eine gute Wahl von Initialwerten (vor allem fuer die Rotationen) ist
% wichtig, da die Ausgleichung sonst moeglicherweise nicht konvergiert!
% -------------------------------------------------------------------------
% Input:    PA      = identische Punkte im System A [X1 Y1 Z1; X2 ...]
%           PB      = identische Punkte im System B [X1 Y1 Z1; X2 ...]
%           xinit   = Naeherungswerte fuer die unbekannten Parameter
%           t       = Wert fuer das Abbruchkriterium der Iteration
% Outut:    x       = 6 unbekannte Parameter (rx, ry, rz, cx, cy, cz)

% Modellparameter
G = eye(size(PA,1)*3,size(PA,1)*3);     % Gewichtsmatrix
x0 = xinit;                             % Unbekannte am Taylorpunkt
itMax = 15;                             % Maximale Anzahl an Iterationen

for iter = 1:itMax
    % Auswertung der Transformation am Taylorpunkt
    P0 = Trafo6(PA,x0);

    % Widerspruchsvektor bestimmen
    w = reshape(PB',[size(PB,1)*3,1]) - reshape(P0',[size(P0,1)*3,1]);

    % Designmatrix bauen (Ableitungen aus symbolischer Auswertung)
    A = zeros(size(PA,1)*3,6);
    for i = 1:size(PA,1)
        [rx,ry,rz,XA,YA,ZA] = deal(x0(1),x0(2),x0(3),PA(i,1),PA(i,2),PA(i,3));
        A(3*i-2:3*i,:) = [ (YA*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) + ZA*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry))), (ZA*cos(rx)*cos(ry)*cos(rz) - XA*cos(rz)*sin(ry) + YA*cos(ry)*cos(rz)*sin(rx)), -(YA*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz)) - ZA*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) + XA*cos(ry)*sin(rz)), 1, 0, 0;...
                          -(YA*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) + ZA*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz))), (ZA*cos(rx)*cos(ry)*sin(rz) - XA*sin(ry)*sin(rz) + YA*cos(ry)*sin(rx)*sin(rz)),  (ZA*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) - YA*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry)) + XA*cos(ry)*cos(rz)), 0, 1, 0;...
                                                                                   (YA*cos(rx)*cos(ry) - ZA*cos(ry)*sin(rx)),                        -(XA*cos(ry) + ZA*cos(rx)*sin(ry) + YA*sin(rx)*sin(ry)),                                                                                                                       0, 0, 0, 1];
    end

    % Verbesserungen
    Dx = (A'*G*A)\A'*G*w;

    % Unbekannte Parameter updaten
    x0 = x0 + Dx;

    % % Residuen
    % vhat = A*Dx+w;

    % Abbruchkriterium 
    if norm(Dx) < t
        % fprintf('Konvergenz nach %.0f Iterationen\n',iter)
        break;
    end
    if iter == 12
        error('Ausgleichung konvergiert nicht (%.0f Iterationen)',itMax)
    end
end

% % Orthogonalitaetsprobe
% check = A'*vhat

% Parameter der Transformation
x = x0;
end