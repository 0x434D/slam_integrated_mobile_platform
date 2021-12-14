function x = TimEst9Trafo3D(PA, PB, xinit, t)
% Est9Trafo3D bestimmt die Parameter einer 9-Parameter Transformation fuer 
% gegebene identische Punkte in zwei Systemen (Kl.-Quadrate-Ausgleichung).
% -------------------------------------------------------------------------
% Autor: Johannes Ernst
% -------------------------------------------------------------------------
% Die 9 unbekannten Parameter sind Maßstab in X,Y,Z, Rotation um x-Achse,
% y-Achse, z-Achse (je in [rad]), Offset in X,Y,Z (in dieser Reihenfolge). 
% Die Reihenfolge der Drehung bei der Transformation ist Rz*Ry*Rx mit den 
% Rotationsmatrizen von Wikipedia, welche eine math. positive Drehrichtung 
% beschreiben (geg. Uhrzeigersinn):
% https://en.wikipedia.org/wiki/Rotation_matrix
%
% Eine gute Wahl von Initialwerten (vor allem fuer die Rotationen) ist
% wichtig, da die Ausgleichung sonst moeglicherweise nicht konvergiert!
% -------------------------------------------------------------------------
% Input:    PA      = identische Punkte im System A [X1 Y1 Z1; X2 ...]
%           PB      = identische Punkte im System B [X1 Y1 Z1; X2 ...]
%           xinit   = Naeherungswerte fuer die unbekannten Parameter
%           t       = Wert fuer das Abbruchkriterium der Iteration
% Outut:    x       = 6 unbekannte Parameter (mx,my,mz, rx,ry,rz, cx,cy,cz)

% Modellparameter
x0 = xinit;                             % Unbekannte am Taylorpunkt
itMax = 15;                             % Maximale Anzahl an Iterationen

for iter = 1:itMax
    % Auswertung der Transformation am Taylorpunkt
    P0 = Trafo9(PA,x0);

    % Widerspruchsvektor bestimmen
    w = reshape(PB',[size(PB,1)*3,1]) - reshape(P0',[size(P0,1)*3,1]);

    % Designmatrix bauen (Ableitungen aus symbolischer Auswertung)
    A = zeros(size(PA,1)*3,length(x0));
    for i = 1:size(PA,1)
        [mx,my,mz,rx,ry,rz,XA,YA,ZA] = deal(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),PA(i,1),PA(i,2),PA(i,3));
        A(3*i-2:3*i,:) = [ ZA*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) - YA*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry)) + XA*cos(ry)*cos(rz), 0, 0,...
                           mx*(YA*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) + ZA*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry))),...
                           mx*(ZA*cos(rx)*cos(ry)*cos(rz) - XA*cos(rz)*sin(ry) + YA*cos(ry)*cos(rz)*sin(rx)),...
                          -mx*(YA*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz)) - ZA*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) + XA*cos(ry)*sin(rz)),...
                           1, 0, 0;...
                           0, YA*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz)) - ZA*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) + XA*cos(ry)*sin(rz), 0,...
                          -my*(YA*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) + ZA*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz))),...
                           my*(ZA*cos(rx)*cos(ry)*sin(rz) - XA*sin(ry)*sin(rz) + YA*cos(ry)*sin(rx)*sin(rz)),...
                           my*(ZA*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) - YA*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry)) + XA*cos(ry)*cos(rz)),...
                           0, 1, 0;...
                           0, 0, ZA*cos(rx)*cos(ry) - XA*sin(ry) + YA*cos(ry)*sin(rx),...
                           mz*(YA*cos(rx)*cos(ry) - ZA*cos(ry)*sin(rx)),...
                          -mz*(XA*cos(ry) + ZA*cos(rx)*sin(ry) + YA*sin(rx)*sin(ry)),...
                           0, 0, 0, 1];                        
    end

    % Verbesserungen
    Dx = (A'*A)\A'*w;

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