%% Project SIMP - Lever Arm Estimation
% Author: Johannes Ernst (3220104)
clearvars
close all
format longG
clc
fprintf('Project SIMP - Lever Arm Estimation\n')

%% Load Data
% Scanner points (target 1-9) [m]
SP = [0 0 0;...         
      0 6 0;...
      6 6 0;...
      6 0 0;...
      0 6 6;...
      6 6 6;...
      6 0 6;...
      3 6 3;...
      6 3 3];
 
% Tachymeter points (target 1-9) [m]
params = [pi/8; 0; pi/4; 5; 5; 2];     % simulate rotated coord system
TP = Trafo6(SP,params);

% New points (tachymeter system) [m]
NP = [3 3 3;2 2 2];

%% Visualize Data 
% Plot tachymeter points and add number
figure
scatTP = scatter3(TP(:,1),TP(:,2),TP(:,3),'filled');
axis equal
hold on
for i = 1:length(TP)
    text(TP(i,1)+0.2,TP(i,2),TP(i,3)+0.2,{"Z" + num2str(i)})
end

% Plot scanner points and add number
scatSP = scatter3(SP(:,1),SP(:,2),SP(:,3),'r','filled');
for i = 1:length(SP)
    text(SP(i,1)+0.2,SP(i,2),SP(i,3)+0.2,{"Z" + num2str(i)})
end

% Plot new points and add number
scatNP = scatter3(NP(:,1),NP(:,2),NP(:,3),'g','filled');
for i = 1:size(NP,1)
    text(NP(i,1)+0.2,NP(i,2),NP(i,3)+0.2,{"N" + num2str(i)})
end

% Plot connection lines to estimate rotation
for i = 1:length(SP)
    plot3([TP(i,1);SP(i,1)],[TP(i,2);SP(i,2)],[TP(i,3);SP(i,3)],'--k')
end

% Add title and legend
title('Before Transformation')
legend([scatTP scatSP scatNP],{'Tachymeter','Scanner','New'})


%% Estimate transformation and transform
estParam = Est6Trafo3D(TP,SP,[0;0;0.5;0;0;0],1e-12);
TPtrans = Trafo6(TP,estParam);
NPtrans = Trafo6(NP,estParam);
rmsError = rms(SP-TPtrans);
fprintf('Estimated parameters [rotX,rotY,rotZ,offX,offY,offZ]:\n[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', estParam(1),estParam(2),estParam(3),estParam(4),estParam(5),estParam(6));
fprintf('Root-Mean-Squared Error in [X,Y,Z]:\n[%.3f, %.3f, %.3f]\n', rmsError(1),rmsError(2),rmsError(3));

% Plot tachymeter points and add number
figure
scatTP = scatter3(TPtrans(:,1),TPtrans(:,2),TPtrans(:,3),'filled');
axis equal
hold on
scatSP = scatter3(SP(:,1),SP(:,2),SP(:,3),'r','filled');
for i = 1:length(TP)
    text(TPtrans(i,1)+0.2,TPtrans(i,2),TPtrans(i,3)+0.2,{"Z" + num2str(i)})
    text(SP(i,1)+0.2,SP(i,2),SP(i,3)+0.2,{"Z" + num2str(i)})
end
scatNP = scatter3(NPtrans(:,1),NPtrans(:,2),NPtrans(:,3),'g','filled');
for i = 1:size(NPtrans,1)
    text(NPtrans(i,1)+0.2,NPtrans(i,2),NPtrans(i,3)+0.2,{"N" + num2str(i)})
end
title('After Transformation (TP -> SP)')
legend([scatTP scatSP scatNP],{'Tachymeter','Scanner','New'})

% Calculate lever arm in scanner system
leverArmT = NP(2,:)-NP(1,:);
leverArmS = NPtrans(2,:)-NPtrans(1,:);
fprintf('Lever arm in Tachymeter system [X,Y,Z]:\n[%.3f, %.3f, %.3f]\n', leverArmT(1),leverArmT(2),leverArmT(3));
fprintf('Lever arm in Scanner system [X,Y,Z]:\n[%.3f, %.3f, %.3f]\n', leverArmS(1),leverArmS(2),leverArmS(3));
