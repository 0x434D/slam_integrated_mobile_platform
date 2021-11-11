clc
clear all
close all

load('trajektories.mat')
%load('trajectory_odometry_slam_two_loops.mat')
x=Xgt
X=Xsl
%x=trajectory;
%  x=[1.9 0.8
%     0.9 -0.9]*x';

figure


for i=1:30
%% itter
plot(X(:,1),X(:,2),'b.-')
hold on
D=pdist2(X,x);
[dm,idmin]=min(D,[],1);

one=ones(size(x(:,1)));
zero=zeros(size(x(:,1)));
%*[  1       2      3      4     5     6   ]
A=[x(:,1) x(:,2) zero    zero   one   zero  
    zero  zero   x(:,1)  x(:,2) zero   one ]; 
 
%   x1  1 2    X1   5
%   x2 =3 4  * X2 + 6     ->   x=R*X+t


p=A\[X(idmin,1);X(idmin,2)];

R=[p(1) p(2)
    p(3) p(4)];
t=repmat([p(5) p(6)],length(x),1);

x= [R*x']'+t;





plot(x(:,1),x(:,2),'r.-')
lx=[X(idmin,1) x(:,1) one(idmin)*nan]';
ly=[X(idmin,2) x(:,2) one(idmin)*nan]';

plot(lx(:),ly(:),'k')
hold off
drawnow
pause(1)
i
end

