%% B747 Lateral LQR
clear all; close all;
A=[-.0558 -.9968 .0802 .0415; .598 -.115 -.0318 0; -3.05 .388 -.4650 0; 0 0.0805 1 0];
B=[ .00729 0; -0.475 0.00775; 0.153 0.143; 0 0];
C=[0 1 0 0; 0 0 0 1];
D=[0 0;0 0];
sys = ss(A,B,C,D);
set(sys, 'inputname', {'rudder' 'aileron'},...
'outputname', {'yaw rate' 'bank angle'});
set(sys, 'statename', {'beta' 'yaw rate' 'roll rate' 'phi'});
[Yol,Tol]=initial(ss(A,B,[1 0 0 0],zeros(1,2)),[1 0 0 0]',[0:.1:30]);
damp(A)
[V,E]=eig(A);
% CONTROL ? actuator dynamics are a lag at 10
actn=10;actd=[1 10]; % H_r(s) in notes
H=tf({actn 0;0 1},{actd 1;1 1});
%
tau=3;washn=[1 0];washd=[1 1/tau]; % washout filter on yaw rate, H_w in notes
WashFilt=tf({washn 0;0 1},{washd 1;1 1});
%
Gp=WashFilt*sys*H;
set(Gp, 'statename', {'xwo' 'beta' 'yaw rate' 'roll' 'phi' 'xa'});
set(Gp, 'inputname', {'rudder inputs' 'aileron'},...
'outputname', {'filtered yaw rate' 'bank angle'});
[Ap,Bp,Cp,Dp]=ssdata(Gp);
% \rho = 0.1 obtained from tuning
[Klqr,S,Elqr]=lqr(Ap,Bp(:,1),Cp(1,:)'*Cp(1,:),0.1);
% close the LQR loop
Acl=Ap-Bp(:,1)*Klqr;
Bcl=Bp(:,1);
% choose output to be just state beta and control u
Cpbeta=[0 1 0 0 0 0]; % performance output variable
Ccl=[Cpbeta;Klqr];
Dcl=[0;0];
xp0=[0 1 0 0 0 0]';
Glqr=ss(Acl,Bcl,Ccl,Dcl);
[Y,T]=initial(Glqr,xp0,Tol);
figure(1);clf
plot(Tol,Yol,T,Y(:,1));axis([0 30 -1 1]);
%setlines(2)
legend('OL','LQR')
ylabel('\beta');xlabel('Time')
grid on
%%% Compare to root locus approach
figure(2);clf
rlocus(Ap,Bp(:,1),-Cp(1,:),0);
sgrid([.1 .2 .3 .4],[.7 .8 .9 1]);grid on;axis([-1.4 .1 -1.2 1.2])
Kgain=-2;
Egain=eig(Ap-Bp(:,1)*Kgain*Cp(1,:));
hold on;plot(Egain+eps*sqrt(-1),'bd');hold off
Ggain=ss(Ap-Bp(:,1)*Kgain*Cp(1,:),Bp,Cpbeta,0);
[Ygain,Tgain]=initial(Ggain,xp0,Tol);
figure(3);clf
plot(Tol,Yol,T,Y(:,1),Tgain,Ygain);axis([0 30 -1 1]);
legend('OL','LQR','RL')
ylabel('\beta');xlabel('Time')
grid on