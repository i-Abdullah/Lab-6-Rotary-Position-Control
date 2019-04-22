%% info:

%  flexible arm dynamics can be modeled as the rigid arm
%  (a spring-mass-damper system) plus an additional spring-mass- damper


%% housekeepign:

clear
clc
close all

%% closed loop transfer constants:

Kg = 33.3 ; % no units
Km = 0.0401 ; %V/(rad/sec);
Rm = 19.2 ; % Ohms

Jhub = 0.0005;
Jload = 0.0015;
J = Jhub+Jload ;
LinkLength = 0.45;
Marm = 0.06 ; %kg
Jarm = 0.004 ; %kgm^2
Mtip = 0.05 ; %kg
JM = 0.01;
fc = 1.8 ; %HZ

JL = Jarm + JM;

Karm = (2*pi*fc)^2 *(JL) ;

% note:
% K1 = Kp?
% K2 = Kpd
% K3 = KD?
% K4 = KDd

K1 = linspace(0,5,5);
K2 = linspace(50,0,5);
K3 = linspace(0,1.5,5);
K4 = linspace(0,1.5,5);

% definitions in lab docuement:


p1 = - (Kg^2 * Km^2) / (Jhub*Rm) ;
p2 = (Kg^2 * Km^2 * LinkLength ) / (Jhub*Rm) ;

q1 = Karm / ( LinkLength*Jhub) ;
q2 = - ( (Karm*( Jhub + JL )) / (JL*Jhub)) ;

r1 = (Kg*Km)/(Jhub*Rm) ;
r2 = - (Kg*Km*LinkLength)/(Jhub*Rm);

% l = lambda;

l3 = -p1 + K3.*r1 + K4.*r2 ;
l2 = -q2 + K1.*r1 + K2.*r2 + K4.*(p2*r1 - r2*pi) ;
l1 = p1*q2 - q1*p2 + K3.*(q1*r2 - r1*q2) + K2.*(p2*r1 - r2*p1) ;
l0 = K1.*(q1*r2 - r1*q2);

% The angle of the arm (or output shaft) is denoted by, ?L



% Closed Loop System
num1 = K1 .* [ r1 ; 0 ; (q1*r2 - r1*q2) ] ;
den1 = [1 ; l3 ; l2 ; l1 ; l0];

System = tf(num,den);

%%% Step Response

[x,t] = step(System);

x = 2*thetad*x;

figure(1);clf;
plot(t,x);