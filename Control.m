%%% Closed Loop System
num = n1;den = [d2 d1 d0];
sysTF = tf(num,den);

%%% Step Response

[x,t] = step(sysTF);

x = 2*thetad*x;

figure(1);clf;
plot(t,x);