clear all;
close all;
clc;

%%
%//////////////////
b1 = 82.4
L1 = 0.4282
tau11 = 0.6777
%//////////////////
kp1 = (0.37 / (b1 * L1)) + (0.02 * tau11 / (b1 *L1*L1))
ki1 = (0.03 / (b1 * (L1*L1))) + (0.0012 * tau11 / (b1*L1*L1*L1))
kd1 = (0.16 / b1) + (0.28 * tau11 / (b1 * L1))
s = tf('s')
N1 = 5
C1 = pid(kp1,ki1,kd1,N1)

Gp1 = b1/(s*(tau11*s+1))* exp(-L1*s)
Gol1 = feedback(C1*Gp1,1)

step(50*Gol1)

%% 
%//////////////////
b2 = 96.82
L2 = 0.71
tau12 = 0.96
%//////////////////

kp2 = (0.37 / (b2 * L2)) + (0.02 * tau12 / (b2 *L2*L2))
ki2 = (0.03 / (b2 * (L2*L2))) + (0.0012 * tau12 / (b2*L2*L2*L2))
kd2 = (0.16 / b2) + (0.28 * tau12 / (b2 * L2))
s = tf('s')
N2 = 5
C2 = pid(kp2,ki2,kd2,N2)

Gp2 = b2/(s*(tau12*s+1))* exp(-L2*s)
Gol2 = feedback(C2*Gp2,1)
figure
step(50*Gol2)

%% 
%//////////////////
b3 = 80
L3 = 0.5574
tau13 = 0.4328
%//////////////////

kp3 = (0.37 / (b3 * L3)) + (0.02 * tau13 / (b3 *L3*L3))
ki3 = (0.03 / (b3 * (L3*L3))) + (0.0012 * tau13 / (b3*L3*L3*L3))
kd3 = (0.16 / b3) + (0.28 * tau13 / (b3 * L3))
s = tf('s')
N3 = 5
C3 = pid(kp3,ki3,kd3,N3)

Gp3 = b3/(s*(tau13*s+1))* exp(-L3*s)
Gol3 = feedback(C3*Gp3,1)
figure
step(50*Gol3)