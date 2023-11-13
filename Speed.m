%%%----------Speed----------
%%-----------Uncompensated system-----------

%Defining values of constants
J = 0.01;   %moment of inertia of the rotor     0.01 kg.m^2
b = 0.1;    %motor viscous friction constant    0.1 N.m.s
Ke = 0.01;  %electromotive force constant       0.01 V/rad/sec
Kt = 0.01;  %motor torque constant              0.01 N.m/Amp
K = 0.01;   %Ke=Kt=K
R = 1;      %electric resistance                1 Ohm
L = 0.5;    %electric inductance                0.5 H

s = tf('s');
Tf_motor = (K)/((J*s+b)*(L*s+R)+K^2);

A = [-b/J   K/J
    -K/L   -R/L];
B = [0
    1/L];
C = [1   0];
D = 0;

[num,den] = ss2tf(A,B,C,D);
Transfer_Function = tf(num,den);
Transfer_Function;
t = 0:0.01:5;
figure;
pzplot(Transfer_Function);
title('Pole-zero plot of uncompensated system');
figure;
rlocus(Transfer_Function); 
title('Root locus of uncompensated system');
figure;
bode(Transfer_Function);
title('Bode plot of uncompensated system');
grid on;
figure;
margin(Transfer_Function);
figure;
step(Transfer_Function);
ylabel('Speed (rad/s)');
title('Step Response of uncompensated system');
isstable(Transfer_Function)


%%-----------P-controller-----------

Kp=100;
C = pid(Kp);
P=feedback(Transfer_Function*C,1);
figure;
pzplot(P);
title('Pole-zero plot of system with P-controller');
figure;
rlocus(P); 
title('Root locus of system with P-controller');
figure;
bode(P);
grid on;
title('Bode plot of system with P-controller');
figure;
margin(P);
figure;
step(P);
ylabel('Speed (rad/s)');
title('Step response of system with P-controller');

%%-----------PD-controller-----------

Kd = 10;
C = pid(Kp,0,Kd);
PD=feedback(Transfer_Function*C,1);
figure;
pzplot(PD);
title('Pole-zero plot of system with PD-controller');
figure;
rlocus(PD); 
title('Root locus of system with PD-controller');
figure;
bode(PD);
grid on;
title('Bode plot of system with PD-controller');
figure;
margin(PD);
figure;
step(PD);
ylabel('Speed (rad/s)');
title('Step response of system with PD-controller');

%%-----------PID-controller-----------
Ki = 200;
C = pid(Kp,Ki,Kd);
PID=feedback(Transfer_Function*C,1);
figure;
pzplot(PID);
title('Pole-zero plot of system with PID-controller');
figure;
rlocus(PID); 
title('Root locus of system with PID-controller');
figure;
bode(PID);
grid on;
title('Bode plot of system with PID-controller');
figure;
margin(PID);
figure;
step(PID);
ylabel('Speed (rad/s)');
title('Step response of system with PID-controller');