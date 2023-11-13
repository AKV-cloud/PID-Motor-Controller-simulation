%%%----------Position----------
%%-----------Uncompensated system-----------

%Defining values of constants
J = 3.2284E-6;      %moment of inertia of the rotor
b = 3.5077E-6;      %motor viscous friction constant
K = 0.0274;         %electromotive force constant=motor torque constant
R = 4;              %electric resistance 
L = 2.75E-6;        %electric inductance 

s = tf('s');
P_motor = K/(s*((J*s+b)*(L*s+R)+K^2));

A = [0 1 0
    0 -b/J K/J
    0 -K/L -R/L];
B = [0 ; 0 ; 1/L];
C = [1 0 0];
D = [0];

[num,den] = ss2tf(A,B,C,D);
Transfer_Function = tf(num,den);
Transfer_Function;

t = 0:0.001:0.2;

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
step(Transfer_Function,t);
ylabel('Speed (rad/s)');
title('Step Response of uncompensated system');

%%-----------P-controller-----------

Kp=21;
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

%%-----------PI-controller-----------

Ki = 500;
C = pid(Kp,Ki);
PI=feedback(Transfer_Function*C,1);
figure;
pzplot(PI);
title('Pole-zero plot of system with PI-controller');
figure;
rlocus(PI); 
title('Root locus of system with PI-controller');
figure;
bode(PI);
grid on;
title('Bode plot of system with PI-controller');
figure;
margin(PI);
figure;
step(PI);
ylabel('Speed (rad/s)');
title('Step response of system with PI-controller');

%%-----------PID-controller-----------
Kd = 0.25;
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
