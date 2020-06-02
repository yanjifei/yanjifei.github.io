pkg load control signal
clear all
close all
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
s = tf('s');
P_motor = K/((J*s+b)*(L*s+R)+K^2);
P_motor.inname = 'voltage';
P_motor.outname = 'rad/s';
step(P_motor,3);
title('Open Loop Step Response');

Kp1 = 100;

Kp2 = 20;
Ki2 = 40;

Kp3 = 100;
Ki3 = 200;
Kd3 = 10;


C1 = pid(Kp1);
C1.inname = 'err';
C1.outname = 'voltage';

C2 = pid(Kp2,Ki2);
C2.inname = 'err';
C2.outname = 'voltage';

lpf = 1e3/(s+1e3);
# Use a LPF to make pid controller transfer function proper
# A pure differential operation is not implementable
C3 = pid(Kp3,Ki3,Kd3)*lpf;
C3.inname = 'err';
C3.outname = 'voltage';

Sum1 = sumblk('err = setspeed - rad/s');

loop_p = connect(P_motor,C1,Sum1,'setspeed',{'err','voltage','rad/s'});
sys_cl1 = feedback(C1*P_motor,1);
figure
#title('Step Response with Proportional Control');
step(loop_p,1);
[Y,T,X] = step(loop_p,1);


sys_cl2 = feedback(C2*P_motor,1);
loop_pi = connect(P_motor,C2,Sum1,'setspeed',{'err','voltage','rad/s'});
figure
step(loop_pi,1)
#title('Step Response with P and I Control')


sys_cl3 = feedback(C3*P_motor,1);
loop_pid = connect(P_motor,C3,Sum1,'setspeed',{'err','voltage','rad/s'});
figure
step(loop_pid,1)
#title('Step Response with PID Control')
