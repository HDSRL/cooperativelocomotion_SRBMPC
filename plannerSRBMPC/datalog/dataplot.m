close all
clear all
clc

%% command Torque plot

ctorque = load('00_desired_u.txt');

figure(1)
subplot(3,1,1)
plot(ctorque(:,1), ctorque(:,9))
hold on
plot(ctorque(:,1), ctorque(:,12))
hold on
plot(ctorque(:,1), ctorque(:,15))
hold on
plot(ctorque(:,1), ctorque(:,18))
grid on
upperlimit = refline([0 375]);
upperlimit.Color = 'r';
lowerlimit = refline([0 -375]);
lowerlimit.Color = 'r';
title('Hip Roll Torque')
xlabel('Control Tick')
ylabel('Torque (Nm)')
legend('left front','left back','right front','right back')

subplot(3,1,2)
plot(ctorque(:,1), ctorque(:,10))
hold on
plot(ctorque(:,1), ctorque(:,13))
hold on
plot(ctorque(:,1), ctorque(:,16))
hold on
plot(ctorque(:,1), ctorque(:,19))
grid on
upperlimit = refline([0 87.5]);
upperlimit.Color = 'r';
lowerlimit = refline([0 -87.5]);
lowerlimit.Color = 'r';
title('Hip Pitch Torque')
xlabel('Control Tick')
ylabel('Torque (Nm)')
legend('left front','left back','right front','right back')


subplot(3,1,3)
plot(ctorque(:,1), ctorque(:,11))
hold on
plot(ctorque(:,1), ctorque(:,14))
hold on
plot(ctorque(:,1), ctorque(:,16))
hold on
plot(ctorque(:,1), ctorque(:,19))
grid on
upperlimit = refline([0 87.5]);
upperlimit.Color = 'r';
lowerlimit = refline([0 -87.5]);
lowerlimit.Color = 'r';
title('Knee Pitch Torque')
xlabel('Control Tick')
ylabel('Torque (Nm)')
legend('left front','left back','right front','right back')

%% com trajectory plot

dcomtraj = load('03_desired_comtraj.txt');
rcomtraj = load('04_real_comtraj.txt');

figure(4)
subplot(3,1,1)
plot(dcomtraj(:,1), dcomtraj(:,3))
hold on
plot(rcomtraj(:,1), rcomtraj(:,3))
title('Center of Mass X direction')
xlabel('Control Tick')
ylabel('Distance (m)')
legend('Desired Trajectroy', 'Real Trajectroy')
grid on

subplot(3,1,2)
plot(dcomtraj(:,1), dcomtraj(:,4))
hold on
plot(rcomtraj(:,1), rcomtraj(:,4))
title('Center of Mass Y direction')
xlabel('Control Tick')
ylabel('Distance (m)')
legend('Desired Trajectroy', 'Real Trajectroy')
grid on

subplot(3,1,3)
plot(dcomtraj(:,1), dcomtraj(:,5))
hold on
plot(rcomtraj(:,1), rcomtraj(:,5))
title('Center of Mass Z direction')
xlabel('Control Tick')
ylabel('Distance (m)')
legend('Desired Trajectroy', 'Real Trajectroy')
grid on

%% leg trajectory plot

dlegtraj = load('05_desired_legtraj.txt');
rlegtraj = load('06_real_legtraj.txt');

%figure(6)
%plot3(dlegtraj(:,2), dlegtraj(:,3), dlegtraj(:,4))
%hold on
%plot3(rlegtraj(:,2), rlegtraj(:,3), rlegtraj(:,4))

%% output plot
outputstand = load('07_output_y_stand.txt');
outputtrot = load('08_output_y_trot.txt');


figure(8)
plot(outputstand(:,1), outputstand(:,3))
hold on
plot(outputstand(:,1), outputstand(:,4))
hold on
plot(outputstand(:,1), outputstand(:,5))
hold on
plot(outputstand(:,1), outputstand(:,6))
hold on
plot(outputstand(:,1), outputstand(:,7))
hold on
plot(outputstand(:,1), outputstand(:,8))

figure(9)
plot(outputtrot(:,1), outputtrot(:,3))
hold on
plot(outputtrot(:,1), outputtrot(:,4))
hold on
plot(outputtrot(:,1), outputtrot(:,5))
hold on
plot(outputtrot(:,1), outputtrot(:,6))
hold on
plot(outputtrot(:,1), outputtrot(:,7))
hold on
plot(outputtrot(:,1), outputtrot(:,8))
hold on
plot(outputtrot(:,1), outputtrot(:,9))
hold on
plot(outputtrot(:,1), outputtrot(:,10))
hold on
plot(outputtrot(:,1), outputtrot(:,11))
hold on
plot(outputtrot(:,1), outputtrot(:,12))
hold on
plot(outputtrot(:,1), outputtrot(:,13))
hold on
plot(outputtrot(:,1), outputtrot(:,14))

%% output plot
close all
clear all
clc

outputtrot = load('09_output_y_com.txt');

figure(10)
plot(outputtrot(:,1), outputtrot(:,3))
hold on
plot(outputtrot(:,1), outputtrot(:,4))
hold on
plot(outputtrot(:,1), outputtrot(:,5))
hold on
plot(outputtrot(:,1), outputtrot(:,6))
hold on
plot(outputtrot(:,1), outputtrot(:,7))
hold on
plot(outputtrot(:,1), outputtrot(:,8))
hold on
plot(outputtrot(:,1), outputtrot(:,9))
hold on
plot(outputtrot(:,1), outputtrot(:,10))
hold on
plot(outputtrot(:,1), outputtrot(:,11))
hold on
plot(outputtrot(:,1), outputtrot(:,12))
hold on
plot(outputtrot(:,1), outputtrot(:,13))
hold on
plot(outputtrot(:,1), outputtrot(:,14))


commandtorque = load('10_desired_u_for_plot.txt');

gearratio = [0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
   18.5450         0         0         0         0         0         0         0         0         0         0         0
         0   25.0000         0         0         0         0         0         0         0         0         0         0
         0         0   30.0000         0         0         0         0         0         0         0         0         0
         0         0         0   18.5450         0         0         0         0         0         0         0         0
         0         0         0         0   25.0000         0         0         0         0         0         0         0
         0         0         0         0         0   30.0000         0         0         0         0         0         0
         0         0         0         0         0         0   18.5450         0         0         0         0         0
         0         0         0         0         0         0         0   25.0000         0         0         0         0
         0         0         0         0         0         0         0         0   30.0000         0         0         0
         0         0         0         0         0         0         0         0         0   18.5450         0         0
         0         0         0         0         0         0         0         0         0         0   25.0000         0
         0         0         0         0         0         0         0         0         0         0         0   30.0000];
     

gearratio_raisim =  zeros(18,12);
gearratio_raisim(7:18, :) = eye(12);

commandtorque_lint = transpose(commandtorque(:, 3:14));

motortorque = inv(transpose(gearratio)*gearratio)*transpose(gearratio)*gearratio_raisim * commandtorque_lint;

motortorque = transpose(motortorque);

figure(11)
% plot(commandtorque(:,1), commandtorque(:,3))
% hold on
% plot(commandtorque(:,1), commandtorque(:,4))
% hold on
% plot(commandtorque(:,1), commandtorque(:,5))
% hold on
% plot(commandtorque(:,1), commandtorque(:,6))
% hold on
% plot(commandtorque(:,1), commandtorque(:,7))
% hold on
% plot(commandtorque(:,1), commandtorque(:,8))
% hold on
% plot(commandtorque(:,1), commandtorque(:,9))
% hold on
% plot(commandtorque(:,1), commandtorque(:,10))
% hold on
% plot(commandtorque(:,1), commandtorque(:,11))
% hold on
% plot(commandtorque(:,1), commandtorque(:,12))
% hold on
% plot(commandtorque(:,1), commandtorque(:,13))
% hold on
% plot(commandtorque(:,1), commandtorque(:,14))


plot(commandtorque(:,1), motortorque(:,1))
hold on
plot(commandtorque(:,1), motortorque(:,2))
hold on
plot(commandtorque(:,1), motortorque(:,3))
hold on
plot(commandtorque(:,1), motortorque(:,4))
hold on
plot(commandtorque(:,1), motortorque(:,5))
hold on
plot(commandtorque(:,1), motortorque(:,6))
hold on
plot(commandtorque(:,1), motortorque(:,7))
hold on
plot(commandtorque(:,1), motortorque(:,8))
hold on
plot(commandtorque(:,1), motortorque(:,9))
hold on
plot(commandtorque(:,1), motortorque(:,10))
hold on
plot(commandtorque(:,1), motortorque(:,11))
hold on
plot(commandtorque(:,1), motortorque(:,12))

