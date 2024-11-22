%% output plot
close all
clear all
clc

plotuntil = 7000;

outputtrot = load('09_output_y_com.txt');

figure(10)
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,3))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,4))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,5))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,6))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,7))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,8))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,9))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,10))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,11))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,12))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,13))
hold on
plot(outputtrot(1:plotuntil,1), outputtrot(1:plotuntil,14))
title('output y')
xlabel('Time(sec)')
ylabel('y')


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
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,3))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,4))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,5))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,6))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,7))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,8))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,9))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,10))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,11))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,12))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,13))
hold on
plot(commandtorque(1:plotuntil,1), commandtorque(1:plotuntil,14))
title('load side torque')
ylabel('time(sec)')
ylabel('torque(Nm)')

figure(12)
% 3,6,9,12 are saturated ( every knee pitch )
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,1))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,2))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,3))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,4))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,5))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,6))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,7))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,8))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,9))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,10))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,11))
hold on
plot(commandtorque(1:plotuntil,1), motortorque(1:plotuntil,12))
title('motor side torque')
ylabel('time(sec)')
ylabel('torque(Nm)')


