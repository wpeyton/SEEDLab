%% Motor Control Mini Project Documentation
% Group 10
%
% Trevor Bachand, Andrew Burton, William Peyton, Kyra Squier
%  
%% Open Loop Step Response
% Graphs the experimental step response and the estimated transfer function
% step response.
%

%experimental
[V,T,VT] = xlsread('StepResponse.xlsx');
t = V(:,5);
y = V(:,4);

plot(t,y)

%transfer function
K = 0.0539;
sigma = 10.75;
sys = tf(K*sigma, [1  sigma]);

hold on
step(sys)
xlim([0 0.8])
xlabel('Time')
ylabel('Velocity')
title('Open Loop Step Response')
hold off

open_system('openLoop')

% The step response of the estimated transfer function traces the
% general shape of the experimental data so should be accurate
% when tuning the PI controller.
%% Closed Loop Step Response
% Graphs the experimental step response and the estimated transfer function
% with a PI controller step response.
%  

%experimental
[W,R,WR] = xlsread('StepResponse.xlsx');
x = (W(:,21) + 1);
v = W(:,18);

plot(x,v)
xlabel('Time (s)')
ylabel('Position (rad)')
title('Closed Loop Step Response')

%transfer function
out = sim('PIcontrol');
hold on
plot(out.Position)
hold off

open_system('PIcontrol')

% The experimental step response is much quicker than the simulated
% response, which causes additional overshoot and a quicker rise time.
% This is most likely due to the voltage of the battery being lower
% when we collected data on the open loop step response. The steady
% state error is not affected and is approximately 0.
%% Controller Design
% Once the transfer function was estimated using the open loop step
% response, the closed loop system was created in Simulink. The
% proportional and intergral gain were tuned using the built-in tuning
% function. The design in Simulink has an overshoot of 10.4% and a rise
% time of 0.981 seconds, meeting the design specifications.