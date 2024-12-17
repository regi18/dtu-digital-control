clc
clear
close all

run("../parameters");
turnrate_ref_start_time = 0.5;
turnrate_ref = 1;


%%
% N.B. This doesn't have the turnrate controller (nor the acceleration controller)
out = sim('model_no_turnrate_control.slx', 3);

figure();
hold off
plot(out.turn_tf.Time(:), out.turn_tf.Data(:,1),'LineWidth',2)
hold on
plot(out.turn_tf.Time(:), out.turn_tf.Data(:,2),'LineWidth',2)
legend('Turnrate','Actuator signal','Location','east')
grid on
xlabel('Time (sec)')
ylabel('input and response')
title('Robot turn. NB! no measurement delay')


%%
% Resample with constant sample time
timeVec = 0.5:Ts:1.5;
tsout = resample(out.turn_tf, timeVec);

% Get in and out in separate vectors
idin = tsout.Data(:,1);
idout = tsout.Data(:,2);

% Collect in structure
idd = iddata(idout, idin, Ts);

% Estimate
sys32 = tfest(idd, 3, 2);
figure();
compare(sys32, idd);


%%
sys32.InputDelay = transport_delay;
figure();
bode(sys32)
axis([0.3 500 -200 5])
title('Turnrate transfer function, delay 10ms')
grid on


%%
save('tf', 'sys32');












