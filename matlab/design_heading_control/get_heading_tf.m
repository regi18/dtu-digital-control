clear
clc
close all

run("../parameters");

turnrate_ref_start_time = 0.10;
turnrate_ref = 1;

SHOW_FIGURES = true;


%%
% N.B. This doesn't have the heading controller
out = sim('model_no_heading_control.slx', 3);

if SHOW_FIGURES
    figure();
    hold off
    plot(out.heading_tf.Time(:), out.heading_tf.Data(:,1),'LineWidth',2)
    hold on
    plot(out.heading_tf.Time(:), out.heading_tf.Data(:,2),'LineWidth',2)
    legend('Turnrate','Theta', 'Location','east')
    grid on
    xlabel('Time (sec)')
    ylabel('input and response')
    title('Robot heading. NB! no measurement delay')
end


%%
% Resample with constant sample time
timeVec = turnrate_ref_start_time:Ts:2.5;
tsout = resample(out.heading_tf, timeVec);

% Get in and out in separate vectors
idin = tsout.Data(:,1);
idout = tsout.Data(:,2);

% figure; hold on; legend
% plot(idin, '--', 'DisplayName', 'in');
% plot(idout, 'DisplayName', 'out');
% return

% Collect in structure
idd = iddata(idout, idin, Ts);


%% Estimate 


%close all
%sys = tfest(idd, 2, 1);
sys = tfest(idd, 4, 3);
figure();
compare(sys, idd);


%%

sys.InputDelay = transport_delay;

if SHOW_FIGURES
    figure();
    bode(sys)
    axis([0.3 500 -200 5])
    title('Turnrate transfer function, delay 10ms')
    grid on
end

%%
save('tf', 'sys');












