clear
clc
close all

run("../parameters.m");

s = tf('s');
G = load('tf').sys

%%%%%%%%%%%%%%%%%%%%%%%%
SHOW_FIGURES = true;
SAVE_GAINS = false;
%%%%%%%%%%%%%%%%%%%%%%%%


%% PID

phase_margin = 50;
wc = 8;
opts = pidtuneOptions('PhaseMargin', phase_margin); 
[PID, info] = pidtune(G, 'PI', opts);
info


%% Lead

[~,~,~,G_wc] = margin(G);

% Lead compensator
alpha_heading = 0.85; 
taud_heading = 1/(alpha_heading*G_wc);

Lead = (taud_heading*s + 1)/(taud_heading*alpha_heading*s + 1);


%% Evaluate PID

% Ts = 0.004;
% PIDz = c2d(PID, Ts, 'tunstin');

P = PID*G;

if SHOW_FIGURES
    %subplot(1,2,1);
    figure();
    margin(P);

    %subplot(1,2,2);
    % step(feedback(P,Lead), 5);
    figure();
    step(feedback(P, 1), 5);
    grid on;

    %return
end

Kp_heading = PID.Kp;
Ki_heading = PID.Ki;


%% Evaluate Lead + PID

heading_ref = 0.5;

out = sim('../final_robot_model.slx', 5);

figure;
plot(out.heading_out)

if SAVE_GAINS
    save("../heading_pi_gains", 'Kp_heading', 'Ki_heading', 'alpha_heading', 'taud_heading');
end









