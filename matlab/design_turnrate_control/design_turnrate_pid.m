clear
clc
close all

run("../parameters.m");

s = tf('s');
G = load('tf').sys32;

%%%%%%%%%%%%%%%%%%%%%%%%
SHOW_FIGURES = true;
SAVE_GAINS = false;
%%%%%%%%%%%%%%%%%%%%%%%%


%% PID

phase_margin = 60;
wc = 25;
opts = pidtuneOptions('PhaseMargin', phase_margin); 
[PID, info] = pidtune(G, 'PI', wc, opts);
info


%% Lead

% [~,~,~,G_wc] = margin(G);
% 
% % Lead compensator
% alpha = 0.35; 
% taud = 1/(alpha*G_wc);
% 
% Lead = (taud*s + 1)/(taud*alpha*s + 1);


%% Evaluate PID

% Ts = 0.004;
% PIDz = c2d(PID, Ts, 'tunstin');

P = PID*G;

if SHOW_FIGURES
    subplot(1,2,1);
    margin(P);

    subplot(1,2,2);
    step(feedback(P,1), 5);
    grid on;

    return
end

Kp_turnrate = PID.Kp;
Ki_turnrate = PID.Ki;


%% Evaluate Lead + PID

% Deactivate heading controller for testing only the turnrate controller
Kp_heading = 1;
Ki_heading = 0;
K_lead_heading = 0;

out = sim('../final_robot_model.slx', 5);
plot(out.turnrate_out);

if SAVE_GAINS
    save("../turnrate_pi_gains", 'Kp_turnrate', 'Ki_turnrate', 'alpha', 'taud');
end









