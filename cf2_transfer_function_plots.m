%%
clear all
close all
clc

%% Settings

save_figures = false;

%% Load model

cf_nom_param = crazyflie2_nominal_params();
g = 9.8; % gravity
m = cf_nom_param.mass;
I = cf_nom_param.inertia_vect;
d = cf_nom_param.center_to_rotor_dist;
cT = cf_nom_param.cT;
cQ = cf_nom_param.cQ;

cf_sys = quadrotor_lin_model(g, m, I, d, cT, cQ);


%% LQR Control

Q = eye(12);
R = eye(4);
N = zeros(12,4);

[K,S,e] = lqr(cf_sys.sys, Q, R, N);


%% Transfer functions
% Open-loop 
G = cf_sys.sys;

% Closed-loop 
T = feedback(G*K, eye(12));


%% Transfer function impulse responses

% Open-loop impulse response
disp('Plotting 1/11')
figure('Position', [100 100 800 1100]);
impulse(G);
if(save_figures)
    printer('open_loop_impulse')
end

% Closed-loop impulse response
disp('Plotting 2/11')
figure('Position', [100 100 800 1100]);
impulse(T(1:6,1:6));
title('Impulse Response from Inputs 1-6 to Outputs 1-6')
if(save_figures)
    printer('closed_loop_impulse_1')
end

disp('Plotting 3/11')
figure('Position', [100 100 800 1100]);
impulse(T(1:6,7:12));
title('Impulse Response from Inputs 7-12 to Outputs 1-6')
if(save_figures)
    printer('closed_loop_impulse_2')
end

disp('Plotting 4/11')
figure('Position', [100 100 800 1100]);
impulse(T(7:12,1:6));
title('Impulse Response from Inputs 1-6 to Outputs 7-12')
if(save_figures)
    printer('closed_loop_impulse_3')
end

disp('Plotting 5/11')
figure('Position', [100 100 800 1100]);
impulse(T(7:12,7:12));
title('Impulse Response from Inputs 7-12 to Outputs 7-12')
if(save_figures)
    printer('closed_loop_impulse_4')
end


%% Transfer function Bode plots

% Open-loop Bode plots
disp('Plotting 6/11')
figure('Position', [100 100 800 1100]);
bode(G(1:6,:));
title('Bode Plot from Inputs 1-4 to Outputs 1-6')
if(save_figures)
    printer('open_loop_bode_1')
end

disp('Plotting 7/11')
figure('Position', [100 100 800 1100]);
bode(G(7:12,:));
title('Bode Plot from Inputs 1-4 to Outputs 7-12')
if(save_figures)
    printer('open_loop_bode_2')
end

% Closed-loop Bode plots
disp('Plotting 8/11')
figure('Position', [100 100 800 1100]);
bode(T(1:6,1:6));
title('Bode Plot from Inputs 1-6 to Outputs 1-6')
if(save_figures)
    printer('closed_loop_bode_1')
end

disp('Plotting 9/11')
figure('Position', [100 100 800 1100]);
bode(T(1:6,7:12));
title('Bode Plot from Inputs 7-12 to Outputs 1-6')
if(save_figures)
    printer('closed_loop_bode_2')
end

disp('Plotting 10/11')
figure('Position', [100 100 800 1100]);
bode(T(7:12,1:6));
title('Bode Plot from Inputs 1-6 to Outputs 7-12')
if(save_figures)
    printer('closed_loop_bode_3')
end

disp('Plotting 11/11')
figure('Position', [100 100 800 1100]);
bode(T(7:12,7:12));
title('Bode Plot from Inputs 7-12 to Outputs 7-12')
if(save_figures)
    printer('closed_loop_bode_4')
end

%%

function printer(name)
print(['images' filesep name],'-dpng','-r600')
end
