% Simulate CF2.0 Quadrotor 
%
% hover simulator
%

%%
% 
% clear all
% close all
% clc
% 

%% User Settings

% hover point x,y,z
px = 1;
py = 2;
pz = 5;

t_run = 15; % experiment run time

video_name = 'exp1';

make_video = true; % create video or don't


%% Load model

cf_nom_param = crazyflie2_nominal_params();
% m = cf_nom_param.mass;
% I = cf_nom_param.inertia_vect;
% d = cf_nom_param.center_to_rotor_dist;
% cT = cf_nom_param.cT;
% cQ = cf_nom_param.cQ;

cf = quadrotor_lin_model(cf_nom_param);


%% Desired configuration

x_bar = [px, py, pz, 0, 0, 0, 0, 0, 0, 0, 0, 0]'; % desired states
% u_bar = cf.Gamma\[m*g;0;0;0]; % desired input
x_0 = cf.IC; % initial states
% u_0 = [0;0;0;0]; % initial inputs

%% Analysis

Co = ctrb(cf.sys); % controllability matrix
rCo = rank(Co);
if rCo == 12
    disp("Controllable")
else
    disp("Not controllable")
end
Ob = obsv(cf.sys); % controllability matrix
rOb = rank(Ob);
if rOb == 12
    disp("Observable")
else
    disp("Not observable")
end
%% LQR Control

Q = eye(12);
R = eye(4);
N = zeros(12,4);

[K,S,e] = lqr(cf.sys, Q, R, N);

%% System

% feedback control: u = K*(x_bar-x)
% \dot{x} = Ax + Bu = (A-B*K)x + B*K*x_bar = A_K*X + B*K*x_bar
A_K = cf.A - cf.B*K;

%% ODE45 Simulation

disp("Simulating ...")
t = [0, t_run];
[t,x] = ode45( @(t,x) ...
                A_K*x + cf.B*K*x_bar , ...
                t, x_0);
disp("Simulation complete")

%% Video

if make_video 
    disp("Creating video ...")
    hframe = figure(2);
    xmin = min(x(:,1));
    xmax = max(x(:,1));
    ymin = min(x(:,2));
    ymax = max(x(:,2));
    zmin = min(x(:,3));
    zmax = max(x(:,3));

    xmin = xmin-1;
    xmax = xmax+1;
    ymin = ymin-1;
    ymax = ymax+1;
    zmin = zmin-1;
    zmax = zmax+1;

    axis ([xmin xmax ymin ymax zmin zmax]);
    grid on

    resolution = 1; 
    Name = 'Hover';
    frameRate = length(t)/(t_run*resolution);   % Video frame rate
    vidQuality = 100; % A value between 0 to 100
    currentFolder = pwd;
    address =  fullfile(currentFolder,'storedData/');
    fileType = '.avi';
    fullAddress = strcat(address,video_name,fileType);
    vid = VideoWriter(fullAddress);
    vid.Quality = vidQuality;
    vid.FrameRate = frameRate;
    open(vid);
    axis ([xmin xmax ymin ymax zmin zmax]);    
    xlabel('x')
    ylabel('y')
    zlabel('z')
    scatter3(px, py, pz,'*');   
    hold on;
    tic
    for i = 1:length(t)/resolution
        scatter3(px, py, pz,'*');   
        hold on;
        scatter3( x(i*resolution,1), x(i*resolution,2), x(i*resolution,3));
        axis ([xmin xmax ymin ymax zmin zmax]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        drawnow;
        writeVideo(vid, getframe(hframe));
        cla;
    end
    close(vid);
    toc
    disp("Video Done")
end

%% Plot

disp("Plotting ...")

lw = 2;

grid on

figure(1);
subplot(3,2,1);
plot(t, x(:,1), 'LineWidth', lw);
title('x-position vs. time');
xlabel('Time');
ylabel('solution x1 = x');
grid on

subplot(3,2,3);
plot(t, x(:,2), 'LineWidth', lw);
title('y-position vs. time');
xlabel('Time');
ylabel('solution x2 = y');
grid on

subplot(3,2,5);
plot(t, x(:,3), 'LineWidth', lw);
title('z-position vs. time');
xlabel('Time');
ylabel('solution x3 = z');
grid on

subplot(3,2,2);
plot(t, x(:,4), 'LineWidth', lw);
title('\phi-angle vs. time');
xlabel('Time');
ylabel('solution x4 = \phi');
grid on

subplot(3,2,4);
plot(t, x(:,5), 'LineWidth', lw);
title('\theta-angle vs. time');
xlabel('Time');
ylabel('solution x5 = \theta');
grid on

subplot(3,2,6);
plot(t, x(:,6), 'LineWidth', lw);
title('\psi-angle vs. time');
xlabel('Time');
ylabel('solution x6 = \psi');
grid on

disp("Plotting Done")
