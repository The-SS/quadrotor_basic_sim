function [cf_params] = crazyflie2_nominal_params()
%CRAZYFLIE2_NOMINAL_PARAMS Loads the nominal parameters for the Crazyflie 2.0
%   Output:
%       cf_params: struct with nominal parameters

% mostly from: http://mikehamer.info/assets/papers/Crazyflie%20Modelling.pdf
% cT, cQ from: https://groups.csail.mit.edu/robotics-center/public_papers/Landry15.pdf

cf_params.mass = 28*10^-3; % kg
cf_params.inertia_mat = [ ...
    16.571710, 0.830806, 0.718277; ...
    0.830806, 16.655602, 1.800197; ...
    0.718277, 1.800197, 29.261652]*10^-6; % kg.m^2
cf_params.inertia_vect = [cf_params.inertia_mat(1,1), ...
                          cf_params.inertia_mat(2,2), ... 
                          cf_params.inertia_mat(3,3)]; % only diagonal elemtns
cf_params.center_to_rotor_dist = 0.092/2; % m
cf_params.cT = 0.005022; % N.s^2 = kg.m
cf_params.cQ = 1.8580*10^-5; % N.m.s^2 = kg.m^2
cf_params.g = 9.8; % gravity
end

