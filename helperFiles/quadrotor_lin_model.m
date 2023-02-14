function [quadrotor] = quadrotor_lin_model(varargin)
%QUADROTOR_LIN_MODEL Loads the quadrotor linear model about hover
%     Input:
%  cf_param: struct with fields mass, inertia_mat, center_to_rotor_dist, cT, cQ, g
%            OR
%         g: gravity
%         m: mass of quadrotor
%         I: inertia matrix (3x3) assumed diagonal
%         d: distance between center of quadrotor and rotors (assumed the same for all)
%        cT: thrust coefficient  
%        cQ: rotor drag coefficient
%    Output:
% quadrotor: struct with quadrotor dynamics state: pos, ang, vel, angvel

if nargin == 1
    cf_param = varargin{1};
    m = cf_param.mass;
    I = cf_param.inertia_vect;
    d = cf_param.center_to_rotor_dist;
    cT = cf_param.cT;
    cQ = cf_param.cQ;
    g = cf_param.g;
elseif nargin == 6
    [g, m, I, d, cT, cQ] = varargin{:};
else
    error("Invalid number of input arguments")
end


Ix = I(1);
Iy = I(2);
Iz = I(3);

quadrotor.A = [ ...
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 ; ...
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 ; ...
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 ; ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 ; ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 ; ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 ; ...
    0, 0, 0, 0,-g, 0, 0, 0, 0, 0, 0, 0 ; ...
    0, 0, 0, g, 0, 0, 0, 0, 0, 0, 0, 0 ; ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ];

quadrotor.Gamma = [
      cT ,   cT ,   cT ,   cT ; ...
    -d*cT,   0  ,  d*cT,   0  ; ...
      0  , -d*cT,    0 ,  d*cT; ...
     -cQ ,   cQ ,   -cQ,   cQ ];


quadrotor.B = [ ...
    0,   0,   0,   0,   0,   0,   0,   0,  1/m,   0 ,   0 ,     0 ; ...
    0,   0,   0,   0,   0,   0,   0,   0,   0 , 1/Ix,   0 ,     0 ; ...
    0,   0,   0,   0,   0,   0,   0,   0,   0 ,   0 , 1/Iy,     0 ; ...
    0,   0,   0,   0,   0,   0,   0,   0,   0 ,   0 ,   0 ,   1/Iz]' ...
    * quadrotor.Gamma;



quadrotor.C = eye(12);
quadrotor.D = zeros(12,4);

quadrotor.sys = ss(quadrotor.A,quadrotor.B,quadrotor.C,quadrotor.D);


% initial condition
quadrotor.IC = [0,0,0,0,0,0,0,0,0,0,0,0]'; 

end

