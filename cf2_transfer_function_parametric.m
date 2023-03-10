clear
clc

syms g cT cQ d m Ix Iy Iz real
syms s
K = sym('K', [4, 12], 'real');


A = [ ...
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
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]

Gamma = [
      cT ,   cT ,   cT ,   cT ; ...
    -d*cT,   0  ,  d*cT,   0  ; ...
      0  , -d*cT,    0 ,  d*cT; ...
     -cQ ,   cQ ,   -cQ,   cQ ];


B = [ ...
    0,   0,   0,   0,   0,   0,   0,   0,  1/m,   0 ,   0 ,     0 ; ...
    0,   0,   0,   0,   0,   0,   0,   0,   0 , 1/Ix,   0 ,     0 ; ...
    0,   0,   0,   0,   0,   0,   0,   0,   0 ,   0 , 1/Iy,     0 ; ...
    0,   0,   0,   0,   0,   0,   0,   0,   0 ,   0 ,   0 ,   1/Iz]' ...
    * Gamma

G = inv(s*eye(12)-A)*B

% G*K*inv(eye(12)+G*K)