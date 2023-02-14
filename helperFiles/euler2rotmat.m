clear all
close all
clc

syms phi theta psi

Rz = [ cos(psi), -sin(psi), 0; 
       sin(psi),  cos(psi), 0;
              0,         0, 1];
Rx = [1,        0,         0;
      0, cos(phi), -sin(phi);
      0, sin(phi),  cos(phi)];
Ry = [ cos(theta), 0, sin(theta);
                0, 1, 0;
      -sin(theta), 0, cos(theta)];


% R = Rz*Rx*Ry
% R = Ry*Rx*Rz

R = Rz*Ry*Rx