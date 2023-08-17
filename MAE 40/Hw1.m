% script RR_Ex10_02_passive_filters
% Solves the basic equations of four simple passive filters.
% These examples are easy to solve by hand, but illustrate how
% to put a few linear equations into A*x=b form and solve using Matlab.
% Renaissance Robotics codebase, Chapter 10, https://github.com/tbewley/RR
% Copyright 2023 by Thomas Bewley, distributed under Modified BSD License.

clear, close all, syms s Vi Vs C L R R1 R2 Cd
%% Question 1&2
% Second-order low-pass LC filter: Solve for Vo as a function of Vi
% x={I_L,Ic,I_R, Vo}  <-- unknown vector
A  =[ 1  -1 -1 0;    % I_L - Ic - I_R = 0
     L*s 0 0 1;    % L*s*I_L + Vo = Vi
      0 1 0 -C*s;   % -C*s*Vo + Ic = 0
      0 0 R -1];  % I_R*R - Vo = 0
b  =[ 0; Vi; 0; 0];
x=A\b; Vo_LPF2_undamped=simplify(x(3))
omega4=10; F_LPF2_undamped=RR_tf([omega4^2],[1 0 omega4^2]);
figure(3), RR_bode(F_LPF2_undamped)
zeta=[0.1 0.7 1];
for n=1:3
    Vo_V1= RR_tf([omega4^2], [1 2*zeta(n)*omega4 omega4^2])
    figure(n+3), RR_bode(Vo_V1)
end

% omega4=1/sqrt(R*L*C), as the value of the damping variable zeta increases
% the amount of unwanted noise aroung the critical frequency diminishes.

%% Question 3&4
% Second-order low-pass LC filter: Solve for Vo as a function of Vi
% x={I_L,Ic,I_R, Vo, V2}  <-- unknown vector
A  =[ 1  -1 -1 0 0;% I_L - Ic - I_R = 0
     L*s 0 0 1 0;    % L*s*I_L + Vo = Vi
      0 1 0 -C*s 0;   % -C*s*Vo + Ic = 0
      0 0 R -1 1;     % I_R*R + V2 - Vo = 0
      0 0 -1/(Cd*s^2) 0 1];  % V2-I_R/(C*s^2) = 0
b  =[ 0; Vi; 0; 0; 0];
x=A\b; Vo_LPF2_undamped=simplify(x(3))
