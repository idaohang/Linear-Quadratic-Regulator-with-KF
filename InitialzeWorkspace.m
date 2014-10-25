%% Kalman-Filter, continuous-Time example
% This implementation on the classical continous time Kalman filter

% Simple CKF testing script
% Note this equivalency is Hendricks and Burl
% Bu = B
% Sw = V2;
% Sv = V1;
% Odd notation in hendricks: Bw = Bv in Burl (would make sense to use Bv)
% Kalman gain notation: Hendricks L, Burl: G
% Se -- Burl, Q-- Hendricks
% Initial value at integrators
% SS model: X_init (Initial value for X, n elements long)
% Q: Dimension nxn = usually 0

% First order low pass filter equation test
% clc;
% clear all;
% A = -1;
% Bu = 1;
% Bw = 1;
% C = 1;
% Sv = 0.001;
% Sw = 0.0002;
% X_init = 0;
% Se_init = 0;
% 
% 
% A= [0 1 0; 0 0 1; 0 0 -0.5]
% Bu= [0; 0; 0.5]
% Bw = [1;1;1];
% C= [1 0 0]
% D= 0
% Sv = 0.0005;
% Sw = 0.002^2;
% X_init = [0 0 0];
% Se_init = zeros(3,3);
% % Originial 
% Q= [1 0 0 ; 0 0 0; 0 0 0] 
% R= 2
% [K_LQR,P,E]= lqr(A,Bu,Q,R);

% Inverted pendulum example
% Example 1
clc;
clear all;
A = [0 0 1 0;
    0 0 0 1;
    0 1.5216 -11.6513 -0.0049;
    0 26.1093 -26.8458 -0.0831];

Bu = [0;
    0;
    1.5304;
    3.5261];

C  =[1 0 0 0];

Bw = [0.5;
    0.5;
    0.5;
    0.5];

D = 0;

Sv = 0.005;
Sw = 0.002^2;

X_init = [0 0 0 0];
Se_init = zeros(4,4);

Q = [0.75 0 0 0;
    0 4 0 0;
    0 0 0 0;
    0 0 0 0];

R = 3e-4;

[K_LQR,P,E]= lqr(A,Bu,Q,R);
% clc;
% clear all;
% M = 0.5;
% m = 0.2;
% b = 0.1;
% I = 0.006;
% g = 9.8;
% l = 0.3;
% p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices
% 
% 
% A = [0      1              0           0;
%      0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
%      0      0              0           1;
%      0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
% Bu = [     0;
%      (I+m*l^2)/p;
%           0;
%         m*l/p];
% 
% %Number of columns in Bu will equal the number of inputs
% R_scale = 0.001;
% [rows_bu,number_inputs] = size(Bu);
% R_temp = eye(number_inputs);
% R = R_scale * R_temp;
% 
% Bw = [0.1;
%      0;
%      0;
%      0];
%     
% C = [1 0 0 0;
%      0 0 1 0];
% D = [0;
%      0];
%  
% Q = C'*C;
% 
% Sw = 1e-3;
% Sv = 1e-2;
% 
% X_init = [0 0 0 0];
% Se_init = zeros(4,4);

