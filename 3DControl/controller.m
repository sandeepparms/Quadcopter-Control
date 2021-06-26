function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

m = params.mass;
g = params.gravity;
k_p = [15;15;15];
k_d = [20;50;10];

% k_p = [100;70;150];
% k_d = [500;500;500];

k_pphi = 40; k_dphi = 20;
k_ptheta =60; k_dtheta = 20;
k_ppsi = 20; k_dpsi = 20;

% k_pphi = 50; k_dphi = 500;
% k_ptheta =50; k_dtheta = 650;
% k_ppsi = 50; k_dpsi = 500;
% =================== Your code goes here ===================

% Thrust
r1_des_ddot = des_state.acc(1)+k_d(1)*(des_state.vel(1)-state.vel(1))+...
    k_p(1)*(des_state.pos(1)-state.pos(1));
r2_des_ddot = des_state.acc(2)+k_d(2)*(des_state.vel(2)-state.vel(2))+...
    k_p(2)*(des_state.pos(2)-state.pos(2));
r3_des_ddot = des_state.acc(3)+k_d(3)*(des_state.vel(3)-state.vel(3))+...
    k_p(3)*(des_state.pos(3)-state.pos(3));

F = m*g+m*r3_des_ddot;
phi_des = (1/g)*(r1_des_ddot*sin(des_state.yaw) - r2_des_ddot*cos(des_state.yaw));
theta_des = (1/g)*(r1_des_ddot*cos(des_state.yaw) + r2_des_ddot*sin(des_state.yaw));
% Moment
%M = zeros(3,1);
u_1x = k_pphi*(phi_des - state.rot(1)) + k_dphi*(-state.omega(1));
u_1y = k_ptheta*(theta_des - state.rot(2)) + k_dtheta*(-state.omega(2));
u_1z = k_ppsi*(des_state.yaw - state.rot(3)) + k_dpsi*(des_state.yawdot - state.omega(3));

M = [u_1x;u_1y;u_1z];
% =================== Your code ends here ===================

end
