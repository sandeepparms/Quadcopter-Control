function [ u ] = pd_controller(~,s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% FILL IN YOUR CODE HERE
k_p = 100; %proportional gain
k_v = 12; % derivative gain
m = params.mass;
e = s_des-s;

u = m*(k_p*e(1)+ k_v*e(2) + params.gravity);

end

