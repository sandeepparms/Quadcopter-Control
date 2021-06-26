function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.

%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else
        scale = t/d0(t_index-1);
        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%
%% Fill in your code here

%%
% syms p(t) a0 a1 a2 a3 a4 a5 a6 a7;
% p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7;
% p_d = 7*a7*t^6 + 6*a6*t^5 + 5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1;
% p_dd = 42*a7*t^5 + 30*a6*t^4 + 20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2;
% p_ddd = 210*a7*t^4 + 120*a6*t^3 + 60*a5*t^2 + 24*a4*t + 6*a3;
% p_4d = 840*a7*t^3 + 360*a6*t^2 + 120*a5*t + 24*a4;
% p_5d = 2520*a7*t^2 + 720*a6*t + 120*a5;
% p_6d = 720*a6 + 5040*a7*t;
%
% persistent traj_time d0 waypoints0
% if nargin>2
%     desired_state.pos = zeros(3,1);
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
%     [x,y,z] = get_coeff(waypoints);
% %     coffy = y;
% %     coffz = z;
%     dist = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(dist(1,:).^2 + dist(2,:).^2 + dist(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     %traj_time = [0, d0];
%     waypoints0 = waypoints;
% else
%     [x,y,z] = get_coeff(waypoints0);
%     if (t>traj_time(end))
%         t = traj_time(end) - 0.05;
%     end
%     t_indx = find(traj_time >= t,1)-1;
%     if(t_indx > 1)
%         t = t - traj_time(t_indx);
%     end
%     %t_indx = max(t_indx,1);
%     if t_indx == 0
%         desired_state.pos = waypoints0(:,1);
%         desired_state.vel = zeros(3,1);
%         desired_state.acc = zeros(3,1);
%         desired_state.yaw = 0;
%         desired_state.yawdot = 0;
%     elseif t_indx == 4
%         desired_state.pos = waypoints0(:,size(waypoints0,2));
%         desired_state.vel = zeros(3,1);
%         desired_state.acc = zeros(3,1);
%         desired_state.yaw = 0;
%         desired_state.yawdot = 0;
%     else
%         scale = t/d0(t_indx);
%         desired_state.pos(1) = x((8*(t_indx-1))+1) + x((8*(t_indx-1))+2)*scale + x((8*(t_indx-1))+3)*scale^2 ...
%         + x((8*(t_indx-1))+4)*scale^3 + x((8*(t_indx-1))+5)*scale^4 + x((8*(t_indx-1))+6)*scale^5 ...
%         + x((8*(t_indx-1))+7)*scale^6 + x((8*(t_indx-1))+8)*scale^7;
%     
%         desired_state.pos(2) = y((8*(t_indx-1))+1) + y((8*(t_indx-1))+2)*scale + y((8*(t_indx-1))+3)*scale^2 ...
%         + y((8*(t_indx-1))+4)*scale^3 + y((8*(t_indx-1))+5)*scale^4 + y((8*(t_indx-1))+6)*scale^5 ...
%         + y((8*(t_indx-1))+7)*scale^6 + y((8*(t_indx-1))+8)*scale^7;
%     
%         desired_state.pos(3) = z((8*(t_indx-1))+1) + z((8*(t_indx-1))+2)*scale + z((8*(t_indx-1))+3)*scale^2 ...
%         + z((8*(t_indx-1))+4)*scale^3 + z((8*(t_indx-1))+5)*scale^4 + z((8*(t_indx-1))+6)*scale^5 ...
%         + z((8*(t_indx-1))+7)*scale^6 + z((8*(t_indx-1))+8)*scale^7;
%         
%         desired_state.vel(1) = x((8*(t_indx-1))+2) + 2*x((8*(t_indx-1))+3)*scale ...
%         + 3*x((8*(t_indx-1))+4)*scale^2 + 4*x((8*(t_indx-1))+5)*scale^3 + 5*x((8*(t_indx-1))+6)*scale^4 ...
%         + 6*x((8*(t_indx-1))+7)*scale^5 + 7*x((8*(t_indx-1))+8)*scale^6;
%     
%         desired_state.vel(2) = y((8*(t_indx-1))+2) + 2*y((8*(t_indx-1))+3)*scale ...
%         + 3*y((8*(t_indx-1))+4)*scale^2 + 4*y((8*(t_indx-1))+5)*scale^3 + 5*y((8*(t_indx-1))+6)*scale^4 ...
%         + 6*y((8*(t_indx-1))+7)*scale^5 + 7*y((8*(t_indx-1))+8)*scale^6;
%     
%         desired_state.vel(3) = z((8*(t_indx-1))+2) + 2*z((8*(t_indx-1))+3)*scale ...
%         + 3*z((8*(t_indx-1))+4)*scale^2 + 4*z((8*(t_indx-1))+5)*scale^3 + 5*z((8*(t_indx-1))+6)*scale^4 ...
%         + 6*z((8*(t_indx-1))+7)*scale^5 + 7*z((8*(t_indx-1))+8)*scale^6;
% 
%         desired_state.acc(1) = 2*x((8*(t_indx-1))+3) + 6*x((8*(t_indx-1))+4)*scale ...
%         + 12*x((8*(t_indx-1))+5)*scale^2 + 20*x((8*(t_indx-1))+6)*scale^3 ...
%         + 30*x((8*(t_indx-1))+7)*scale^4 + 42*x((8*(t_indx-1))+8)*scale^5;
%     
%         desired_state.acc(2) = 2*y((8*(t_indx-1))+3) + 6*y((8*(t_indx-1))+4)*scale ...
%         + 12*y((8*(t_indx-1))+5)*scale^2 + 20*y((8*(t_indx-1))+6)*scale^3 ...
%         + 30*y((8*(t_indx-1))+7)*scale^4 + 42*y((8*(t_indx-1))+8)*scale^5;
%     
%         desired_state.acc(3) = 2*z((8*(t_indx-1))+3) + 6*z((8*(t_indx-1))+4)*scale ...
%         + 12*z((8*(t_indx-1))+5)*scale^2 + 20*z((8*(t_indx-1))+6)*scale^3 ...
%         + 30*z((8*(t_indx-1))+7)*scale^4 + 42*z((8*(t_indx-1))+8)*scale^5;
%     
%         desired_state.yaw = 0;
%         desired_state.yawdot = 0;
%     end
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

%     function [x,y,z] = get_coeff(waypoints)
%         tx = 3.4641; 
%         n = size(waypoints,2)-1;
%         A = zeros(8*n,8*n);
%         B = zeros(8*n,8*n);
%         C = zeros(8*n,8*n);
%         b1 = zeros(8*n,1);
%         b2 = zeros(8*n,1);
%         b3 = zeros(8*n,1);
% 
%         A(1,1) = 1; b1(1) = waypoints(1,1);
%         A(2,9) = 1; b1(2) = waypoints(1,2);
%         A(3,17) = 1; b1(3) = waypoints(1,3);
%         A(4,25) = 1; b1(4) = waypoints(1,4);
%         A(5,1:8) = [1, tx, tx^2, tx^3, tx^4, tx^5, tx^6, tx^7]; b1(5) = waypoints(1,2);
%         A(6,9:16) = A(5,1:8); b1(6) = waypoints(1,3);
%         A(7,17:24) = A(5,1:8); b1(7) = waypoints(1,4);
%         A(8,25:32) = A(5,1:8); b1(8) = waypoints(1,5);
%         A(9,2) = 1; b1(9) = 0;
%         A(10,3) = 1; b1(10) = 0;
%         A(11,4) = 1; b1(11) = 0;
%         A(12,26) = 1; b1(12) = 0;
%         A(13,27) = 1; b1(13) = 0;
%         A(14,28) = 1; b1(14) = 0;
%         A(15,1:10) = [0, 1, 2*tx, 3*tx^2, 4*tx^3, 5*tx^4, 6*tx^5, 7*tx^6, 0, -1]; b1(15)=0;
%         A(16,9:18) = A(15,1:10); b1(16)=0;
%         A(17,17:26) = A(15,1:10); b1(17)=0;
%         A(18,1:11) = [0, 0, 2, 6*tx, 12*tx^2, 20*tx^3, 30*tx^4, 42*tx^5, 0, 0, -2]; b1(18)=0;
%         A(19,9:19) = A(18,1:11); b1(19)=0;
%         A(20,17:27) = A(18,1:11); b1(20)=0;
%         A(21,1:12) = [0, 0, 0, 6, 24*tx, 60*tx^2, 120*tx^3, 210*tx^4, 0, 0, 0, -6]; b1(21)=0;
%         A(22,9:20) = A(21,1:12); b1(22)=0;
%         A(23,17:28) = A(21,1:12); b1(23)=0;
%         A(24,1:13) = [0, 0, 0, 0, 24, 120*tx, 360*tx^2, 840*tx^3, 0, 0, 0, 0, -24]; b1(24)=0;
%         A(25,9:21) = A(24,1:13); b1(25)=0;
%         A(26,17:29) = A(24,1:13); b1(26)=0;
%         A(27,1:14) = [0, 0, 0, 0, 0, 120, 720*tx, 2520*tx^2, 0, 0, 0, 0, 0, -120]; b1(27)=0;
%         A(28,9:22) = A(27,1:14); b1(28)=0;
%         A(29,17:30) = A(27,1:14); b1(29)=0;
%         A(30,1:15) = [0, 0, 0, 0, 0, 0, 720, 5040*tx, 0, 0, 0, 0, 0, 0, -720]; b1(30)=0;
%         A(31,9:23) = A(30,1:15); b1(31)=0;
%         A(32,17:31) = A(30,1:15); b1(32)=0;
% 
%         B(1,1) = 1; b2(1) = waypoints(2,1);
%         B(2,9) = 1; b2(2) = waypoints(2,2);
%         B(3,17) = 1; b2(3) = waypoints(2,3);
%         B(4,25) = 1; b2(4) = waypoints(2,4);
%         B(5,1:8) = [1, tx, tx^2, tx^3, tx^4, tx^5, tx^6, tx^7]; b2(5) = waypoints(2,2);
%         B(6,9:16) = B(5,1:8); b2(6) = waypoints(2,3);
%         B(7,17:24) = B(5,1:8); b2(7) = waypoints(2,4);
%         B(8,25:32) = B(5,1:8); b2(8) = waypoints(2,5);
%         B(9,2) = 1; b2(9) = 0;
%         B(10,3) = 1; b2(10) = 0;
%         B(11,4) = 1; b2(11) = 0;
%         B(12,26) = 1; b2(12) = 0;
%         B(13,27) = 1; b2(13) = 0;
%         B(14,28) = 1; b2(14) = 0;
%         B(15,1:10) = [0, 1, 2*tx, 3*tx^2, 4*tx^3, 5*tx^4, 6*tx^5, 7*tx^6, 0, -1]; b2(15)=0;
%         B(16,9:18) = B(15,1:10); b2(16)=0;
%         B(17,17:26) = B(15,1:10); b2(17)=0;
%         B(18,1:11) = [0, 0, 2, 6*tx, 12*tx^2, 20*tx^3, 30*tx^4, 42*tx^5, 0, 0, -2]; b2(18)=0;
%         B(19,9:19) = B(18,1:11); b2(19)=0;
%         B(20,17:27) = B(18,1:11); b2(20)=0;
%         B(21,1:12) = [0, 0, 0, 6, 24*tx, 60*tx^2, 120*tx^3, 210*tx^4, 0, 0, 0, -6]; b2(21)=0;
%         B(22,9:20) = B(21,1:12); b2(22)=0;
%         B(23,17:28) = B(21,1:12); b2(23)=0;
%         B(24,1:13) = [0, 0, 0, 0, 24, 120*tx, 360*tx^2, 840*tx^3, 0, 0, 0, 0, -24]; b2(24)=0;
%         B(25,9:21) = B(24,1:13); b2(25)=0;
%         B(26,17:29) = B(24,1:13); b2(26)=0;
%         B(27,1:14) = [0, 0, 0, 0, 0, 120, 720*tx, 2520*tx^2, 0, 0, 0, 0, 0, -120]; b2(27)=0;
%         B(28,9:22) = B(27,1:14); b2(28)=0;
%         B(29,17:30) = B(27,1:14); b2(29)=0;
%         B(30,1:15) = [0, 0, 0, 0, 0, 0, 720, 5040*tx, 0, 0, 0, 0, 0, 0, -720]; b2(30)=0;
%         B(31,9:23) = B(30,1:15); b2(31)=0;
%         B(32,17:31) = B(30,1:15); b2(32)=0;
% 
%         C(1,1) = 1; b3(1) = waypoints(3,1);
%         C(2,9) = 1; b3(2) = waypoints(3,2);
%         C(3,17) = 1; b3(3) = waypoints(3,3);
%         C(4,25) = 1; b3(4) = waypoints(3,4);
%         C(5,1:8) = [1, tx, tx^2, tx^3, tx^4, tx^5, tx^6, tx^7]; b3(5) = waypoints(3,2);
%         C(6,9:16) = C(5,1:8); b3(6) = waypoints(3,3);
%         C(7,17:24) = C(5,1:8); b3(7) = waypoints(3,4);
%         C(8,25:32) = C(5,1:8); b3(8) = waypoints(3,5);
%         C(9,2) = 1; b3(9) = 0;
%         C(10,3) = 1; b3(10) = 0;
%         C(11,4) = 1; b3(11) = 0;
%         C(12,26) = 1; b3(12) = 0;
%         C(13,27) = 1; b3(13) = 0;
%         C(14,28) = 1; b3(14) = 0;
%         C(15,1:10) = [0, 1, 2*tx, 3*tx^2, 4*tx^3, 5*tx^4, 6*tx^5, 7*tx^6, 0, -1]; b3(15)=0;
%         C(16,9:18) = C(15,1:10); b3(16)=0;
%         C(17,17:26) = C(15,1:10); b3(17)=0;
%         C(18,1:11) = [0, 0, 2, 6*tx, 12*tx^2, 20*tx^3, 30*tx^4, 42*tx^5, 0, 0, -2]; b3(18)=0;
%         C(19,9:19) = C(18,1:11); b3(19)=0;
%         C(20,17:27) = C(18,1:11); b3(20)=0;
%         C(21,1:12) = [0, 0, 0, 6, 24*tx, 60*tx^2, 120*tx^3, 210*tx^4, 0, 0, 0, -6]; b3(21)=0;
%         C(22,9:20) = C(21,1:12); b3(22)=0;
%         C(23,17:28) = C(21,1:12); b3(23)=0;
%         C(24,1:13) = [0, 0, 0, 0, 24, 120*tx, 360*tx^2, 840*tx^3, 0, 0, 0, 0, -24]; b3(24)=0;
%         C(25,9:21) = C(24,1:13); b3(25)=0;
%         C(26,17:29) = C(24,1:13); b3(26)=0;
%         C(27,1:14) = [0, 0, 0, 0, 0, 120, 720*tx, 2520*tx^2, 0, 0, 0, 0, 0, -120]; b3(27)=0;
%         C(28,9:22) = C(27,1:14); b3(28)=0;
%         C(29,17:30) = C(27,1:14); b3(29)=0;
%         C(30,1:15) = [0, 0, 0, 0, 0, 0, 720, 5040*tx, 0, 0, 0, 0, 0, 0, -720]; b3(30)=0;
%         C(31,9:23) = C(30,1:15); b3(31)=0;
%         C(32,17:31) = C(30,1:15); b3(32)=0;
% 
%         x = A\b1;
%         y = B\b2;
%         z = C\b3; 
%     end
% end
