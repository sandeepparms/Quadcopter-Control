function [ desired_state ] = traj_generator2(t, state, waypoints)

persistent coffx coffy coffz traj_time d0 waypoints0
if nargin>2
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
    xway = waypoints(1,:);
    yway = waypoints(2,:);
    zway = waypoints(3,:);
    coffx = getcoeff(xway);
    coffy = getcoeff(yway);
    coffz = getcoeff(zway);
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if (t>=traj_time(end))
        t = traj_time(end)-0.01;
    end
    
    t_index = find(traj_time>t,1)-1;
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
    t_index = max(t_index,1);
    scale = (t-traj_time(t_index))/d0(t_index);
    
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    else
        
        t0 = polyT(8,0,scale)';
        t1 = polyT(8,1,scale)';
        t2 = polyT(8,2,scale)';
        index = [8*(t_index-1)+1 : t_index*8];
        desired_state.pos = [ ...
        coffx(index)'*t0; ...
        coffy(index)'*t0; ...
        coffz(index)'*t0 ];
        desired_state.vel=[ ...
        coffx(index)'*t1*(1/d0(t_index)); ...
        coffy(index)'*t1*(1/d0(t_index)); ...
        coffz(index)'*t1*(1/d0(t_index))];
        desired_state.acc=[ ...
        coffx(index)'*t2*(1/d0(t_index)^2); ...
        coffy(index)'*t2*(1/d0(t_index)^2); ...
        coffz(index)'*t2*(1/d0(t_index))^2];
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end

end

%%
function [coff, A, b] = getcoeff(waypoints)
n = size(waypoints,2)-1; % number of segments P1..n
A = zeros(8*n, 8*n);
b = zeros(1,8*n);
% --------------------------------------------

% YOUR CODE HERE 
% Fill A and b matices with values using loops

% Example for first 4 equations:

for i=1:n
    b(1,i) = waypoints(i);
end

for i=n+1:2*n
    b(1,i) = waypoints(i-3);
end

row = 1;
% Constrain 1) Pi(0) = Wi for all i=1..n
for i=1:n
    A(row,(8*(i-1))+1:8*i) = polyT(8,0,0); 
    row = row + 1;
end
% Constrain 2) Pi(1) = Wi+1 for all i=1..n
for i=n+1:2*n
    A(row,(8*(i-5))+1:8*(i-4)) = polyT(8,0,1); 
    row = row + 1;
end
% Constrain 3) P1(k)(0)= 0 for all 1<=k<=3
A(9,1:8) = polyT(8,1,0);
A(10,1:8) = polyT(8,2,0);
A(11,1:8) = polyT(8,3,0);

% Constrain 4) Pn(k)(1) = 0 for all 1<=k<=3
A(12,25:32) = polyT(8,1,1);
A(13,25:32) = polyT(8,2,1);
A(14,25:32) = polyT(8,3,1);

% Constrain 5) Pi-1(k)(1) = Pi(k)(0) for all i=2..n and for all k=1..6
A(15, 1:16) = [polyT(8,1,1) -polyT(8,1,0)];
A(16, 9:24) = [polyT(8,1,1) -polyT(8,1,0)];
A(17, 17:32) = [polyT(8,1,1) -polyT(8,1,0)];

A(18, 1:16) = [ polyT(8,2,1) -polyT(8,2,0)];
A(19, 9:24) = [ polyT(8,2,1) -polyT(8,2,0)];
A(20, 17:32) = [ polyT(8,2,1) -polyT(8,2,0)];

A(21, 1:16) = [ polyT(8,3,1) -polyT(8,3,0)];
A(22, 9:24) = [ polyT(8,3,1) -polyT(8,3,0)];
A(23, 17:32) = [ polyT(8,3,1) -polyT(8,3,0)];

A(24, 1:16) = [ polyT(8,4,1) -polyT(8,4,0)];
A(25, 9:24) = [ polyT(8,4,1) -polyT(8,4,0)];
A(26, 17:32) = [ polyT(8,4,1) -polyT(8,4,0)];

A(27, 1:16) = [ polyT(8,5,1) -polyT(8,5,0)];
A(28, 9:24) = [ polyT(8,5,1) -polyT(8,5,0)];
A(29, 17:32) = [ polyT(8,5,1) -polyT(8,5,0)];

A(30, 1:16) = [ polyT(8,6,1) -polyT(8,6,0)];
A(31, 9:24) = [ polyT(8,6,1) -polyT(8,6,0)];
A(32, 17:32) = [ polyT(8,6,1) -polyT(8,6,0)];

% --------------------------------------------
coff = A\b';

end
%%
function [T] = polyT(n,k,t)
%n is the polynom number of coefficients, k is the requested derivative and
%t is the actual value of t (this can be anything, not just 0 or 1).
T = zeros(n,1);
D = zeros(n,1);
%Init:
for i=1:n
D(i) = i-1;
T(i) = 1;
end
%Derivative:
for j=1:k
    for i=1:n
        T(i) = T(i) * D(i);

        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end
%put t value
for i=1:n
T(i) = T(i) * t^D(i);
end
T = T';
end
%%
end