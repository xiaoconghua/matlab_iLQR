function [x,u, u0] = test_trajopt
% A demo of iLQG/DDP for trajectory optimization
clc; clear;
close all;

% set up the optimization problem
global T;
T       = 50;              % horizon
global dt;
dt      = 0.05;
global x0;  %[arclen, lateral, heading_error, vel]
x0      = [0; 0; 0; 6];   % initial state

% TODO change this according to x0 and x_des?
u0      = zeros(2,T); % Just setting up shape here
u0(1,:) = 0*randn(1,T); % commanded steering
u0(2,:) = 1*randn(1,T);  % commanded acceleration

% controller limit
Op.lims  = [-1  1;   
            -2 2];
Op.maxIter = 30;
 

% Generate a reference trajectory
refTraj = traj_generation();

DYNCST  = @(x,u,r) trajopt_dyn_cst(x,u,refTraj);
% === Run the optimization!
[x,u]= iLQG(DYNCST, x0, u0, Op);

figure;
subplot(2, 2, 1)
plot(x(1, :)); 
hold on
plot(refTraj.arclength);
title('arclength')
legend('opt', 'ref')
subplot(2, 2, 2)
plot(x(4, :)); 
hold on
plot(refTraj.velocity);
title('velocity')
legend('opt', 'ref')
subplot(2, 2, 3)
plot(x(2, :)); 
title('lateral offset')
subplot(2, 2, 4)
plot(x(3, :)); 
title('heading error')

figure;
subplot(2, 1, 1)
plot(u(1, :))
legend('heading command')
subplot(2, 1, 2)
plot(u(2, :))
legend('acceleration command')

end %test_car

% ----------------------------------------
% -----------Dynamics and cost------------
% ----------------------------------------
function new_x = car_dynamics(x,u,refTraj)

global dt;
new_x = zeros(size(x));

for i = 1:size(x,2)
    new_x(:,i) = kinodynamics_finite(x(:,i), u(:,i), dt, refTraj);
end

end %car_dynamics

function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = trajopt_dyn_cst(x,u,refTraj)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives

if nargout == 2
    f = car_dynamics(x,u,refTraj);
    c = trajopt_cost(x,u,refTraj);
else
    % state and control indices
    ix = 1:4;
    iu = 5:6;
    
    % dynamics first derivatives
    % J - Jacobian, derivative of states wrt states and control inputs
    % n x (n+m) x T , where n=dim(x), m=dim(u), T=horizon
    xu_dyn  = @(xu) car_dynamics(xu(ix,:),xu(iu,:), refTraj);
    J       = finite_difference(xu_dyn, [x; u]);
    fx      = J(:,ix,:);
    fu      = J(:,iu,:);
    
    [fxx,fxu,fuu] = deal([]);  
    
    % cost first derivatives
    xu_cost = @(xu) trajopt_cost(xu(ix,:),xu(iu,:), refTraj);
    J       = squeeze(finite_difference(xu_cost, [x; u]));
    cx      = J(ix,:);
    cu      = J(iu,:);
    
    % cost second derivatives
    xu_Jcst = @(xu) squeeze(finite_difference(xu_cost, xu));
    JJ      = finite_difference(xu_Jcst, [x; u]);
    JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
    cxx     = JJ(ix,ix,:);
    cxu     = JJ(ix,iu,:);
    cuu     = JJ(iu,iu,:);
    
    [f,c] = deal([]);
end
end %trajopt_dyn_cost

function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 2^-17;
end

[n, K]  = size(x);
H       = [zeros(n,1) h*eye(n)];
H       = permute(H, [1 3 2]);
X       = pp(x, H);
X       = reshape(X, n, K*(n+1));
Y       = fun(X);
m       = numel(Y)/(K*(n+1));
Y       = reshape(Y, m, K, n+1);
J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
J       = permute(J, [1 3 2]);
end

% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);
end


