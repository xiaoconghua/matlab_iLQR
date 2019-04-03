function [c] = trajopt_cost(x, u, refTraj)
% cost function for trajopt problem
% sum of terms:
% lu: quadratic cost on controls
% lf: final cost on state from target state
% lx: running cost on lateral
% lobs: static related cost
% lb: boundary related cost

% Get the final state from the reference trajectory. Importantly, we want
% the lateral offset and heading error to be 0.
x_final = [refTraj.arclength(end), 0, 0, refTraj.velocity(end)]';

% final cost
cf  = [10 10 10 10];    % final cost coefficients
pf  = [.1 .1 .1 .1]';    % smoothness scales for final cost

final = isnan(u(1,:));
u(:,final)  = 0;
if any(final)
   dist     = bsxfun(@minus,x(1:4,final),x_final);
   llf      = cf*(sabs(dist,pf)+sabs(dist,pf).^2);
   lf       = double(final);
   lf(final)= llf;
else
   lf       = 0;
end

% Control cost
cu  = 1e-1*[2 1];        % control cost coefficients
lu  = cu*u.^2;

% Running cost. Currently, we only consider running cost on lateral offset
% and heading error
cx  = 1e-2*[1 1];   % running cost coefficients 
px  = [.01 .01]';   % smoothness scales for running cost

dist = bsxfun(@minus,x(2:3,:),x_final(2:3));
lx = cx*sabs(dist,px);

global obs
% Dynamic obstacle cost
k_pos = 1e-3;
d_thres = 0.1;
arc_thres = 0.2;

lobs = 0;
if ~isempty(obs)
    for i = 1:size(obs, 2)
        obs_i = obs(:, i);
        obs_mat = repmat(obs_i,1,size(x,2));
        arc2obs = bsxfun(@minus,obs_mat(1, :), x(1, :));
        vec2obs = bsxfun(@minus,obs_mat,x(1:2,:));
        dist2obs = sqrt(sum(vec2obs.^2,1));

%         Ustatic = (1./dist2obs - 1/d_thres).^2;  
        Ustatic = 1./(dist2obs - d_thres).^2;   
        toofar = abs(arc2obs) >= arc_thres;
        Ustatic(toofar) = 0;

        lobs = lobs + k_pos*Ustatic;
    end
end

% stay with the boundary cost
% For initial pass, we assume constant lateral bounds
bound_right = -1;
bound_left  = 1;
k_bound = 1e-1;
lb    = k_bound .* (x(2, :) - bound_right).*(x(2,:) - bound_left);

% total cost
c     = lu + lf + lx + lobs + lb;
end

% smooth absolute-value function (a.k.a pseudo-Huber)
function y = sabs(x,p)
y = pp( sqrt(pp(x.^2,p.^2)), -p);
end

% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);
end
