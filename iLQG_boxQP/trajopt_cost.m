function [c] = trajopt_cost(x, u, refTraj)
% cost function for trajopt problem
% sum of terms:
% lu: quadratic cost on controls
% lf: final cost on state from target state
% lx: running cost on lateral
% ldu: quadratic cost on change in controls
% lobs: static and dynamic obstacle costs

global obs;

x_final = [refTraj.arclength(end), 0, 0, refTraj.velocity(end)];
% final cost
cf  = [ 10 1 1 10];    % final cost coefficients
pf  = [ .01 .01 .1 .1]';    % smoothness scales for final cost

final = isnan(u(1,:));
u(:,final)  = 0;
if any(final)
   dist = bsxfun(@minus,x(1:4,final),x_final);
   llf      = cf*(sabs(dist,pf)+sabs(dist,pf).^2);
   lf       = double(final);
   lf(final)= llf;
else
   lf       = 0;
end

% Control cost
cu  = 1e-2*[.1 .1];        % control cost coefficients
lu  = cu*u.^2;

% Running cost 
cx  = 1e-1*[5  1 4];          % running cost coefficients 
cv  = 1e-2*[5 0.5 0.2];
px  = [.01 .01 .1]';   % smoothness scales for running cost

dist = bsxfun(@minus,x(1:3,:),x_des(1:3));
vdist = bsxfun(@minus,x(4:6,:),x_des(4:6));
lx = cx*sabs(dist,px) + cv*sabs(vdist,px);

% % Dynamic obstacle cost
% k_pos = 0.5;
% k_vel = 0.1;
% d_thres = 0.3;
% 
% lobs = 0;
% if ~isempty(obs)
%     % pos = x(1:2,:); vel = x(4:5,:);
%     obs_mat = repmat(obs,1,size(x,2));
%     vec2obs = bsxfun(@minus,obs_mat,x(1:2,:));
%     dist2obs = sqrt(sum(vec2obs.^2,1));
%     velnorms = sqrt(sum(x(4:5,:).^2,1));
%     
%     Ustatic = (1./dist2obs - 1/d_thres).^2;    
%     toofar = dist2obs >= d_thres;
%     Ustatic(toofar) = 0;
%     
%     Udynamic = diag(vec2obs'*x(4:5,:))'./(dist2obs.*velnorms);
%     neg = Udynamic<0;
%     Udynamic(neg) = 0;
%         
%     lobs = k_pos*Ustatic + k_vel*Udynamic;
% end


% total cost
c     = lu + lf + lx + ld;
end

% smooth absolute-value function (a.k.a pseudo-Huber)
function y = sabs(x,p)
y = pp( sqrt(pp(x.^2,p.^2)), -p);
end

% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);
end

function c = tt(a,b)
c = bsxfun(@times,a,b);
end
