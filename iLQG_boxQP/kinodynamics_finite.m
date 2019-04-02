function [ x_new ] = kinodynamics_finite( x, u, dt, refTraj)
%kinodynamics_finite 4th order Runge-Kutta integration to discretize dynamics

k1 = kinodynamics(x, u, refTraj);
%     k2 = dynamics(x + 0.5 * dt * k1, u);
%     k3 = dynamics(x + 0.5 * dt * k2, u);
%     k4 = dynamics(x + dt * k3, u);

x_new = x + dt*k1; %(dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4); %+ dt*k1;

end
