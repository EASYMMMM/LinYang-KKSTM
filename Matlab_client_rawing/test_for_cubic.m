
init_v=0.01*ones(7,1);
theta_d_des(:,1) = init_v;
[theta,theta_dot,theta_dotdot,pp, VBCC] = cubicpolytraj_test(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);