function trajectory = joint_space_trajectory(theta_i, theta_f, velocity_i, velocity_f, t_i, t_f, samples)
    syms a0 a1 a2 a3;

    % define system of equations
    eq1 = 0 == a0 + a1*t_i + a2*t_i^2 + a3*t_i^3 - theta_i;
    eq2 = 0 == a0 + a1*t_f + a2*t_f^2 + a3*t_f^3 - theta_f;
    eq3 = velocity_i == a1 + 2*a2*t_i + 3*a3*t_i^2;
    eq4 = velocity_f == a1 + 2*a2*t_f + 3*a3*t_f^2;

    % compute polynomial solution (degree 3)
    [a0, a1, a2, a3] = vpasolve(eq1,eq2,eq3,eq4,a0,a1,a2,a3);
    p = cast([a3 a2 a1 a0], "double");

    % sample points along trajectory
    time_step = linspace(t_i,t_f,samples);
    trajectory = polyval(p,time_step,3);

end