function [sim_time y] = ode45_sim(case_param, u, time)

  %dX = car_dynamics(t, X, u)

    options = odeset('RelTol',1e-4,'AbsTol',[1e-4 1e-4 1e-5 1e-4 1e-4 1e-5]);

    %global time
    %time = solution.time;
    %tf = time(end);
    %tf = 2;
    tf = time(end);

    Vx_t0  = case_param.guess_Vx(1); 
    Vy_t0  = case_param.guess_Vy(1);
    r_t0   = case_param.guess_r(1);
    psi_t0 = case_param.guess_psi(1);
    xi_t0  = case_param.guess_xi(1);
    yi_t0  = case_param.guess_yi(1); 

    %u = solution.control;
    %u = ones(size(time,1),1)*[-1000 -10000 -45*pi/180; ];
    %u (50:100,3) = -u (50:100,3);
    %time = [0:0.0001:tf];

    [sim_time y] = ode45(@car_dynamics, ...
                    [0 tf],... %output.solution.time, ...
                    [Vx_t0 Vy_t0 r_t0 psi_t0 xi_t0 yi_t0], ...
                    options, u, time);

    %{
    Xs = output.solution.state;
    us = output.solution.control;

    Vx  = Xs(:,1);
    Vy  = Xs(:,2);
    r   = Xs(:,3);
    psi = Xs(:,4);
    xi  = Xs(:,5);
    yi  = Xs(:,6);


    f_Fx  = us(:,1);
    f_Rx  = us(:,2);
    delta = us(:,3);

    %Auxiliar variables
    V = sqrt(Vx.^2+Vy.^2);
    beta = atan2(Vy,Vx);

    f_Fy_max = sqrt( (CONSTANTS.mu*CONSTANTS.f_Fz).^2 - f_Fx.^2 );
    s_Fy = (V.*sin(beta-delta)+r*CONSTANTS.l_F.*cos(delta))./(V.*cos(beta-delta)+r*CONSTANTS.l_F.*sin(delta));
    f_Fy = -f_Fy_max.*sin(CONSTANTS.C*atan(CONSTANTS.B*s_Fy));


    f_Ry_max = sqrt( (CONSTANTS.mu*CONSTANTS.f_Rz).^2 - f_Rx.^2 );
    s_Ry =(V.*sin(beta)-r*CONSTANTS.l_R)./(V.*cos(beta));
    f_Ry = -f_Ry_max.*sin(CONSTANTS.C*atan(CONSTANTS.B*s_Ry));

     %}

end