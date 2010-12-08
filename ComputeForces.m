function [f_Fy, f_Ry] = ComputeForces (X, u, s)

    global CONSTANTS

    g   = CONSTANTS.g;
    m   = CONSTANTS.m;
    Iz = CONSTANTS.Iz;
    l_F = CONSTANTS.l_F;
    l_R = CONSTANTS.l_R;
    mu  = CONSTANTS.mu;
    C   = CONSTANTS.C;
    B   = CONSTANTS.B;

    Vx  = X(1); 
    Vy  = X(2);
    r   = X(3);
    psi = X(4);

    f_Fx  = u(1);
    f_Rx  = u(2);
    delta = u(3);

    s_Fy = s(1);
    s_Ry = s(2); 


    f_Fz = m*g*l_R*(l_F + l_R);
    f_Rz = m*g*l_F*(l_F + l_R);

    f_Fy_max = sqrt((mu*f_Fz)^2 - f_Fx^2);
    f_Ry_max = sqrt((mu*f_Rz)^2 - f_Rx^2);

    f_Fy = -f_Fy_max*sin(C*atan(B*s_Fy)); 
    f_Ry = -f_Ry_max*sin(C*atan(B*s_Ry));

end