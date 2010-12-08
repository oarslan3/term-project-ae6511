function s = ComputeLateralSlips( X, u)
    %#eml
    
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


    if (Vx == 0 && Vy ==0)
        beta = 0;
    else
        beta = atan(Vy/Vx);
    end

    V = sqrt(Vx^2 + Vy^2);

    s_Fy_num = (V*sin(beta-delta) + r*l_F*cos(delta));
    s_Fy_denum = (V*cos(beta-delta) + r*l_F*sin(delta));

    if (s_Fy_num == 0 && s_Fy_denum == 0)
        s_Fy = 0;
    else
        s_Fy = s_Fy_num/s_Fy_denum;
    end

    s_Ry_num = (V*sin(beta) - r*l_R);
    s_Ry_denum = (V*cos(beta));

    if (s_Ry_num == 0 && s_Ry_denum == 0)
        s_Ry = 0;
    else
        s_Ry = s_Ry_num/s_Ry_denum;
    end

    s = [s_Fy; s_Ry];

end