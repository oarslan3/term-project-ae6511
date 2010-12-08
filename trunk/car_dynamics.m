function dX = car_dynamics(t, X, u, time)

    global CONSTANTS
    %global time
    
    g   = CONSTANTS.g;
    m   = CONSTANTS.m;
    Iz = CONSTANTS.Iz;
    l_F = CONSTANTS.l_F;
    l_R = CONSTANTS.l_R;
    mu  = CONSTANTS.mu;
    C   = CONSTANTS.C;
    B   = CONSTANTS.B;
    f_Fz = CONSTANTS.f_Fz;
    f_Rz = CONSTANTS.f_Rz;
    
    % Dynamic equation for case 1
    % udot = v.*r;
    % vdot = -u.*r;
    % rdot = g.*uc;
    % psidot = r;
    % xdot = u.*cos(psi) - v.*sin(psi);
    % ydot = u.*sin(psi) + v.*cos(psi);

    uapp = interp1(time,u,t,'linear');

    
    Vx  = X(1);
    Vy  = X(2);
    r   = X(3);
    psi = X(4);
    xi  = X(5);
    yi  = X(6);

    % ks = find(time==t); 
    f_Fx  = uapp(1);
    f_Rx  = uapp(2);
    delta = uapp(3);
    
    f_Fz = CONSTANTS.f_Fz;
    f_Rz = CONSTANTS.f_Rz;
    
    mu = CONSTANTS.mu;
    
    Vxw = cos(delta).*Vx + sin(delta).*Vy;
    Vyw = -sin(delta).*Vx + cos(delta).*Vy;
    
    f_Fx = f_Fx*sgn(Vxw);
    
    if ( f_Fx >= 0)
        f_Fx = 0;
    elseif (f_Fx <= -mu*f_Fz)
        f_Fx = -mu*f_Fz;
    end
    
    f_Rx = sgn(Vx)*f_Rx;
    
    if ( f_Rx >= mu*f_Rz)
        f_Rx =  mu*f_Rz;
    elseif (f_Rx <= -mu*f_Rz)
        f_Rx = -mu*f_Rz;
    end
    
    
    if ( delta >= 45*pi/180)
        delta = 45*pi/180;
    elseif (delta <= -45*pi/180)
        delta = -45*pi/180;
    end
    
    %Auxiliar variables
    V = sqrt(Vx.^2+Vy.^2);
    beta = atan2(Vy,Vx);

    f_Fy_max = sqrt( (mu*f_Fz).^2 - f_Fx.^2 );
    s_Fy = (V.*sin(beta-delta)+r*l_F.*cos(delta))./(V.*cos(beta-delta)+r*l_F.*sin(delta));
    f_Fy = -f_Fy_max.*sin(C*atan(B*s_Fy));


    f_Ry_max = sqrt( (mu*f_Rz).^2 - f_Rx.^2 );
    s_Ry =(V.*sin(beta)-r*l_R)./(V.*cos(beta));
    f_Ry = -f_Ry_max.*sin(C*atan(B*s_Ry));

    
    dVx = 1/m.*(f_Fx.*cos(delta) - f_Fy.*sin(delta) + f_Rx) + Vy.*r;
    dVy = 1/m.*(f_Fx.*sin(delta) + f_Fy.*cos(delta) + f_Ry) - Vx.*r;
    dr = 1/Iz.*((f_Fy.*cos(delta) + f_Fx.*sin(delta)).*l_F - f_Ry.*l_R);
    dpsi = r;
    dxi = cos(psi).*Vx - sin(psi).*Vy;
    dyi = sin(psi).*Vx + cos(psi).*Vy;
    
    dX = [dVx dVy dr dpsi dxi dyi]';


end