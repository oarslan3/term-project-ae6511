%---------------------------------------
% Begin: function ManeuverDae.m
%---------------------------------------

function dae = ManeuverDae(solution)

    global CONSTANTS
    
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
    
    t = solution.time;
    X = solution.state;
    u = solution.control;
    p = solution.parameter;

    % There are 4 states for case 1
    % u = xx(:,1);
    % v = xx(:,2);
    % r = xx(:,3);
    % psi = xx(:,4);
    % x = xx(:,5);
    % y = xx(:,6);

    % Dynamic equation for case 1
    % udot = v.*r;
    % vdot = -u.*r;
    % rdot = g.*uc;
    % psidot = r;
    % xdot = u.*cos(psi) - v.*sin(psi);
    % ydot = u.*sin(psi) + v.*cos(psi);

    
    Vx  = X(:,1);
    Vy  = X(:,2);
    r   = X(:,3);
    psi = X(:,4);
    xi  = X(:,5);
    yi  = X(:,6);

     
    f_Fx  = u(:,1);
    f_Rx  = u(:,2);
    delta = u(:,3);
    
    % f_Fy = 0;
    % f_Ry = 0;
   

 
    Vxw = cos(delta).*Vx + sin(delta).*Vy;
    Vyw = -sin(delta).*Vx + cos(delta).*Vy;
    
    f_Fx = f_Fx.*sign(Vxw);
    f_Rx = sign(Vx).*f_Rx;
    
    k = 1;
    while (k <= size(t,1))
        
        if ( f_Fx(k) >= 0)
            f_Fx(k) = 0;
        elseif (f_Fx <= -mu*f_Fz)
            f_Fx(k) = -mu*f_Fz;
        end

        if ( f_Rx(k) >= mu*f_Rz)
            f_Rx(k) =  mu*f_Rz;
        elseif (f_Rx(k) <= -mu*f_Rz)
            f_Rx(k) = -mu*f_Rz;
        end
    
        k = k + 1;
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

    % f_Fy = zeros(len,1);
    % f_Ry = zeros(len,1);
    
    %{
    for k = 1:len
        Xt = X(k,:);
        ut = u(k,:);
        
        st = ComputeLateralSlips(Xt,ut);
        [f_Fyt, f_Ryt] = ComputeForces(Xt,ut,st);
        
        f_Fy(k,1) = f_Fyt;
        f_Ry(k,1) = f_Ryt;
    end
    f_Fy
    f_Ry
    %}
    
    dVx = 1/m.*(f_Fx.*cos(delta) - f_Fy.*sin(delta) + f_Rx) + Vy.*r;
    dVy = 1/m.*(f_Fx.*sin(delta) + f_Fy.*cos(delta) + f_Ry) - Vx.*r;
    dr = 1/Iz.*((f_Fy.*cos(delta) + f_Fx.*sin(delta)).*l_F - f_Ry.*l_R);
    dpsi = r;
    dxi = cos(psi).*Vx - sin(psi).*Vy;
    dyi = sin(psi).*Vx + cos(psi).*Vy;
    
    dXt = [dVx dVy dr dpsi dxi dyi];

    
    dae = dXt;

    

end

%------------------------------------
% END: function ManeuverDae.m
%------------------------------------