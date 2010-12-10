function animate(solution, case_param, movie_name)
    global CONSTANTS car_specs;

    disp('Loading...');
    
    %[sim_time y] = ode45_sim(case_param, solution.control, solution.time);

    newres = size(solution.state,1);
    oldres = size(solution.control,1);
    finalres = 300;
    
    final_time = linspace(0,solution.time(end),finalres);
    
%    temp  = zeros(newres,3);
    
    % Extrapolate all the control values to the simulated resolution
%     for i=1:newres
%         temp(i,1)  = interp1((0:oldres-1)',solution.control(:,1),(i-1)*(oldres-1)/(newres-1));
%         temp(i,2)  = interp1((0:oldres-1)',solution.control(:,2),(i-1)*(oldres-1)/(newres-1));
%         temp(i,3) = interp1((0:oldres-1)',solution.control(:,3),(i-1)*(oldres-1)/(newres-1));
%     end

    i = 1;
    for t = final_time
        Vx(i)    = interp1( solution.time,solution.state(:,1),t);
        Vy(i)    = interp1( solution.time,solution.state(:,2),t);
        r(i)     = interp1( solution.time,solution.state(:,3),t);
        psi(i)   = interp1( solution.time,solution.state(:,4),t);
        xi(i)    = interp1( solution.time,solution.state(:,5),t);
        yi(i)    = interp1( solution.time,solution.state(:,6),t);
        f_Fx(i)  = interp1( solution.time,solution.control(:,1),t);
        f_Rx(i)  = interp1( solution.time,solution.control(:,2),t);
        delta(i) = interp1( solution.time,solution.control(:,3),t);
        i = i + 1;
    end
    
    %% Copied from car_dynamics.m
    
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
    
%     Vx  = y(:,1);
%     Vy  = y(:,2);
%     r   = y(:,3);
%     psi = y(:,4);
%     xi  = y(:,5);
%     yi  = y(:,6);

    f_Fz = CONSTANTS.f_Fz;
    f_Rz = CONSTANTS.f_Rz;
    
    mu = CONSTANTS.mu;
    
    Vxw = cos(delta).*Vx + sin(delta).*Vy;
    Vyw = -sin(delta).*Vx + cos(delta).*Vy;
    
    if ( f_Fx >= 0)
        f_Fx = 0;
    elseif (f_Fx <= -mu*f_Fz)
        f_Fx = -mu*f_Fz;
    end
    
    f_Fx = f_Fx.*sign(Vxw);
    
    if ( f_Rx >= mu*f_Rz)
        f_Rx =  mu*f_Rz;
    elseif (f_Rx <= -mu*f_Rz)
        f_Rx = -mu*f_Rz;
    end
    
    if (f_Rx < 0)   % braking request
        f_Rx = sign(Vx).*f_Rx;
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
    
    if (isnan(s_Fy))
        s_Fy = 0;
    end
    
    f_Fy = -f_Fy_max.*sin(C*atan(B*s_Fy));


    f_Ry_max = sqrt( (mu*f_Rz).^2 - f_Rx.^2 );
    s_Ry =(V.*sin(beta)-r*l_R)./(V.*cos(beta));
    
    if (isnan(s_Ry))
        s_Ry = 0;
    end
    
    f_Ry = -f_Ry_max.*sin(C*atan(B*s_Ry)); 
   %% End of Copied Section 
   
    createfigure(xi, yi, 'X, [m]', 'Y, [m]');
    fig = gcf;
    grid on;
    hold on;

    set(gcf,'Units','pixels') 
    winsize = get(gcf,'Position');
    winsize(1:2) = [0 0];
    set(gcf,'Units','normalized');
    set(gcf,'Color',[1 1 1]);
    set(gca,'NextPlot','replacechildren');
    mov = moviein(finalres,gcf,winsize);
    
    window = 0.5;
    textbox = annotation('textbox',[0.4,0.75,0.2,0.1],'BackgroundColor',[1 1 .6]);
    
    for i = 1:finalres;
        plot(xi,yi);
        ComputeCarPlotInfo(car_specs, [xi(i) yi(i)], [psi(i) delta(i)], [f_Fx(i) f_Fy(i)], [f_Rx(i) f_Ry(i)], 1);
        xlim([xi(i)-window xi(i)+window]);
        ylim([yi(i)-window yi(i)+window]);
        set(textbox,'String',[{['V = ' num2str(V(i))]} {['t = ' num2str(final_time(i))]}]);
        mov(:,i) = getframe(gcf, winsize);
    end

disp('Writing...');
mpgwrite(mov,jet,[movie_name '.mpg']);
