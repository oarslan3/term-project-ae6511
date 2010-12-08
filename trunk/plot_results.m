function plot_results(sample, car_specs, solution)

%                 phase: 1
%                  time: [151x1 double]
%                 state: [151x6 double]
%               control: [151x3 double]
%             parameter: [0x1 double]
%               costate: [151x6 double]
%              pathmult: []
%           Hamiltonian: [151x1 double]
%            Mayer_cost: 10.0000
%         Lagrange_cost: 0
%    control_pontryagin: [151x3 double]
%                 timeL: [501x1 double]
%               stateL: [501x6 double]
                

    global CONSTANTS;
    
    t = solution.time;
    len = size(t,1);
    
    X = solution.state;
    u = solution.control;
    
    Vx  = X(:,1);
    Vy  = X(:,2);
    r   = X(:,3);
    psi = X(:,4);
    xi  = X(:,5);
    yi  = X(:,6);
    
    f_Fx = u(:,1);
    f_Rx = u(:,2);
    delta = u(:,3);
    
    %Auxiliar variables
    
    V = sqrt(Vx.^2+Vy.^2);
    beta = atan2(Vy,Vx);

    f_Fy_max = sqrt( (CONSTANTS.mu*CONSTANTS.f_Fz).^2 - f_Fx.^2 );
    s_Fy = (V.*sin(beta-delta)+r*CONSTANTS.l_F.*cos(delta))./(V.*cos(beta-delta)+r*CONSTANTS.l_F.*sin(delta));
    f_Fy = -f_Fy_max.*sin(CONSTANTS.C*atan(CONSTANTS.B*s_Fy));

    f_Ry_max = sqrt( (CONSTANTS.mu*CONSTANTS.f_Rz).^2 - f_Rx.^2 );
    s_Ry =(V.*sin(beta)-r*CONSTANTS.l_R)./(V.*cos(beta));
    f_Ry = -f_Ry_max.*sin(CONSTANTS.C*atan(CONSTANTS.B*s_Ry));
    
    
    
    % Question 6: Plot the vehicle trajectory in the x - y plane.
    
    
    createfigure(xi, yi, 'X, [m]', 'Y, [m]');
    grid on;
    hold on;
  
    for k = 1:floor(len/sample):len

        output = ComputeCarPlotInfo(car_specs, [xi(k), yi(k)], [psi(k), delta(k)],[f_Fx(k) f_Fy(k)], [f_Rx(k) f_Ry(k)], 1);

    end
%     
%     
    
    %%{
    % Question 7: Plot the optimal control inputs, namely, fFx, fRx and delta.
    
    
    %axes1 = subplot(1,3,1); 
    %createfigure(t, f_Fx, 't, [s]', 'f_{Fx}, [N]', axes1);
    createfigure(t, f_Fx, 't, [s]', 'f_{Fx}, [N]');
    hold on;
    grid on;
    plot(t, (-CONSTANTS.mu*CONSTANTS.f_Fz)*ones(len,1),'Color',[1 0 0]);
    plot(t, 0,'Color',[1 0 0]);
    
    %axes1 = subplot(1,3,2);
    %createfigure(t, f_Rx, 't, [s]', 'f_{Rx}, [N]', axes1);
    createfigure(t, f_Rx, 't, [s]', 'f_{Rx}, [N]');
    hold on;
    grid on;
    plot(t, (-CONSTANTS.mu*CONSTANTS.f_Rz)*ones(len,1),'Color',[1 0 0]);
    plot(t, CONSTANTS.mu*CONSTANTS.f_Rz*ones(len,1),'Color',[1 0 0]);
    
    %axes1 = subplot(1,3,3);
    %createfigure(t, delta*CONSTANTS.rad2deg, 't, [s]', '\delta, [deg]', axes1);
    createfigure(t, delta*CONSTANTS.rad2deg, 't, [s]', '\delta, [deg]');
    hold on;
    grid on;
    plot(t,-45*ones(len,1),'Color',[1 0 0]);
    plot(t, 45*ones(len,1),'Color',[1 0 0]);
    
    %}
    
    % 8. Plot the lateral forces at the front and rear tires, as well as the total force at the front and
    % rear tire.
    
    

    createfigure(t, f_Fy, 't, [s]', 'Forces (front tire), [N]');
    % hold on;
    % grid on;
    % plot(t, sqrt(f_Fy.^2 + f_Fx.^2));
    
    createfigure(t, f_Ry, 't, [s]', 'Forces (rear tire), [N]');
    % hold on;
    % grid on;
    % plot(t, sqrt(f_Ry.^2 + f_Rx.^2));
    
    % 9. Plot the time histories of sFy and sRy.
    %{
    createfigure(t, s_Fy, 't, [s]', 's_{Fy}');
    grid on;
    
    createfigure(t, s_Ry, 't, [s]', 's_{Ry}');
    grid on;
    
    %}
    
    %{
    % 10. Plot the time histories of the states and co-states of the optimal trajectory.
    
    Vx = solution.state(:,1);
    Vy = solution.state(:,2);
    r = solution.state(:,3);
    psi = solution.state(:,4);
    
    lamda1 = solution.costate(:,1);
    lamda2 = solution.costate(:,2);
    lamda3 = solution.costate(:,3);
    lamda4 = solution.costate(:,4);
    
    axes1 = createfigure(t, Vx, 't, [s]', 'V_{x}, [m/s]');
    hold on;
    grid on;
    plot(t, lamda1);
    
    axes1 = createfigure(t, Vy, 't, [s]', 'V_{y}, [m/s]');
    hold on;
    grid on;
    plot(t, lamda2);
    
    axes1 = createfigure(t, r*CONSTANTS.rad2deg, 't, [s]', 'r, [deg/s]');
    hold on;
    grid on;
    plot(t, lamda3);
    
    axes1 = createfigure(t, psi*CONSTANTS.rad2deg, 't, [s]', '\Psi, [deg]');
    hold on;
    grid on;
    plot(t, lamda4);
    
    %}
    
    % 11. Plot the time history of the Hamiltonian.
    hold off;
    lh = size(solution.Hamiltonian,1);
    
    createfigure(t(1:lh), solution.Hamiltonian, 't, [s]', 'Hamiltonian');
    grid on;


    %%

    
   
    
    
    % 12. Add any schematics and plot any additional variables of the problem that you feel are nec-
    % essary in order to make your case that the obtained solution is a (at least locally optimal)
    % solution for your problem formulation.
    
    % 13. Repeat the solution for the following cases of friction coefficient and initial velocity
    % (a) ? = 0.2 and V (0) = 25 Km/hr
    % (b) ? = 0.8 and V (0) = 75 Km/hr
    % 14. (Extra bonus) Provide an animation of the moving vehicle of each case.

    %{

    %%
    subplot(1,2,1);

    plot(t, position_out(:,1));
    xlabel({'t, [s]'});
    ylabel({'X, [m]'});

    %createfigure(t, position_out(:,1), 't [s]', 'X, [m]');

    subplot(1,2,2);
    plot(t, position_out(:,2));
    xlabel({'t, [s]'});
    ylabel({'Y, [m]'});

    %createfigure(t, position_out(:,2), 't [s]', 'Y, [m]');

    %%
    createfigure(t, velocity_out(:,1), 't [s]', 'V_{x}, [m/s]');
    createfigure(t, velocity_out(:,1)/10*36, 't [s]', 'V_{x}, [km/h]');

    createfigure(t, velocity_out(:,2), 't [s]', 'V_{y}, [m/s]');
    createfigure(t, velocity_out(:,2)/10*36, 't [s]', 'V_{y}, [km/h]');

    createfigure(t, r_out, 't [s]', 'r, [rad/s]');
    createfigure(t, r_out*180/pi, 't [s]', 'r, [deg/s]');

    createfigure(t, orientation_out(:,1), 't [s]', '\Psi, [rad]');
    createfigure(t, orientation_out(:,1)*180/pi, 't [s]', '\Psi, [deg]');

    %% 

    % u : [f_Fx; f_Rx; delta]; 

    createfigure(t, u_out(:,1), 't [s]', 'f_{Fx}, [N]');
    createfigure(t, u_out(:,2), 't [s]', 'f_{Rx}, [N]');

    createfigure(t, u_out(:,3), 't [s]', '\delta, [rad]');
    createfigure(t, u_out(:,3)*180/pi, 't [s]', '\delta, [deg]');

    %}
end