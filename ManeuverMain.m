function [solution] = ManeuverMain(case_param)
    clc
    %clear all
    global CONSTANTS

    % initial and final values of states 

    %{
    Vx_t0 = 55*CONSTANTS.kmh2ms;
    Vx_tf = 100; % free

    Vy_t0 = 0;
    Vy_tf = 100; %free

    r_t0 = 0;
    r_tf = 0;

    psi_t0 = 0;
    psi_tf = pi/2;

    xi_t0 = 0;
    xi_tf = 10; % free

    yi_t0 = 0;
    yi_tf = 10; % free



    % initial and final values of controls 
    f_Fx_t0 = 0;
    f_Fx_tf = 0;

    f_Rx_t0 = 0;
    f_Rx_tf = 0;

    %delta_t0 = 0;
    %delta_tf = 0; 

    % Minimum and maximum values of the states
    % Vx_min = -250*CONSTANTS.kmh2ms;
    % Vx_max = 250*CONSTANTS.kmh2ms;

    Vy_min = -250*CONSTANTS.kmh2ms;
    Vy_max = 250*CONSTANTS.kmh2ms;

    r_min = -4*pi;
    r_max = 4*pi;

    psi_min = -pi;
    psi_max = pi;

    xi_min = 0;
    xi_max = 1000;

    yi_min = -1000;
    yi_max = 1000;

    % Minimum and maximum values of the controls
    f_Fx_min = CONSTANTS.f_Fx_min;
    f_Fx_max = CONSTANTS.f_Fx_max; 

    f_Rx_min = CONSTANTS.f_Rx_min;
    f_Rx_max = CONSTANTS.f_Rx_max;


    delta_min = -45*CONSTANTS.deg2rad;
    delta_max = 45*CONSTANTS.deg2rad;

    % Minimum and maximum values of the t0 and tf

    t0_min = 0; 
    t0_max = 0;

    tf_min = 0;
    tf_max = 10;
    %}

    % Minimum and maximum values of parameters

    param_min = [];
    param_max = [];

    path_min = [];
    path_max = [];

    event_min = [];
    event_max = [];

    duration_min = [];
    duration_max = [];

    % Phase 1 Information

    iphase = 1;
    limits(iphase).nodes = 50;
    limits(iphase).segtimes = [];
    limits(iphase).intervals = [1];
    limits(iphase).nodesperint = [15];
    limits(iphase).time.min = case_param.time_min;    %[t0_min param.time_tf_min];
    limits(iphase).time.max = case_param.time_max;   %[t0_max param.time_tf_max];

    % state(:,1) : Vx, Vy, r, psi, xi, yi
    limits(iphase).state.min(1,:) = case_param.Vx_min;      %[Vx_t0 Vx_min 0]; % param.Vx_tf_min];   %[Vx_t0_min Vx_min Vx_tf_min]
    limits(iphase).state.max(1,:) = case_param.Vx_max;      %[Vx_t0 Vx_max 10]; % param.Vx_tf_max];   %[Vx_t0_max Vx_max Vx_tf_max]

    limits(iphase).state.min(2,:) = case_param.Vy_min;      %[Vy_t0 Vy_min -20]; % param.Vy_tf_min];
    limits(iphase).state.max(2,:) = case_param.Vy_max;      %[Vy_t0 Vy_max 20]; % param.Vy_tf_max];

    limits(iphase).state.min(3,:) = case_param.r_min;       %[r_t0 r_min 0]; % param.r_tf_min]; %r_tf
    limits(iphase).state.max(3,:) = case_param.r_max;       %[r_t0 r_max 0]; %param.r_tf_max]; %r_tf

    limits(iphase).state.min(4,:) = case_param.psi_min;  	% [psi_t0 psi_min pi/2];% param.psi_tf_min];
    limits(iphase).state.max(4,:) = case_param.psi_max;     % [psi_t0 psi_max pi/2];% param.psi_tf_max];

    limits(iphase).state.min(5,:) = case_param.xi_min;      % [xi_t0 xi_min 0]; % param.xi_tf_min];
    limits(iphase).state.max(5,:) = case_param.xi_max;      % [xi_t0 xi_max 100]; % param.xi_tf_max];

    limits(iphase).state.min(6,:) = case_param.yi_min;      % [yi_t0 yi_min -100]; % param.yi_tf_min];
    limits(iphase).state.max(6,:) = case_param.yi_max;      % [yi_t0 yi_max 100]; % param.yi_tf_max];

    % control(1,:) : f_Fx, f_Rx, delta
    limits(iphase).control.min(1,:) = case_param.f_Fx_min;
    limits(iphase).control.max(1,:) = case_param.f_Fx_max;
    limits(iphase).control.min(2,:) = case_param.f_Rx_min;
    limits(iphase).control.max(2,:) = case_param.f_Rx_max;
    limits(iphase).control.min(3,:) = case_param.delta_min;
    limits(iphase).control.max(3,:) = case_param.delta_max;


    limits(iphase).parameter.min  = param_min;
    limits(iphase).parameter.max  = param_max;
    limits(iphase).path.min       = path_min;
    limits(iphase).path.max       = path_max;
    limits(iphase).event.min      = event_min;
    limits(iphase).event.max      = event_max;
    limits(iphase).duration.min   = duration_min;
    limits(iphase).duration.max   = duration_max;

    % initial guess for the solution
    guess(iphase).time            = case_param.guess_time;  % [t0_min;    0.9329;     5];% param.time];                          0.3641;
    guess(iphase).state(:,1)      = case_param.guess_Vx;    % [Vx_t0;   -0.012237;     0];%param.Vx];                        10.237;
    guess(iphase).state(:,2)      = case_param.guess_Vy;    % [Vy_t0; -0.037213;   -20];%param.Vy];                        -2.0914;
    guess(iphase).state(:,3)      = case_param.guess_r;     % [r_t0; -0.090408 ;   0];%param.r]; %r_tf                     -1.639;
    guess(iphase).state(:,4)      = case_param.guess_psi;   % [psi_t0;   -0.73758;  pi/2];%param.psi]; %psi_tf             -0.36438;
    guess(iphase).state(:,5)      = case_param.guess_xi;    % [xi_t0; 5.9345;   50];%param.xi];                            4.4744;
    guess(iphase).state(:,6)      = case_param.guess_yi;    % [yi_t0; -3.1087;  0];%param.yi];                            -1.0005;

       
      
    guess(iphase).control(:,1)    = case_param.guess_f_Fx; % [f_Fx_max;  -10000 ; f_Fx_max];%param.f_Fx];            -1000
    guess(iphase).control(:,2)    = case_param.guess_f_Rx; % [f_Rx_max; -10000; f_Rx_max];%param.f_Rx];               -1000
    guess(iphase).control(:,3)    = case_param.guess_delta; % [-45*pi/180; 0; 0];%param.delta];                         -0.7854

    guess(iphase).parameter       = [];

    setup.name  = 'maneuver';
    setup.method = 'gauss';

    %setup.method = 'radau';


    setup.funcs.cost = @ManeuverCost;
    setup.funcs.dae = @ManeuverDae;
    setup.limits = limits;
    setup.guess = guess;
    setup.linkages = [];
    setup.direction = 'increasing';
    %setup.derivatives = 'automatic-intlab';  %Intlab is the fastest
    setup.derivatives = 'automatic';  %Intlab is the fastest
    setup.controlinterp = 'lagrange';
    setup.autoscale = 'on';   %If you need autoscaling turn on.
    setup.solver = 'snopt';   %Use SNOPT as IPOPT not coded for Radau yet
    setup.mesh.grid = 'hp';   %Use local, hp, global.  Local or hp are the best.
    setup.mesh.tolerance = 1e-3;  %Maybe start with 1e-1, then 1e-2, then 1e-3 if necessary.
    setup.mesh.iteration = 5;   %If using global, set a low upper limit like 5.
    setup.mesh.on = 'yes';      %Yes to run refinement.
    setup.mesh.nodelimit = 200;  
    setup.mesh.guess = 'yes';

    output = gpops(setup);

    % val = output.solution.costate;
    
    % val = val.^2;
    
    % cost = -sum(sum(val));
    
    % global mycost myparam
   
    % if (cost < mycost)
    %    mycost = cost
    %    myparam = param
    % end
    
    
    global car_specs;
    
    sample = 30;
    plot_results(sample, car_specs, output.solution);
    
    solution = output.solution;
    
    % solution.costate

    
end