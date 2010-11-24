clear setup limits guess CONSTANTS

global CONSTANTS

figure;
sn = 1;

for x0 = [-4 -3 2]
    xf = 1;
    tf = 5;

    xmin = -20;
    xmax =  20;
    umin = -1;
    umax =  1;
    t0min = 0;
    t0max = 0;
    tfmin = tf;
    tfmax = tf;

    % Phase 1 Information
    iphase = 1;
    limits(iphase).nodes           = 50;
    limits(iphase).time.min        = [t0min tfmin];
    limits(iphase).time.max        = [t0max tfmax];
    limits(iphase).state.min       = [x0 xmin xf];
    limits(iphase).state.max       = [x0 xmax xf];
    limits(iphase).control.min    = -1;
    limits(iphase).control.max    = 1;
    limits(iphase).parameter.min  = [];
    limits(iphase).parameter.max  = [];
    limits(iphase).path.min       = [];
    limits(iphase).path.max       = [];
    limits(iphase).event.min      = [];
    limits(iphase).event.max      = [];
    limits(iphase).duration.min    = [];
    limits(iphase).duration.max    = [];
    guess(iphase).time             = [t0min; tfmax];
    guess(iphase).state           = [x0; x0];
    guess(iphase).control         = [umin; umin];
    guess(iphase).parameter       = [];

    linkages = [];
    setup.name  = 'P4';
    setup.funcs.cost = 'P4Cost';
    setup.funcs.dae = 'P4Dae';
    setup.limits = limits;
    setup.guess = guess;
    setup.linkages = linkages;
    setup.derivatives = 'complex';
    setup.direction = 'increasing';
    setup.autoscale = 'off';

    output = gpops(setup);
    solution = output.solution;
    
    subplot(1,3,sn);
    plot(solution.time, solution.control);
    title(['x_0 = ' num2str(x0)]);
    xlabel('Time (s)');
    ylabel('Control u');
    drawnow;
    sn = sn + 1;
end