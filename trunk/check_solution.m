function check_solution(case_param, solution)

    [tsim y] = ode45_sim (case_param, solution.control, solution.time);
    
    
    other_solution.time = tsim;
    other_solution.state = y;
    
    other_solution.control = interp1(solution.time,solution.control,tsim,'linear');

    global car_specs;
    
    len = size(tsim,1);
    
    dy = zeros(len, 6);
    
    for i = 1:len
    
        dy(i,:) = car_dynamics(tsim(i), y(i,:), other_solution.control, tsim)';
    
    end
    
    other_solution.costate = interp1(solution.time,solution.costate,tsim,'linear');
    
    other_solution.Hamiltonian = 1 + other_solution.costate(:,1).*dy(:,1) ...
                                   + other_solution.costate(:,2).*dy(:,2)...
                                   + other_solution.costate(:,3).*dy(:,3)...
                                   + other_solution.costate(:,4).*dy(:,4)...
                                   + other_solution.costate(:,5).*dy(:,5)....
                                   + other_solution.costate(:,6).*dy(:,6);
    
    sample = 30;
    plot_results(sample, car_specs, other_solution);
end