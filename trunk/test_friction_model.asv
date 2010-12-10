
Vx_t0  = 50;
Vy_t0  = 0;
r_t0   = 0;
psi_t0 = 0;
xi_t0  = 0;
yi_t0  = 0;

tf = 5;

time = [0:.01:tf]';
u = ones(size(time,1),1)*[-10000 -10000 0];

options = odeset('RelTol',1e-4,'AbsTol',[1e-4 1e-4 1e-5 1e-4 1e-4 1e-5]);


[sim_time y] = ode45(@car_dynamics, ...
                    [0 tf],... %output.solution.time, ...
                    [Vx_t0 Vy_t0 r_t0 psi_t0 xi_t0 yi_t0], ...
                    options, u, time);
                
                
fric_solution.time = sim_time;
fric_solution.state = y(:,1:6);
fric_solution.control = interp1(time,u,sim_time,'linear');
fric_solution.Hamiltonian = sign(sim_time);
sample = 10;
car_specs.scale = 1;
%plot_results(sample, car_specs, fric_solution);
%%

fric_solution.time = t;
fric_solution.state = states_out;
fric_solution.control = u_out;
fric_solution.Hamiltonian = sign(t);
sample = 20;
car_specs.scale = 0.5;
plot_results(sample, car_specs, fric_solution);