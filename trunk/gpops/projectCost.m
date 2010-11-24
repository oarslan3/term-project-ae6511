function [Mayer,Lagrange]=projectCost(solcost);

t0 = solcost.initial.time;
x0 = solcost.initial.state;
tf = solcost.terminal.time;
xf = solcost.terminal.state;
t  = solcost.time;
x  = solcost.state;
u  = solcost.control;
p  = solcost.parameter;

Mayer = zeros(size(t0));
Lagrange = 1/2*x.^2;
