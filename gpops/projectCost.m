function [Mayer,Lagrange]=projectCost(solcost)

t0 = solcost.initial.time;
x0 = solcost.initial.state;
tf = solcost.terminal.time;
xf = solcost.terminal.state;
t  = solcost.time;
x  = solcost.state;
u  = solcost.control;
p  = solcost.parameter;

Mayer = xf(6)^2; %1000*tf + xf(6)^2 + (xf(3)-pi/2)^2; %zeros(size(tf)) + abs(xf(6)); %(xf(3)-pi/2)^2;  % 
Lagrange = ones(size(t));
