function daeout = projectDae(soldae)

global CONSTANTS

t = soldae.time;
x = soldae.state;
u = soldae.control;
p = soldae.parameter;

Vx = x(:,1);
Vy = x(:,2);
psi = x(:,3);
r = x(:,4);
X = x(:,5);
Y = x(:,6);

fFx = u(:,1);
fRx = u(:,2);
delta = u(:,3);

fFy = 

Vxdot = 1/CONSTANTS.m*(fFx*cos(delta) - fFy*sin(delta) + fRx) - Vy*r;
Vydot =
psidot =
rdot =
Xdot =
Ydot =



daeout = [Vxdot Vydot psidot rdot Xdot Ydot];
