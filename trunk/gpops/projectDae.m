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

% VF = sign(Vx.*cos(delta)+Vy.*sin(delta));
% VR = sign(Vx);
% fFx(fFx < 0) = fFx(fFx < 0) .* VF(fFx < 0);
% fRx(fRx < 0) = fRx(fRx < 0) .* VR(fRx < 0);

fFymax = sqrt((CONSTANTS.mu*CONSTANTS.fFz)^2-fFx.^2);
fRymax = sqrt((CONSTANTS.mu*CONSTANTS.fRz)^2-fRx.^2);

sFy = ((Vy+r*CONSTANTS.lF).*cos(delta)-Vx.*sin(delta)) ./ ...
        ((Vy+r*CONSTANTS.lF).*sin(delta)+Vx.*cos(delta));
sRy = (Vy-r*CONSTANTS.lR)./Vx; 
    
% Note minus sign is differnt than equations in project description
fFy = -fFymax.*sin(CONSTANTS.C*atan(CONSTANTS.B*sFy));
fRy = -fRymax.*sin(CONSTANTS.C*atan(CONSTANTS.B*sRy));

Vxdot = 1/CONSTANTS.m*(fFx.*cos(delta) - fFy.*sin(delta) + fRx) - Vy.*r;
Vydot = 1/CONSTANTS.m*(fFx.*sin(delta) - fFy.*cos(delta) + fRy) - Vx.*r;
psidot = r;
rdot = 1/CONSTANTS.Iz*(CONSTANTS.lF*fFy.*cos(delta) + ...
    CONSTANTS.lF*fFx.*sin(delta) - fRy*CONSTANTS.lR);
Xdot = Vx.*cos(psi) - Vy.*sin(psi);
Ydot = Vx.*sin(psi) + Vy.*cos(psi);

daeout = [Vxdot Vydot psidot rdot Xdot Ydot];
