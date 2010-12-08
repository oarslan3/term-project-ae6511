global CONSTANTS

CONSTANTS.mu = 0.5;
CONSTANTS.f_Fz = CONSTANTS.m*CONSTANTS.g*CONSTANTS.l_R*(CONSTANTS.l_F + CONSTANTS.l_R);
CONSTANTS.f_Rz = CONSTANTS.m*CONSTANTS.g*CONSTANTS.l_F*(CONSTANTS.l_F + CONSTANTS.l_R);
f_Fx_min = -CONSTANTS.mu*CONSTANTS.f_Fz;
f_Fx_max = 0;

f_Rx_min = -CONSTANTS.mu*CONSTANTS.f_Rz;
f_Rx_max = CONSTANTS.mu*CONSTANTS.f_Rz;

ObjectiveFunction = @ManeuverMain;
nvars = 8;    % Number of variables
LB = [0 -100*10/36 -100*10/36 0 -100 f_Fx_min f_Rx_min -45*pi/180];   % Lower bound
UB = [10 100*10/36  100*10/36 100 100 f_Fx_max f_Rx_max 45*pi/180];  % Upper bound

global mycost myparam

[paramstar,fval] = ga(ObjectiveFunction,nvars,[],[],[],[],LB,UB)