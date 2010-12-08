% initialization

global CONSTANTS

CONSTANTS.g = 9.82;
CONSTANTS.m = 1450;
CONSTANTS.Iz = 2740;
CONSTANTS.l_F = 1.1;
CONSTANTS.l_R = 1.6;
CONSTANTS.B = 7;
CONSTANTS.C = 1.4;
CONSTANTS.mu = 0.5;
CONSTANTS.f_Fz = CONSTANTS.m*CONSTANTS.g*CONSTANTS.l_R*(CONSTANTS.l_F + CONSTANTS.l_R);
CONSTANTS.f_Rz = CONSTANTS.m*CONSTANTS.g*CONSTANTS.l_F*(CONSTANTS.l_F + CONSTANTS.l_R);

CONSTANTS.kmh2ms = 10/36;
CONSTANTS.ms2kmh = 36/10;
CONSTANTS.deg2rad = pi/180;
CONSTANTS.rad2deg = 180/pi;

% Minimum and maximum values of the controls

CONSTANTS.f_Fx_min = -CONSTANTS.mu*CONSTANTS.f_Fz + 2000;
CONSTANTS.f_Fx_max = 0; 

CONSTANTS.f_Rx_min = -CONSTANTS.mu*CONSTANTS.f_Rz + 2000;
CONSTANTS.f_Rx_max = CONSTANTS.mu*CONSTANTS.f_Rz - 2000;

global car_specs;
%%% Car specifications
car_specs.scale = 0.1;

%%% Chasis dimensions

car_specs.l_F = CONSTANTS.l_F;
car_specs.l_R = CONSTANTS.l_R; 
car_specs.lcy = 0.8;

%%% Axel dimensions
car_specs.lax = 0.1;
car_specs.lay = 0.3;

%%% Wheel dimensions
car_specs.lwx = 0.6;
car_specs.lwy = 0.2;