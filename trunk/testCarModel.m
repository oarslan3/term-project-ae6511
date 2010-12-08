%Simulation parameters
% param.m = 1450;
% param.Iz = 2740;
% param.l_F = 1.1;
% param.l_R = 1.6;
% param.B = 7;
% param.C = 1.4;
% param.mu = 0.5;
% param.delta_max = pi/45;

clear all;

m = 1450;
Iz = 2740;
l_F = 1.1;
l_R = 1.6;
B = 7;
C = 1.4;
mu = 0.5;
delta_max = pi/45;
g = 9.82;

%%% Car plotting parameters

lcy = 0.8;

lax = 0.1;
lay = 0.3;

lwx = 0.6;
lwy = 0.2;


car_specs.scale = 0.2;

%%% Chasis dimensions

car_specs.l_F = l_F;
car_specs.l_R = l_R; 
car_specs.lcy = lcy;

%%% Axel dimensions
car_specs.lax = lax;
car_specs.lay = lay;

%%% Wheel dimensions
car_specs.lwx = lwx;
car_specs.lwy = lwy;
        

    
%%% X0 : initial value of the states (body frame)      
Vx0 = 45/36*10;
Vy0 = 0;
r0 = 0;
psi0 = 0;
X0 = [Vx0 Vy0 r0 psi0]'; 

% Xin0 : initial position of the vehicle (inertial frame)
posX0 = 0;
posY0 = 0;
    
Xin0 = [posX0 posY0]';


% u0 : initial value of the controller
f_Fx0 = -100;
f_Rx0 = -200;
delta0 = 5/180*pi;
u0 = [f_Fx0, f_Rx0, delta0]'; 

% initial values of the forces
f_F0 = [0 m*g*l_R*(l_F + l_R)]';
f_R0 = [0 m*g*l_R*(l_F + l_R)]';
