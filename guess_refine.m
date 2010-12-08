param.time  = [            1.7];
param.Vx    = [         10*10/36];
param.Vy    = [         0 ];
param.r     = [        1.9*pi];
param.psi   = [           0*pi/180];
param.xi    = [         20];
param.yi    = [           -3];

param.f_Fx  = [         0];
param.f_Rx  = [       CONSTANTS.f_Rx_max];
param.delta = [      0];

param.time_tf_min  = 0;% 0;
param.time_tf_max  = 20;% 2;

param.Vx_tf_min    = 0; % 0;
param.Vx_tf_max    = 100*10/36; % 100*10/36;

param.Vy_tf_min    = -100*10/36; % 0;
param.Vy_tf_max    = 0; % -100*10/36;

param.r_tf_min     =  -2*pi;
param.r_tf_max     =  2*pi;

param.psi_tf_min   =  -90*pi/180;
param.psi_tf_max   =  90*pi/180;

param.xi_tf_min    =  0;
param.xi_tf_max    = 50;

param.yi_tf_min    =  -5;
param.yi_tf_max    =  0;

param.f_Fx_tf_min  = CONSTANTS.f_Fx_min; % CONSTANTS.f_Fx_min;
param.f_Fx_tf_max  = 0; % 0;
 
param.f_Rx_tf_min  = CONSTANTS.f_Rx_min; % CONSTANTS.f_Rx_min;
param.f_Rx_tf_max  = CONSTANTS.f_Rx_max; % CONSTANTS.f_Rx_max;

param.delta_tf_min =  -45*pi/180;
param.delta_tf_max =  45*pi/180;

%% Case1: no penalty for lateral deviation

kmh2ms = CONSTANTS.kmh2ms;
deg2rad = CONSTANTS.deg2rad;

case1.time_min = [0 0]; 
case1.time_max = [0 25];

case1.Vx_min = [55*kmh2ms -250*kmh2ms -250*kmh2ms]; % Vx(t0): fixed   
case1.Vx_max = [55*kmh2ms  250*kmh2ms  250*kmh2ms]; % Vx(tf): free

case1.Vy_min = [0 -250*kmh2ms -250*kmh2ms];         % Vy(t0): fixed 
case1.Vy_max = [0  250*kmh2ms  250*kmh2ms];         % Vy(tf): free

case1.r_min = [0 -4*pi 0];
case1.r_max = [0  4*pi 0];

case1.psi_min = [0 -pi  pi/2];
case1.psi_max = [0  pi  pi/2];

case1.xi_min = [0   0   0];   % xi(t0): fixed
case1.xi_max = [0 200 200];   % xi(tf): free


case1.yi_min = [0 -200 -200]; % yi(t0): fixed
case1.yi_max = [0  200  200]; % yi(tf): free

case1.f_Fx_min = CONSTANTS.f_Fx_min;
case1.f_Fx_max = CONSTANTS.f_Fx_max;
case1.f_Rx_min = CONSTANTS.f_Rx_min;
case1.f_Rx_max = CONSTANTS.f_Rx_max;
case1.delta_min = -45*deg2rad;
case1.delta_max =  45*deg2rad;

case1.guess_time    = [0; 5];
case1.guess_Vx      = [55*kmh2ms; 0];
case1.guess_Vy      = [0; -10];
case1.guess_r       = [0; 0];
case1.guess_psi     = [0; pi/2];
case1.guess_xi      = [0; 20];
case1.guess_yi      = [0; 5];

case1.guess_f_Fx    = [CONSTANTS.f_Fx_min; 0];
case1.guess_f_Rx    = [CONSTANTS.f_Rx_min; 0];
case1.guess_delta   = [45*deg2rad; 0];



%% Case2: to hit the switching surface

kmh2ms = CONSTANTS.kmh2ms;
deg2rad = CONSTANTS.deg2rad;

case2.time_min = [0 0]; 
case2.time_max = [0 30];

case2.Vx_min = [55*kmh2ms 0 -250*kmh2ms]; % Vx(t0): fixed   
case2.Vx_max = [55*kmh2ms  250*kmh2ms  250*kmh2ms]; % Vx(tf): free

case2.Vy_min = [0 -250*kmh2ms -250*kmh2ms];         % Vy(t0): fixed 
case2.Vy_max = [0  250*kmh2ms  250*kmh2ms];         % Vy(tf): free

case2.r_min = [0 -4*pi 0];
case2.r_max = [0  4*pi 0];

case2.psi_min = [0 -pi  pi/2];
case2.psi_max = [0  pi  pi/2];

case2.xi_min = [0   0   0];   % xi(t0): fixed
case2.xi_max = [0 200 200];   % xi(tf): free


case2.yi_min = [0 -200 -200]; % yi(t0): fixed
case2.yi_max = [0  0.1  0.1]; % yi(tf): free

case2.f_Fx_min = CONSTANTS.f_Fx_min;
case2.f_Fx_max = CONSTANTS.f_Fx_max;
case2.f_Rx_min = CONSTANTS.f_Rx_min;
case2.f_Rx_max = CONSTANTS.f_Rx_max;
case2.delta_min = -45*deg2rad;
case2.delta_max =  45*deg2rad;

case2.guess_time    = [0;  1.205557515315867;         4   ];
case2.guess_Vx      = [55*kmh2ms;  8.82841373613096;  3];
case2.guess_Vy      = [0; -1.27477265906489;  -4];
case2.guess_r       = [0;  2.13718136707670;  0];
case2.guess_psi     = [0;  -0.2;  pi/2];
case2.guess_xi      = [0;  2.57062541130178;  20];
case2.guess_yi      = [0;  -1.834898507368333;  0];




case2.guess_f_Fx    = [CONSTANTS.f_Fx_min;    -13511.6076366385; -15000;];
case2.guess_f_Rx    = [CONSTANTS.f_Rx_min;   -19144.9150000000; -20000];
case2.guess_delta   = [45*deg2rad;           -0.785398163397448; 45*deg2rad];





