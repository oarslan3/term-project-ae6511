clear setup limits guess CONSTANTS

global CONSTANTS

CONSTANTS.g = 9.80665;
CONSTANTS.m = 1450;
CONSTANTS.Iz = 2740;
CONSTANTS.lF = 1.1;
CONSTANTS.lR = 1.6;
CONSTANTS.B = 7;
CONSTANTS.C = 1.4;
CONSTANTS.mu = 0.5;

CONSTANTS.fFz = CONSTANTS.m*CONSTANTS.g*CONSTANTS.lR/(CONSTANTS.lF+CONSTANTS.lR);
CONSTANTS.fRz = CONSTANTS.m*CONSTANTS.g*CONSTANTS.lF/(CONSTANTS.lF+CONSTANTS.lR);

CONSTANTS.fFxmax = CONSTANTS.mu*CONSTANTS.fFz;
CONSTANTS.fRxmax = CONSTANTS.mu*CONSTANTS.fRz;



frontForce = solution.control(:,1);
frontForce(frontForce > 0) = 0;

solution1 = solution;

solution1.control(:,3) = solution.control(:,2);

solution1.control(:,1) = frontForce*CONSTANTS.fFxmax;

solution1.control(:,2) = solution.control(:,1)*CONSTANTS.fRxmax;


