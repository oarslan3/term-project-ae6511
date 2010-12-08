function testComputeCarPlotInfo()

    l_F = 1.1;
    l_R = 1.6;
    lcy = 0.8;

    lax = 0.1;
    lay = 0.3;

    lwx = 0.6;
    lwy = 0.2;
    
    car_specs.scale = 1;
    
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
    
    position = [1 2];
    orientation = [0*pi/4 0*pi/180];
    
    
    f_F = [-30756 21145];
    f_R = [30756  -21145];
    
    output = ComputeCarPlotInfo(car_specs, position, orientation, f_F, f_R, 1);
    
end