function output = ComputeCarPlotInfo(car_specs, position, orientation, f_F, f_R, plot_figure)

    %%% Car scale
    scale = car_specs.scale;
    
    %%% Chasis dimensions
    l_F = car_specs.l_F*scale;
    l_R = car_specs.l_R*scale;
    lcy = car_specs.lcy*scale;

    %%% Axel dimensions
    lax = car_specs.lax*scale;
    lay = car_specs.lay*scale;

    %%% Wheel dimensions
    lwx = car_specs.lwx*scale;
    lwy = car_specs.lwy*scale;

    %%% Position and orientation of the car
    cmx0 = position(1);
    cmy0 = position(2);
    psi = orientation(1);
    delta = orientation(2);

    %%% Transformation matrices
    Rphi = [cos(psi) sin(psi); -sin(psi) cos(psi)];
    Rdelta = [cos(delta) sin(delta); -sin(delta) cos(delta)];

    vertIndex = 1;
    faceIndex = 1;

    %%% Chasis
    localCoordsChasis = [-l_R -lcy/2;
                         -l_R  lcy/2;
                         l_F  lcy/2;
                         l_F -lcy/2];

    inertiaCoordsChasis = zeros(4,2);

    for i=1:4
        inertiaCoordsChasis(i,:) = localCoordsChasis(i,:)*Rphi + [cmx0 cmy0];
    end

    vertices = zeros(39,2);
    faceList = zeros(10,4);

    vertices(vertIndex:vertIndex+3,:) = inertiaCoordsChasis;
    faceList(faceIndex,:) = vertIndex:vertIndex+3;  
    faceIndex = faceIndex + 1;
    vertIndex = vertIndex + 4;

    %%% Bumper
    localCoordsBumper= [    l_F  lcy/2;
                         1.2*l_F  0;
                            l_F -lcy/2];

    inertiaCoordsBumper = zeros(3,2);

    for i=1:3
        inertiaCoordsBumper(i,:) = localCoordsBumper(i,:)*Rphi + [cmx0 cmy0];
    end

    vertices(vertIndex:vertIndex + 2,:) = inertiaCoordsBumper;
    faceList(faceIndex,:) = [vertIndex:vertIndex + 2 , vertIndex];  
    faceIndex = faceIndex + 1;
    vertIndex = vertIndex + 3;

    %%% Axels
    axelR0 = [l_F*0.7   lcy/2 + lay/2; ...  
              l_F*0.7  -lcy/2 - lay/2; ...
              -l_R*0.7   lcy/2 + lay/2; ...
              -l_R*0.7  -lcy/2 - lay/2];

    localCoordsAxel = [-lax/2 -lay/2;
                       -lax/2  lay/2;
                        lax/2  lay/2;
                        lax/2 -lay/2];

    inertiaCoordsAxel = zeros(4,2);

    for k = 1:4
        for i = 1:4
            inertiaCoordsAxel(i,:) = (localCoordsAxel(i,:) + axelR0(k,:))*Rphi + [cmx0 cmy0];
        end

        vertices(vertIndex:vertIndex + 3,:) = inertiaCoordsAxel;
        faceList(faceIndex,:) = vertIndex:vertIndex + 3;  
        faceIndex = faceIndex + 1;
        vertIndex = vertIndex + 4;
    end   

    %%% Wheel
    wheelR0 = [ axelR0(1,1) axelR0(1,2) + lay/2; ...
                axelR0(2,1) axelR0(2,2) - lay/2; ...
                axelR0(3,1) axelR0(3,2) + lay/2; ...
                axelR0(4,1) axelR0(4,2) - lay/2];

    localCoordsWheel = [-lwx/2 -lwy/2;
                        -lwx/2  lwy/2;
                         lwx/2  lwy/2;
                         lwx/2 -lwy/2];

    inertiaCoordsWheel = zeros(4,2);

    global CONSTANTS;
    

    
    for k = 1:4
        for i = 1:4

            if (k < 3 )   %%% Change the orientation of the front wheels
                inertiaCoordsWheel(i,:) = (localCoordsWheel(i,:)*Rdelta + wheelR0(k,:))*Rphi + [cmx0 cmy0];
            else
                inertiaCoordsWheel(i,:) = (localCoordsWheel(i,:) + wheelR0(k,:))*Rphi + [cmx0 cmy0];
            end
        end

        vertices(vertIndex:vertIndex + 3,:) = inertiaCoordsWheel;
        faceList(faceIndex,:) = vertIndex:vertIndex+3; 
        faceIndex = faceIndex + 1;
        vertIndex = vertIndex + 4;
       
    end
    
    
    %
    f_Fx = f_F(1);
    f_Fy = f_F(2);
    
    f_Rx = f_R(1);
    f_Ry = f_R(2);
    
    
    norm_f_Fx = f_Fx/abs(CONSTANTS.f_Fx_min)*scale;
    norm_f_Fy = f_Fy/abs(CONSTANTS.f_Fx_min)*scale;
    
    norm_f_Rx = f_Rx/abs(CONSTANTS.f_Fx_min)*scale;
    norm_f_Ry = f_Ry/abs(CONSTANTS.f_Fx_min)*scale;
    
    inertialCoordsXForces = zeros (4,4);
    inertialCoordsYForces = zeros (4,4);
    
    for k =1:4      % k : wheel number
        if (k < 3)  % front wheels
            
            % force on x direction, 
            % coordinate of tail of the arrow 
            inertialCoordsXForces (k,1:2) = (wheelR0(k,:))*Rphi + [cmx0 cmy0];
            % coordinate of head of arrow
            inertialCoordsXForces (k,3:4) = ([norm_f_Fx 0]*Rdelta + wheelR0(k,:))*Rphi + [cmx0 cmy0];
            
            
            % force on y direction,
            % coordinate of tail of the arrow 
            inertialCoordsYForces (k,1:2) = (wheelR0(k,:))*Rphi + [cmx0 cmy0];
            
            % coordinate of head of arrow
            inertialCoordsYForces (k,3:4) = ([0 norm_f_Fy]*Rdelta + wheelR0(k,:))*Rphi + [cmx0 cmy0];
            
        else % rear wheels

            % force on x direction
            % coordinate of tail of the arrow 
            inertialCoordsXForces (k,1:2) = (wheelR0(k,:))*Rphi + [cmx0 cmy0];
            % coordinate of head of arrow
            inertialCoordsXForces (k,3:4) = ([norm_f_Rx 0] + wheelR0(k,:))*Rphi + [cmx0 cmy0];
            
            
            % force on y direction,
            % coordinate of tail of the arrow 
            inertialCoordsYForces (k,1:2) = (wheelR0(k,:))*Rphi + [cmx0 cmy0];
            
            % coordinate of head of arrow
            inertialCoordsYForces (k,3:4) = ([0 norm_f_Ry] + wheelR0(k,:))*Rphi + [cmx0 cmy0];
        end
    end
    
    
    
   
    
    cdata = [0.6   0 0.7 0.7 0.7 0.7 0.2 0.2 0.2 0.2;
             0     0 0.7 0.7 0.7 0.7 0.2 0.2 0.2 0.2;
             0   0.5 0.7 0.7 0.7 0.7 0.2 0.2 0.2 0.2]';

    if (plot_figure)     
        p = patch('Faces',faceList,'Vertices', vertices,'FaceColor','flat','FaceVertexCData', cdata);
        grid on;    
        hold on;
        
        plot(cmx0, cmy0,'MarkerFaceColor',[0 0 0], ...
                        'MarkerEdgeColor',[0 0 0], ...
                        'Marker','o', ...
                        'LineWidth',3*scale, ...
                        'Color',[0 0 0]);
        hold off;

        for k = 1:4
            if (k < 3) % front wheel
                if (norm_f_Fx <= 0) % braking mode
                    color = 'r';
                else                % driving mode
                    color = 'g';
                end
            else
                if (norm_f_Rx <= 0) % braking mode
                    color = 'r';
                else                % driving mode
                    color = 'g';
                end
            end
            
            % x forces
            point_tail = inertialCoordsXForces (k,1:2);
            point_head = inertialCoordsXForces (k,3:4);
            
            if (norm(point_head - point_tail )> 0.01) 
                p = ARROW(point_tail, point_head, scale, 'BaseAngle',60);
                set(p, 'FaceColor', color);
                set(p, 'EdgeColor', color);
                set(p, 'LineWidth', 2.5*scale);
            end
            
            % y forces
            point_tail = inertialCoordsYForces (k,1:2);
            point_head = inertialCoordsYForces (k,3:4);
            
            if (norm(point_head - point_tail )> 0.01) 
                p = ARROW(point_tail, point_head, scale, 'BaseAngle',60);
                set(p, 'FaceColor', 'y');
                set(p, 'EdgeColor', 'y');
                set(p, 'LineWidth', 2.5*scale);
            end
            
        end
        
        %arrrow fixlimits
        
        axis('equal')
    end
    
    output.vertices = vertices;
    output.faceList = faceList;
    output.cdata = cdata;    
end