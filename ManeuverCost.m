%--------------------------------------
% BEGIN: function optimalprojectCost.m
%--------------------------------------
function [Mayer, Lagrange] = ManeuverCost(solution)
    
    tf = solution.terminal.time;
    t = solution.time;
    
    xi = solution.state(:,5);
    yi = solution.state(:,6);
    
    f_Fx = solution.control(:,1);
    f_Rx = solution.control(:,2);
    
    val = find( f_Rx.*f_Fx < 0);
    
    gain = 1;
    
    %if (~isempty(val))
    %    gain = 1000;
    %end
    
 
    
    Mayer =  2*yi(end).^2;
    
    
    Lagrange = ones(size(t));
end