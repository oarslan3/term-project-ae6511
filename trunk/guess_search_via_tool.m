function [x,fval,exitflag,output,population,score] = guess_search_via_tool(nvars,lb,ub)
% This is an auto generated M-file from Optimization Tool.

% Start with the default options
options = gaoptimset;
% Modify options setting
options = gaoptimset(options,'Display', 'off');
options = gaoptimset(options,'PlotFcns', {  @gaplotbestindiv @gaplotscores });
[x,fval,exitflag,output,population,score] = ...
ga(@ManeuverMain,nvars,[],[],[],[],lb,ub,[],options);
