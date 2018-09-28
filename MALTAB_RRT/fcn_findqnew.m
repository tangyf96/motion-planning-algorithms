function [qnewx,qnewy] = fcn_findqnew(vertices,q_rand,EPSILON)
%% This function is to find the new state in the RRT algorithm       
%
%
%
%
%

%% find the new states from the nearest neighbors in the direction to the new states 

% Check if input arguments
if nargin < 2 || nargin > 3
    error('incorrect number of arguments');
end

angle = atan2((q_rand(2)-vertices.y),(q_rand(1)-vertices.x));
qnewx = vertices.x + EPSILON*cos(angle);
qnewy = vertices.y + EPSILON*sin(angle);
end