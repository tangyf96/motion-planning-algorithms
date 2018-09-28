function [out] = fcn_collisioncheck(obstacle,xt,yt)
% coliisioncheck function\
% Date: 10/23/2017
% IVSG at Penn State 
% [out] = collisioncheck(q_near,q_new,obstacle)
% [out] is a boolean value for checking whether two line segments have an
% intersection
% [inputs]
% Obstacle is a N-by-2 Matrix, the first column is for the x coordinates
% for the obstacles, and the second column is for the y coordinates.
% q_near [a]  q_new [b] obstacle previous [c]  obstacle next [d]
% reference [https://stackoverflow.com/questions/29485025/point-in-polygon-algorithm-explanation]
% reference [http://mathhelpforum.com/geometry/42487-point-inside-outside-polygon.html]
% intersection
% vector experssion of r is q_new - q_near
% vector experssion of line segment previous corner vertex and current
% corner vertex 
% Example 
% Obstacle = []
% 
%% check the number of input arguments
    if nargin ~= 3 || nargin==1
        error('Incorrect numbers of input arguments')
    end
%% check NaN value in obstacles, if there is a NaN value, delete it
    if isnan(obstacle) == 1
        obstacle(find(isnan(obstacle))) = [];
    end
%% check to see whether obstacle is empty
    if isempty(obstacle) == 1
        error('the obstacle array is empty, please correct the array')
    end
    rows = size(obstacle);  
    npoints = rows(1);
    xold = obstacle(npoints,1);
    yold = obstacle(npoints,2);
    out = 0;
%% check whether an intersection occurs
% condition (xnew < xt) == (xt <= xold) checks whether the Y-parallel line
% from point (xt,yt) meets the edges of the obstacles, the second condition
% checks whether the point (xt,yt) sits at the proper side 
for i = 1:1:npoints
    xnew = obstacle(i,1);
    ynew = obstacle(i,2);
    if (xnew > xold)
        x1 = xold;
        x2 = xnew;
        y1 = yold;
        y2 = ynew;
    else
        x1 = xnew;
        x2 = xold;
        y1 = ynew;
        y2 = yold;
    end
    if ((xnew < xt) == (xt <= xold) && (yt - y1)*(x2 - x1)-(y2 - y1)*(xt-x1) < 0)
        out = ~out;
    end
    xold = xnew;
    yold = ynew;
%% NOT WORKING CODE
%     r = (q_new-q_near);                    % vector one 
%     s = (obstacle(i,:)-obstacle(i-1,:));   % vector two 
%     cross = r(1)*s(2)-r(2)*s(1);           % 2d cross product
%     % scalar value t on the line segment [a b]
%     t = ((obstacle(i-1,1)-q_near(1))*s(2)-(obstacle(i-1,2)-q_near(2))*s(1))/cross; 
%     % scalar value u on the line segment [c d]
%     u = ((obstacle(i-1,1)-q_near(1))*r(2)-(obstacle(i-1,2)-q_near(2))*r(1))/cross; 
%     if (-1 <= u && u <= 1 && -1 <= t && t <= 1) % [if] statement determining intersection
%         out = true;                        % intersect occurs
%     else 
%         out = false;                       % no intersect
%     end   
end
end
