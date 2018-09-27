%% rrt implementation with adding vertices and edges only to the map. 
% This testing file test Rapidly-randomized Trees algorithms in an arbitrary start
% and end goal. 
% The randomized tree would explore the entire map so that a path could be
% obtained from any pairs of starting and end points.
%
% INPUTS --- [X_G, Y_G, EPSILON, INIT, GOAL] 
% X_G: The x coordinate of the wheelchair in the global coordination, it is
% calculated from the bicycle kinematic model.
%
% Y_G: The y coordinate of the wheelchair in the global coordination, it is
% also calculated from the bicycle kinematic model
% 
% EPSILON: The shortest distance that is pre-defined by the users. This
% would allow user to determine the distance between the random nodes and
% the nearest neighbor nodes
% 
% INIT: Starting point defined by users, and it needs to be inside the map
% 
% GOAL: End goal point defined by users, and it needs to be inside the map
%
% OUTPUTS --- [Q_RAND, Q_NEAR, Q_NEW, VERTICES] 
% Q_RAND: The random node generated in the map, the number of iterations
% would assign the number of the random nodes; it has x and y coordinate
% respectively
% 
% Q_NEAR: The nearest neighbor nodes which are nearest away from its random 
% nodes. They are generated by the function file "fcn_findqnear.m"
% 
% Q_NEW: The new nodes which would be epsilon away from the nearest
% neighbors in the direction to the random nodes; The nodes are generated
% by the function file "fcn_findqnew.m"
%
% Developed by Yifeng Shi in IVSG at Penn State
% yms5066@psu.edu
% 02/08/2018
%%
clear all
close all

load 'x_g.mat'  % inputs [global positions on x axis]
load 'y_g.mat'  % inputs [global positions on y axis]
%tic            % Measure time period
init= [6,-14];   % initialize state for vertices
goal = [14,-13]; 
x_max = round(max(x_g));
y_max = round(min(y_g));
n_iterations = 1000; 
EPSILON = 0.4; % the short distance of the 
vertices = struct('x',[],'y',[]);
vertices(1).x = init(1);
vertices(1).y = init(2);
% obstacle setup 
obsx = [NaN 3 9 9 4 3 NaN 11 14 14 10 11 NaN 0 8 8 0 0 NaN 17 20 20 17 17];
obsy = [NaN -5 -7 -9 -9 -5 NaN -5 -7 -9 -9 -5 NaN -16 -18 -20 -20 -16 NaN -18 -18 -20 -20 -18];
obstacle = [obsx;obsy]';
plot(obsx,obsy,'LineWidth',1,'color','r')
%threshold = 0.25; % the distance that will determine whether the point robot will reach its goal
viscircles(init,0.25,'color','b');
viscircles(goal,0.25,'color','r');
axis equal
axis([0 21 -21 0])
%%  Main function 
% tic
for i = 1:n_iterations
            q_rand = [rand()*x_max, rand()*y_max];
            ind = fcn_findqnear(vertices,q_rand);
            q_near = [vertices(ind).x,vertices(ind).y];
        if (fcn_collisioncheck(obstacle,q_near(1),q_near(2)) == 0)
            [vertices(i+1).x,vertices(i+1).y] = fcn_findqnew(vertices(ind),q_rand,EPSILON);
            q_new = [vertices(i+1).x,vertices(i+1).y];   
        if (fcn_collisioncheck(obstacle,q_new(1),q_new(2)) == 0)
            line([q_near(1),q_new(1)],[q_near(2),q_new(2)],'Color','black')      % draw line from nearest point to new point
        end
        if sqrt((q_new(1)-goal(1))^2+(q_new(2)-goal(2))^2) <= 0.25 % stop expandsion when the tree reaches the end goal
            fprintf('goal reached\n')
            break
        end
        end
        pause(0.001)
end
% toc
