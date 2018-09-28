function [ind] = fcn_findqnear(vertices,q_rand)
% [IND] = FCN_FINDQNEAR(VERTICES,Q_RAND) returns the index of
% vertices in the RRT algroithm.
% 
% 
% 
%% find the nearest neighbors from the random states 
%% euclidean distance 
% ind=1;
% mindistance = sqrt((vertices(1).x-q_rand(1))^2+(vertices(1).y-q_rand(2))^2); 
% for i = 1:length(vertices)
%     e_dist = sqrt((vertices(i).x-q_rand(1))^2+(vertices(i).y-q_rand(2))^2);  
%     if e_dist < mindistance
%         mindistance = e_dist;
%         ind = i;
%     end
% end
%% manhatten distance
ind=1;
mindistance = sqrt((vertices(1).x-q_rand(1))^2)+sqrt((vertices(1).y-q_rand(2))^2); 
for i = 1:length(vertices)
    e_dist = sqrt((vertices(i).x-q_rand(1))^2)+sqrt((vertices(i).y-q_rand(2))^2);  
    if e_dist < mindistance
        mindistance = e_dist;
        ind = i;
    end
end
end