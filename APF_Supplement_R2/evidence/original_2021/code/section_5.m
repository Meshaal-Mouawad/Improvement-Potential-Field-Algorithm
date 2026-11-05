%%% Southfield,Michigan
%%% May 23, 2016
%%% Potential Fields for Robot Path Planning
%
%
% Initially proposed for real-time collision avoidance [Khatib 1986].  
% Hundreds of papers published on APF
% A potential field is a scalar function over the free space.
% To navigate, the robot applies a force proportional to the 
% negated gradient of the potential field.
% A navigation function is an ideal potential field 
%
% code was edited by Meshaal Mouawad 09/13/2021
% email: mm4922@msstate.edu
% Mississippi State University 
% ECE Department 

clc
close all
clear 
%% Defining environment variables
startPos = [15,8];
goalPos = [100, 75];
obs1Pos = [20, 25];
obs2Pos = [20, 35];
obs3Pos = [47, 27];
obs4Pos = [50, 40];
obs5Pos = [100, 15];
obs6Pos = [75, 60];
obs7Pos = [70, 80];
obs8Pos = [80, 83];

goalR = 0.2;             % The radius of the goal
goalS = 20;              % The spread of attraction of the goal
obsS = 15 ;              % The spread of repulsion of the obstacle
alpha = 0.9;             % Strength of attraction
beta = 0.5;              % Strength of repulsion
map_s = 110;
%% To perform the Potential Field Math as follows:
u = zeros(map_s, map_s);
v = zeros(map_s, map_s);  
testu = zeros(map_s, map_s); 
testv = zeros(map_s, map_s); 


for x = 1:1:map_s
    for y = 1:1:map_s  % tells MATLAB to create a vector of values from 1 to 100, with a spacing of 1. Similarly,
        [uG, vG] = GoalDelta(x, y, goalPos(1), goalPos(2), goalR, goalS, alpha); % Calculates the Attractive force  cause by the Goal
        
        [uO, vO] = ObsDelta(x, y, obs1Pos(2), obs1Pos(1), 12, obsS, beta); % find the Repulsive force 
        [uO2, vO2] = ObsDelta(x, y, obs2Pos(2), obs2Pos(1), 18, obsS, beta);% find the Repulsive force by the second obstacle 
        [uO3, vO3] = ObsDelta(x, y, obs3Pos(2), obs3Pos(1), 11, obsS, beta);% find the Repulsive force by the third obstacle 
        [uO4, vO4] = ObsDelta(x, y, obs4Pos(2), obs4Pos(1), 18, obsS, beta);% find the Repulsive force by forth obstacle 
        [uO5, vO5] = ObsDelta(x, y, obs5Pos(2), obs5Pos(1), 10, obsS, beta);% find the Repulsive force by the fifth obstacle 
        [uO6, vO6] = ObsDelta(x, y, obs6Pos(2), obs6Pos(1), 15, obsS, beta);% Calculate the Repulsive force by the sixth obstacle 
        [uO7, vO7] = ObsDelta(x, y, obs7Pos(2), obs7Pos(1), 10, obsS, beta);% Calculate the Repulsive force by the seventh obstacle 
        [uO8, vO8] = ObsDelta(x, y, obs8Pos(2), obs8Pos(1), 10, obsS, beta);% Calculate the Repulsive force by the eigth obstacle 

        xnet = uG + uO + uO2+ uO3 +  uO4 +  uO5 +  uO6  +  uO7 +  uO8;
        ynet = vG + vO + vO2+  vO3 +  vO4+  vO5 +  vO6 +  vO7 +  vO8;
        vspeed = sqrt(xnet^2 + ynet^2);
        theta = atan2(ynet,xnet);
        u(x,y) = vspeed*cos(theta);
        v(x,y) = vspeed*sin(theta);
        
    end
end
%%
[X,Y] = meshgrid(1:1:map_s,1:1:map_s);
figure
quiver(X, Y, u, v, 3) 

 


%% Defining the grid

 

% Plotting the obstacles
circles(obs1Pos(1),obs1Pos(2),12, 'facecolor','red')
axis square

hold on
circles(obs2Pos(1),obs2Pos(2),18, 'facecolor','red')
hold on
circles(obs3Pos(1),obs3Pos(2),11, 'facecolor','red')
hold on
circles(obs4Pos(1),obs4Pos(2),18, 'facecolor','red')
hold on
circles(obs5Pos(1),obs5Pos(2),10, 'facecolor','red')
hold on
circles(obs6Pos(1),obs6Pos(2),15, 'facecolor','red')
hold on
circles(obs7Pos(1),obs7Pos(2),10, 'facecolor','red')
hold on
circles(obs8Pos(1),obs8Pos(2),10, 'facecolor','red')
hold on % Plotting start position
circles(startPos(1),startPos(2),2, 'facecolor','green')

 

hold on % Plotting goal position
circles(goalPos(1),goalPos(2),2, 'facecolor','yellow')

 

%% Priting of the path
currentPos = startPos;
x = 0;

 

while sqrt((goalPos(1)-currentPos(1))^2 + (goalPos(2)-currentPos(2))^2) > 1
    tempPos = currentPos + [u(currentPos(1),currentPos(2)), v(currentPos(1),currentPos(2))]
    currentPos = round(tempPos)
    hold on
    plot(currentPos(1),currentPos(2),'o', 'MarkerFaceColor', 'black')
    pause(0.5)
end

 


tempPos