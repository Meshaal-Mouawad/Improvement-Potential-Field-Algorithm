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

clc
close all
clear 
%% Defining environment variables
startPos = [5,15]; % robot Start Point
goalPos = [90, 95]; % target location
obs1Pos = [50, 50];% the obstacle coordinate
obs2Pos = [30, 80];
obsRad = 10; % obstical radius 
goalR = 0.2; % The radius of the goal
goalS = 20;  % The spread of attraction of the goal
obsS = 20;   % The spread of repulsion of the obstacle
alpha = 0.7; % Strength of attraction
beta = 0.80;  % Strength of repulsion 
%% To perform the Potential Field Math as follows:
u = zeros(100, 100);
v = zeros(100, 100);  
testu = zeros(100, 100);  
testv = zeros(100, 100); 

for x = 1:1:100
    for y = 1:1:100  
        [uG, vG] = GoalDelta(x, y, goalPos(1), goalPos(2), goalR, goalS, alpha); 
        [uO, vO] = ObsDelta(x, y, obs1Pos(2), obs1Pos(1), obsRad, obsS, beta); 
        [uO2, vO2] = ObsDelta(x, y, obs2Pos(2), obs2Pos(1), obsRad, obsS, beta);
% the resultant force 
        xnet = uG + uO + uO2 ;
        ynet = vG + vO + vO2 ;
        vspeed = sqrt(xnet^2 + ynet^2);
        theta = atan2(ynet,xnet);
        u(x,y) = vspeed*cos(theta);
        v(x,y) = vspeed*sin(theta);
%         hold on
        
    end
end
[X,Y] = meshgrid(1:1:100,1:1:100);
figure
quiver(X, Y, u, v, 3) 


%% Defining the grid

% Plotting the obstacles
circles(obs1Pos(1),obs1Pos(2),obsRad, 'facecolor','black')
axis square

hold on
circles(obs2Pos(1),obs2Pos(2),obsRad, 'facecolor','black')

hold on % Plotting initial position
circles(startPos(1),startPos(2),2, 'facecolor','green')

hold on % Plotting target position
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


