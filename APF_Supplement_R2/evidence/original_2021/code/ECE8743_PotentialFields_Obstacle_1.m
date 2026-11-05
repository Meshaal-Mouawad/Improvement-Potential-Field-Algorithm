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
startPos = [5,5];    % robot Start Point
goalPos = [90, 95];  % target coordinate 
obs1Pos = [50, 50];  % the obstacle coordinate
obsRad = 10;         % the obstacle radius
goalR = 0.2; % The radius of the goal
goalS = 20;  % The spread of attraction of the goal
obsS = 30;   % The spread of repulsion of the obstacle
alpha = 0.8; % Strength of attraction
beta = 0.6;  % Strength of repulsion
%% Carry out the Potential Field Math as follows: 
u = zeros(100, 100);
v = zeros(100, 100);
testu = zeros(100, 100);
testv = zeros(100, 100);
for x = 1:1:100
    for y = 1:1:100
        [uG, vG] = GoalDelta(x, y, goalPos(1), goalPos(2), goalR, goalS, alpha);
        [uO, vO] = ObsDelta(x, y, obs1Pos(2), obs1Pos(1), obsRad, obsS, beta);
        xnet = uG + uO; % the resultant on x
        ynet = vG + vO; % the resultant on y
        vspeed = sqrt(xnet^2 + ynet^2);   % the speed 
        theta = atan2(ynet,xnet);
        u(x,y) = vspeed*cos(theta);
        v(x,y) = vspeed*sin(theta);
%         hold on
        
    end
end
%%
[X,Y] = meshgrid(1:1:100,1:1:100);
figure
quiver(X, Y, u, v, 3)


%% Defining the grid

% Plotting the obstacles
circles(obs1Pos(1),obs1Pos(2),obsRad, 'facecolor','red')
axis square

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
    plot(currentPos(1),currentPos(2),'-o', 'MarkerFaceColor', 'black')
    pause(0.5)
end
