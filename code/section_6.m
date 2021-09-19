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
startPos = [5,60];
goalPos = [77, 60]; 
obsRad = 5;         % obstical radius 
obs1Pos = [30, 39];
obs2Pos = [50, 39];
obs3Pos = [50, 59];
obs4Pos = [50, 79];
obs5Pos = [30, 79];

%% Initial Parameters 

goalR = 0.2;        % The radius of the goal
goalS = 10          % The spread of attraction of the goal
obsS = 20;          % The spread of repulsion of the obstacle
alpha = 0.1         % Strength of attraction
beta = 0.7;         % Strength of repulsion


%% To perform the Potential Field Math as follows:
u = zeros(90, 90);
v = zeros(90, 90);  
testu = zeros(90, 90); % can not seem to find their prupose 
testv = zeros(90, 90);% can not see to find their purpose 

% Responsible for completing the calculation need to properly run the
% neural activity. 
for x = 1:1:90
    for y = 1:1:90  
        [uG, vG] = GoalDelta(x, y, goalPos(1), goalPos(2), goalR, goalS, alpha); % Calculates the Attractive force cause by the Goal
        [uO, vO] = ObsDelta(x, y, obs1Pos(2), obs1Pos(1), obsRad, obsS, beta);   % Calculates the Repulsive force by 1st obstacle
        [uO2, vO2] = ObsDelta(x, y, obs2Pos(2), obs2Pos(1), obsRad, obsS, beta); % Calculate the Repulsive force by the 2nd obstacle 
        [uO3, vO3] = ObsDelta(x, y, obs3Pos(2), obs3Pos(1), obsRad, obsS, beta); % Calculate the Repulsive force by the 3rd obstacle 
        [uO4, vO4] = ObsDelta(x, y, obs4Pos(2), obs4Pos(1), obsRad, obsS, beta); % Calculate the Repulsive force by the 4th obstacle 
        [uO5, vO5] = ObsDelta(x, y, obs5Pos(2), obs5Pos(1), obsRad, obsS, beta); % Calculate the Repulsive force by the 5th obstacle 
%       
        xnet = uG + uO + uO2 + uO3 + uO4 + uO5 ;
        ynet = vG + vO + vO2 + vO3 + vO4 + vO5  ;
        vspeed = sqrt(xnet^2 + ynet^2);
        theta = atan2(ynet,xnet);
        u(x,y) = vspeed*cos(theta);
        v(x,y) = vspeed*sin(theta);
%         hold on
        
    end
end
%%
[X,Y] = meshgrid(1:1:90,1:1:90);
figure
quiver(X, Y, u, v, 3) % plots th directional arrows seen in the graph 


%% Defining the grid
% Plotting the Circle obstacles

circles(obs1Pos(1),obs1Pos(2),obsRad, 'facecolor','red')
axis square

hold on
circles(obs2Pos(1),obs2Pos(2),obsRad, 'facecolor','red')

hold on
circles(obs3Pos(1),obs3Pos(2),obsRad, 'facecolor','red')

hold on
circles(obs4Pos(1),obs4Pos(2),obsRad, 'facecolor','red')

hold on
circles(obs5Pos(1),obs5Pos(2),obsRad, 'facecolor','red')

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
