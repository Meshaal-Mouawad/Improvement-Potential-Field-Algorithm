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
startPos = [5,15];
goalPos = [90, 95];
obs1Pos = [50, 50];
obs2Pos = [30, 80];
obs3Pos = [20,25];
obs4Pos= [25,35];
obs5Pos = [47,27];
obs6Pos = [50,40] ;
obs7Pos = [75,50] ;
obs8Pos = [70,65];
obs9Pos = [80,70];
obs10Pos = [90,15];
obsRad = 10;
goalR = 0.2; % The radius of the goal
goalS = 10;  % The spread of attraction of the goal
obsS = 10;   % The spread of repulsion of the obstacle
alpha = 0.8; % Strength of attraction
beta = 2;  % Strength of repulsion


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
        xnet = uG + uO + uO2; % the resultant on x
        ynet = vG + vO + vO2; % the resultant on y
        vspeed = sqrt(xnet^2 + ynet^2);
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

hold on
circles(obs2Pos(1),obs2Pos(2),obsRad, 'facecolor','red')
hold on
circles(obs3Pos(1),obs3Pos(2),obsRad, 'facecolor','red')
hold on
circles(obs4Pos(1),obs4Pos(2),obsRad, 'facecolor','red')
hold on
circles(obs5Pos(1),obs5Pos(2),obsRad, 'facecolor','red')
hold on
circles(obs6Pos(1),obs6Pos(2),obsRad, 'facecolor','red')
hold on
circles(obs7Pos(1),obs7Pos(2),obsRad, 'facecolor','red')
hold on
circles(obs8Pos(1),obs8Pos(2),obsRad, 'facecolor','red')
hold on
circles(obs9Pos(1),obs9Pos(2),obsRad, 'facecolor','red')
hold on
circles(obs10Pos(1),obs10Pos(2),obsRad, 'facecolor','red')

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
tic
   % The program section to time. 
toc