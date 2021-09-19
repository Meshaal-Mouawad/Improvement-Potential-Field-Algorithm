function [delXG, delYG] = GoalDelta(vx, vy, gx, gy, goalR, goalS, alpha)
% This function gives delX, delY of attraction caused by the goal point

dGoal = sqrt((gx-vx)^2 + (gy-vy)^2); % distance bw goal and current position
thetaG = atan2((gy-vy),(gx-vx));     % angle between goal and current position

if dGoal<goalR 
    delXG = 0; delYG = 0; 
    
elseif ((goalS + goalR) >= dGoal) && (dGoal >= goalR)    
    delXG = 2*((dGoal - goalR)^alpha)*cos(thetaG); 

    delYG = 2*((dGoal - goalR)^alpha)*sin(thetaG);

else 
    delXG = (goalS^alpha)*cos(thetaG);
    delYG = (goalS^alpha)*sin(thetaG);

end
