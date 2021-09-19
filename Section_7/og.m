function [delXO, delYO] = ObsDelta(vx, vy, ox, oy, obsRad, obsS, beta)
% This function gives delX, delY of repulsion caused by the obstacle

inf = 10;
dObs = sqrt((ox-vx)^2 + (oy-vy)^2); % distance bw obstacle and current position
thetaO = atan2((oy-vy),(ox-vx));     % angle between goal and current position
% delXO = 0; delYO = 0;
dd = 4*obsRad;
if dObs<obsRad % if the obstacle distnace is greater than the distance between the obstacle and current position 
   %Generate the X cordinate if the condition is met 
    delXO = -(sign(cos(thetaO)))*inf;
    
    %Generate the Y cordinate if the condition is met 
    delYO = -(sign(sin(thetaO)))*inf;
    
elseif (dObs < (obsS + obsRad)) && (dObs>=obsRad) 
    % if the distance bwetween the current postion and obstacle is less than the
    % sum of the repulsion  of the ostacle and obstical radus  and is the distance
    %to the goal  greater to or equal to the obstical radius 
    
    %Generate the X cordinate if the condition is met 
    delXO = -((obsS + obsRad - dObs)^beta)*cos(thetaO);
    
    %Generate the Y cordinate if the condition is met 
    delYO = -((obsS + obsRad - dObs)^beta)*sin(thetaO);
   
else % if no condition is meet return z 
    delXO = 0;
    delYO = 0;
end

end