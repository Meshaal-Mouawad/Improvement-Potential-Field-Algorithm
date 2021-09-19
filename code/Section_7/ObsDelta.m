function [delXO, delYO] = ObsDelta(vx, vy, ox, oy, obsRad, obsS, beta)
% This function gives delX, delY of repulsion caused by the obstacPF
% this the improved AFP
L = 1;
inf = 10;
dObs = sqrt((ox-vx)^2 + (oy-vy)^2); % distance bw obstacle and current position
thetaO = atan2((oy-vy),(ox-vx));     % angle between goal and current position

dd = 4*obsRad;
if dObs<=obsRad
    delXO = -(sign(cos(thetaO)))*inf;
    
    delYO = -(sign(sin(thetaO)))*inf;
    
elseif (dObs < dd) && (dObs>=obsRad) 
    
    delXO = -L*((dd/ (dObs-obsRad))^beta)*cos(thetaO);
    delYO =-L*((dd/ (dObs-obsRad))^beta)*sin(thetaO);
   
else 
    delXO = 0;
    delYO = 0;
end

end