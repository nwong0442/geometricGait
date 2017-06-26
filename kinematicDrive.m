clear all
close all
clc

RFun = @(theta) [cos(theta) -sin(theta); 
                 sin(theta) cos(theta)];
             
tau = 49/64*pi;
alphaMatFun =  @(t) [-pi/2;pi/2] + RFun(-pi/4) * [5/4*cos(t+tau); 17/20*sin(t+tau)];

vectorFieldFun = @(alpha1, alpha2) [1+cos(alpha2) 1+cos(alpha1);
                                    0             0            ;
                                    -sin(alpha2)  -sin(alpha1)];
DFun = @(alpha1, alpha2) sin(alpha1)-sin(alpha2)+sin(alpha1-alpha2);

alphaDotMatFun = @(t) RFun(-pi/4) * [-5/4*sin(t+tau); 17/20*cos(t+tau)];

epsilonFun = @(vectorField, D, alphaDotMat) -1/D * vectorField * alphaDotMat;

%--------------------------------------------------------------------------
% Simulation parameters

currentPose = [0;
               0;
               0];
robot = ExampleHelperRobotSimulator('emptyMap');
robotRadius = 0.5;
robot.setRobotSize(robotRadius);
robot.enableLaser(false); % Leaving this on will show the robot's field of vision
robot.showTrajectory(true); % Draw the path that the robot makes
robot.setRobotPose(currentPose);
controlRate = robotics.Rate(50); % Affects timing of loop iterations and 
                                  % resolution of pose measurements? 
                                  % Not entirely sure on what this does
                                  % specifically. 
xlim([-1 10]);
ylim([-5 5]);

%--------------------------------------------------------------------------
% Simulation

t = 0;
dt = pi/16;

while true
    alphaMat = alphaMatFun(t);
    vectorField = vectorFieldFun(alphaMat(1), alphaMat(2));
    D = DFun(alphaMat(1), alphaMat(2));
    alphaDotMat = alphaDotMatFun(t);
    epsilon = epsilonFun(vectorField, D, alphaDotMat);
    
    sign = epsilon(1)*epsilon(2)/(abs(epsilon(1)*epsilon(2)));
    if isnan(sign)
        if epsilon(1) == 0 && epsilon(2) == 0
            sign = 1;
        elseif epsilon(1) == 0
            sign = epsilon(2)/abs(epsilon(2));
        else
            sign = epsilon(1)/abs(epsilon(1));
        end
    end
    v = sign*(epsilon(1)^2 + epsilon(2)^2)^0.5;
    omega = epsilon(3);
    
    drive(robot, v, omega);
    t = t + dt;
    waitfor(controlRate);
end