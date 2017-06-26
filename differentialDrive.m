clear all
close all
clc

% Eyeballed the points from Fig 3a
points = [0,    0; 
          3.5,  3.5;
          2.5,  4.5;
          -0.5, 2.5];
      
% Vector field for differential drive 
vectorField = [-1 -1;
                0  0;
                1 -1];
            
alpha1Dot = [points(2,1)-points(1,1),...
             points(3,1)-points(2,1),...
             points(4,1)-points(3,1)];
         
alpha2Dot = [points(2,2)-points(1,2),...
             points(3,2)-points(2,2),...
             points(4,2)-points(3,2)];
         
epsilonFun = @(alphaMat) -vectorField * alphaMat;
%--------------------------------------------------------------------------
% First shape change

epsilon1 = epsilonFun([alpha1Dot(1); alpha2Dot(1)]);
% A dumb method to maintain the +/- sign of the velocity scalar
sign1 = epsilon1(1)*epsilon1(2)/(abs(epsilon1(1)*epsilon1(2)));
if isnan(sign1)
    if epsilon1(1) == 0 && epsilon1(2) == 0
        sign1 = 1;
    elseif epsilon1(1) == 0
        sign1 = epsilon1(2)/abs(epsilon1(2));
    else
        sign1 = epsilon1(1)/abs(epsilon1(1));
    end
end

% velocity scalar
v1 = sign1*(epsilon1(1)^2 + epsilon1(2)^2)^0.5;

% angular velocity scalar
omega1 = epsilon1(3);

%--------------------------------------------------------------------------
% Second shape change
epsilon2 = epsilonFun([alpha1Dot(2); alpha2Dot(2)]);
sign2 = epsilon2(1)*epsilon2(2)/(abs(epsilon2(1)*epsilon2(2)));
if isnan(sign2)    
    if epsilon2(1) == 0 && epsilon2(2) == 0
        sign2 = 1;
    elseif epsilon2(1) == 0
        sign2 = epsilon2(2)/abs(epsilon2(2));
    else
        sign2 = epsilon2(1)/abs(epsilon2(1));
    end
end
v2 = sign2*(epsilon2(1)^2 + epsilon2(2)^2)^0.5;
omega2 = epsilon2(3);

%--------------------------------------------------------------------------
% Third shape change
epsilon3 = epsilonFun([alpha1Dot(3); alpha2Dot(3)]);
sign3 = epsilon3(1)*epsilon3(2)/(abs(epsilon3(1)*epsilon3(2)));
if isnan(sign3)
    if epsilon3(1) == 0 && epsilon3(2) == 0
        sign3 = 1;
    elseif epsilon3(1) == 0
        sign3 = epsilon3(2)/abs(epsilon3(2));
    else
        sign3 = epsilon3(1)/abs(epsilon3(1));
    end
end
v3 = sign3*(epsilon3(1)^2 + epsilon3(2)^2)^0.5;
omega3 = epsilon3(3);

%--------------------------------------------------------------------------
% Set properties to the robot in theh simulation
currentPose = [2;
               5;
               0];
currentAngle = currentPose(3);
robot = ExampleHelperRobotSimulator('emptyMap');
robotRadius = 0.5;
robot.setRobotSize(robotRadius);
robot.enableLaser(false); % Leaving this on will show the robot's field of vision
robot.showTrajectory(true); % Draw the path that the robot makes
robot.setRobotPose(currentPose);
controlRate = robotics.Rate(100); % Affects timing of loop iterations and 
                                  % resolution of pose measurements? 
                                  % Not entirely sure on what this does
                                  % specifically. 
xlim([0 10]);
ylim([0 10]);

% The angles do not reach -pi and pi, so I am using this error to limit the
% range from -pi+angleError to pi-angleError
angleError = 0.05;

%--------------------------------------------------------------------------
% Simulation starts here
% I hard-coded values for when each shape change ends since time intervals
% were not given in the example. 

% First shape change
% Ends after an arbitrary distance
while (currentPose(1) < 5)
    drive(robot, v1, omega1);
    currentPose = robot.getRobotPose;
    waitfor(controlRate);
end

% Second shape change
% Ends when the robot rotates until a certain point
while (currentPose(3) < pi/2)
    drive(robot, v2, omega2);
    currentPose = robot.getRobotPose;
    waitfor(controlRate);
end

% Third shape change
% Ends when the robot rotates until a certain point
while (currentPose(3) < pi-angleError)
    drive(robot, v3, omega3);
    currentPose = robot.getRobotPose;
    waitfor(controlRate);
end