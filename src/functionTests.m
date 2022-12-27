%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ./lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
robot = Robot(myHIDSimplePacketComs); 

%This script was made for testing all of the functions written in Robot.m
try
    %Right now set up to zero the joints then move to a defined point from
    %our by hand movements
    robot.interpolate_jp(1000, [0 0 0])
    robot.interpolate_jp(3000, [33.12 94.54 -39.97])
    tic
    while (toc < 3)
        %Same plot technique as described in readAndPlot.m
        startTimeArray = [0];
        timeArray = [toc];
        startArray = [0 0 0];
        posArray = robot.setpoint_js();
        finalArray = [startArray; posArray];
        startArray = finalArray;
        finalTimeArray = [startTimeArray; timeArray];
        startTimeArray = finalTimeArray;
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
%Same plot technique as described in readAndPlot.m
finalArray = [finalTimeArray finalArray];
plot(finalArray(:,1), finalArray(:,2))
hold on;
plot(finalArray(:,1), finalArray(:,3))
plot(finalArray(:,1), finalArray(:,4))
legend("Joint 1", "Joint 2", "Joint 3");
title("Joint Rotation vs Time");
xlabel("Time (s)");
ylabel("Angle (deg)");
hold on;

% Clear up memory upon termination
robot.shutdown()