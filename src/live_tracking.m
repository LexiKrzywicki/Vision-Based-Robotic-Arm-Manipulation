clc;
clear;
clear java;
format short
    

%% Flags
DEBUG = false;
STICKMODEL = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath ./lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

%Load robot and camera
robot = Robot(myHIDSimplePacketComs);
load 'camera.mat';

try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end

    %Set intial points
    dataPoint1 = [500, 250];
    prevRedPoints = [0, 0];

    f0 = [75, -100];

    %Set drop off point
    dataPoint2 = [900 150];
    wPoints2 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint2);
    w2Trans = [wPoints2(2) wPoints2(1)];
    robotPoints2 = w2Trans + f0 + [12.5 12.5];
    ballPoint2 = robot.ik3001([robotPoints2 0]);

    %While ball is still on checkerboard
    while(dataPoint1(1)<950 && dataPoint1(1)>200)
        %Take pictures and find ball
        [redPoints, orangePoints, yellowPoints, greenPoints, bluePoints] = detection.detect();
        imshow(cam.getImage())

        %If ball is on checkerboard return its points, otherwise, off the
        %board
        if (redPoints)
            dataPoint1 = redPoints(1,:);
        else
            dataPoint1 = [1000 178];
        end

        %Transfer to robot coordinate frame
        wPoints1 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint1);
        
        w1Trans = [wPoints1(2) wPoints1(1)];
        
        robotPoints1 = w1Trans + f0 - correctCoords(wPoints1);
    
        ballPoint1 = robot.ik3001([robotPoints1 50]);
        
        %Move to ball position as fast as possible
        robot.interpolate_jp(1, ballPoint1);
        
        %If the ball hasn't moved pick it up
        if((prevRedPoints(1) > (dataPoint1(1) - 5)) && (prevRedPoints(1) < (dataPoint1(1) + 5)) && (prevRedPoints(2) > (dataPoint1(2) - 5)) && (prevRedPoints(2) < (dataPoint1(2) + 5)))
            %Does same thing as robot.moveToBallPoint() but SIGNIFICANTLY
            %faster
            ballPoint1 = robot.ik3001([robotPoints1(1) robotPoints1(2) 0]);
            robot.interpolate_jp(1,[ballPoint1(1)-10 0 0]);
            pause(0.01);
            robot.interpolate_jp(1,[ballPoint1(1)-10 ballPoint1(2) ballPoint1(3)]);
            pause(0.01);
            robot.closeGripper();
            pause(0.25);
            robot.interpolate_jp(1,ballPoint2);
            pause(0.5);
            robot.openGripper;
        end
        
        %Set previous points for picking it up
        prevRedPoints = dataPoint1;
        pause(0.01);
    end  
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end



%% Shutdown Procedure
disp("DONE!!!");
robot.shutdown()
cam.shutdown()

function [newX, newY] = correctCoords(coords)
    if(coords(2) > 400)
        r = sqrt(coords(1)^2 + coords(2)^2);
        theta = atan2(170, r);
        realPoint = 12.5/tan(theta);
        r = r - realPoint;
        alpha = atan2(coords(1), coords(2));
        newX = r*cos(alpha);
        newY = r*sin(alpha);
    else
        newX = 0;
        newY = 0;
    end
end