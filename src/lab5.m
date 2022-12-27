

%%
% RBE 3001 Lab 5 example code!
% Developed by Alex Tacescu (https://alextac.com)
%%
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

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
   
   %Detect ball positions
   [redPoints, orangePoints, yellowPoints, greenPoints, bluePoints] = detection.detect()

    imshow(cam.getImage());
    f0 = [75, -100];

    %If points exist, save those points, otherwise, it is at the drop off
    if (redPoints)
        dataPoint1 = redPoints(1,:);
    else
        dataPoint1 = [900 150];
    end

    dataPoint2 = [900 150];
    if (greenPoints)
        dataPoint3 = greenPoints;
    else
        dataPoint3 = [900 300];
    end
    
    dataPoint4 = [900 300];
    if (yellowPoints)
        dataPoint5 = yellowPoints;
    else
        dataPoint5 = [250 250];
    end

    dataPoint6 = [250 250];
    
    if (orangePoints)
        dataPoint7 = orangePoints;
    else
        dataPoint7 = [250 150];
    end
    dataPoint8 = [250 150];

    if(bluePoints)
        dataPoint9 = bluePoints;
    else
        dataPoint9 = dataPoint2;
    end

    %Find the points on the checkerboard in the checkerboard frame
    wPoints1 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint1);
    wPoints2 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint2);
    wPoints3 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint3);
    wPoints4 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint4);
    wPoints5 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint5);
    wPoints6 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint6);
    wPoints7 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint7);
    wPoints8 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint8);
    wPoints9 = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4), dataPoint9);

    %Convert to robot reference frame (rotation only)
    w1Trans = [wPoints1(2) wPoints1(1)];
    w2Trans = [wPoints2(2) wPoints2(1)];
    w3Trans = [wPoints3(2) wPoints3(1)];
    w4Trans = [wPoints4(2) wPoints4(1)];
    w5Trans = [wPoints5(2) wPoints5(1)];
    w6Trans = [wPoints6(2) wPoints6(1)];
    w7Trans = [wPoints7(2) wPoints7(1)];
    w8Trans = [wPoints8(2) wPoints8(1)];
    w9Trans = [wPoints9(2) wPoints9(1)];
    
    %Convert to robot reference frame (translation) and localize centerpoint of the ball
    robotPoints1 = w1Trans + f0 - correctCoords(wPoints1); %red
    robotPoints2 = w2Trans + f0 + [12.5 12.5];
    
    robotPoints3 = w3Trans + f0 - correctCoords(wPoints2); %green
    robotPoints4 = w4Trans + f0 + [12.5 12.5];

    robotPoints5 = w5Trans + f0 - correctCoords(wPoints3); %yellow
    robotPoints6 = w6Trans + f0 + [12.5 12.5];

    robotPoints7 = w7Trans + f0 - correctCoords(w4Trans); %orange
    robotPoints8 = w8Trans + f0 + [12.5 12.5];
    
    robotPoints9 = w9Trans + f0 - correctCoords(wPoints9); %blue

   %If the ball is detected on the board, move to its position and drop it
   %off
   if(dataPoint1(1)<890 && dataPoint1(1)>300)

    ballPoint1 = robot.ik3001([robotPoints1 0]);
    ballPoint2 = robot.ik3001([robotPoints2 0]);
    robot.moveToBallPoint(ballPoint1, ballPoint2);
   end

   pause(0.01);  
   if(dataPoint3(1)<890)
    ballPoint3 = robot.ik3001([robotPoints3 0]);
    ballPoint4 = robot.ik3001([robotPoints4 0]);
    robot.moveToBallPoint(ballPoint3, ballPoint4);
   end

   pause(0.01);
   if(dataPoint5(1)>260)
    ballPoint5 = robot.ik3001([robotPoints5 0]);
    ballPoint6 = robot.ik3001([robotPoints6 0]);
   robot.moveToBallPoint(ballPoint5, ballPoint6);
   end

   pause(0.01);
   if(dataPoint7(1)>260)
    ballPoint7 = robot.ik3001([robotPoints7 0]);
    ballPoint8 = robot.ik3001([robotPoints8 0]);
    robot.moveToBallPoint(ballPoint7, ballPoint8);
   end

   pause(0.01)
   if(dataPoint9(1)<890)
       ballPoint9 = robot.ik3001([robotPoints9 0]);
       ballPoint10 = robot.ik3001([robotPoints2 0]);
       robot.moveToBallPoint([ballPoint9(1) ballPoint9(2)+10 ballPoint9(3)], ballPoint10);
   end

   detection.detect();
    %If there are balls left on the checkerboard, recurse
   if((dataPoint1(1) < 850  && dataPoint1(1) > 325) || dataPoint7(1)>325 ||dataPoint5(1)>325 || dataPoint3(1)<850)
       robot.shutdown()
       cam.shutdown()
       lab5;
   end
   
    
   
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end


%If not, we are done
%% Shutdown Procedure
disp("DONE!!!");
robot.shutdown()
cam.shutdown()

%Localizes the centerpoint of the balls
function [newX, newY] = correctCoords(coords)
    if(coords(2) > 400)
        %Get x and y coordinates in R vector
        r = sqrt(coords(1)^2 + coords(2)^2);
        %Find rise angle
        theta = atan2(170, r);
        %Convert r to the r of the ball
        realPoint = 12.5/tan(theta);
        r = r - realPoint;
        %Find angle between the checkerboard edge and r
        alpha = atan2(coords(1), coords(2));
        %Find new x and y coordinates
        newX = r*cos(alpha);
        newY = r*sin(alpha);
    else
        newX = 0;
        newY = 0;
    end
end