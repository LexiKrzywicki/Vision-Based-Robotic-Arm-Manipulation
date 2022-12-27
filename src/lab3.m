clear
clear java;
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
planner = Traj_Planner();
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  %Preallocate matrices
  %posMatrix = zeros(600, 3);
  %timeMatrix = zeros(600, 1);
  %jointMatrix = zeros(600, 3);
  
  %Create joint position matrices from setpoints
  jointPos1 = robot.ik3001([100 0 191]);
  jointPos2 = robot.ik3001([0 0 -90]);
  %jointPos2 = robot.ik3001([100 0 100]);
  jointPos3 = robot.ik3001([0 -45 0]);
  %jointPos3 = robot.ik3001([60 -50 145]);
 

  %Create joint angle and position matrices from quintic trajectory function for first edge
   traj_MatrixJoint1(1, :) = planner.quintic_traj(0, 1, jointPos1(1), jointPos2(1), 0, 0, 0, 0);
   traj_MatrixJoint1(2, :) = planner.quintic_traj(0, 1, jointPos1(2), jointPos2(2), 0, 0, 0, 0);
   traj_MatrixJoint1(3, :) = planner.quintic_traj(0, 1, jointPos1(3), jointPos2(3), 0, 0, 0, 0);
   traj_MatrixPos1(1, :) = planner.quintic_traj(0, 1, 100, 100, 0, 0, 0, 0);
   traj_MatrixPos1(2, :) = planner.quintic_traj(0, 1, 0, 0, 0, 0, 0, 0);
   traj_MatrixPos1(3, :) = planner.quintic_traj(0, 1, 191, 100, 0, 0, 0, 0);
   edge1Pos = robot.run_trajectory(traj_MatrixPos1, 1, 0);
   edge1Joint = robot.run_trajectory(traj_MatrixJoint1, 1, 1);
   pause(5);

  %Create joint angle and position matrices from quintic trajectory function for second edge
   traj_MatrixJoint2(1, :) = planner.quintic_traj(0, 1, jointPos2(1), jointPos3(1), 0, 0, 0 ,0);
   traj_MatrixJoint2(2, :) = planner.quintic_traj(0, 1, jointPos2(2), jointPos3(2), 0, 0, 0 ,0);
   traj_MatrixJoint2(3, :) = planner.quintic_traj(0, 1, jointPos2(3), jointPos3(3), 0, 0, 0 ,0);
   traj_MatrixPos2(1, :) = planner.quintic_traj(0, 1, 100, 60, 0, 0, 0 ,0);
   traj_MatrixPos2(2, :) = planner.quintic_traj(0, 1, 0, -50, 0, 0, 0 ,0);
   traj_MatrixPos2(3, :) = planner.quintic_traj(0, 1, 100, 145, 0, 0, 0, 0);
   edge2Pos = robot.run_trajectory(traj_MatrixPos2, 1, 0);
   edge2Joint = robot.run_trajectory(traj_MatrixJoint2, 1, 1);
   pause(5);

  %Create joint angle and position matrices from quintic trajectory function for third edge
   traj_MatrixJoint3(1, :) = planner.quintic_traj(0, 1, jointPos3(1), jointPos1(1), 0, 0, 0, 0);
   traj_MatrixJoint3(2, :) = planner.quintic_traj(0, 1, jointPos3(2), jointPos1(2), 0, 0, 0, 0);
   traj_MatrixJoint3(3, :) = planner.quintic_traj(0, 1, jointPos3(3), jointPos1(3), 0, 0, 0, 0);
   traj_MatrixPos3(1, :) = planner.quintic_traj(0, 1, 60, 100, 0, 0, 0, 0);
   traj_MatrixPos3(2, :) = planner.quintic_traj(0, 1, -50, 0, 0, 0, 0, 0);
   traj_MatrixPos3(3, :) = planner.quintic_traj(0, 1, 145, 191, 0, 0, 0, 0);
   edge3Pos = robot.run_trajectory(traj_MatrixPos3, 1, 0);
   edge3Joint = robot.run_trajectory(traj_MatrixJoint3, 1, 1);
   pause(5);

  robot.closeGripper()
  pause(1)
  robot.openGripper()
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

%Concatenate joint angle and position matrices
finalEdgePos = [edge1Pos; edge2Pos; edge3Pos];
%finalEdgeJoint = [edge1Joint; edge2Joint; edge3Joint];
[sizeRow, sizeCol] = size(finalEdgePos);

% create a new time matrix from 0 to 3 with step equal to 
% the amount of rows in the finalEdge matrix
time = 0: 3/sizeRow : 3;
% transpose the matrix: row -> column
time = time';
%start at element 1 and go to the number of sizeRow elements
%number of sizeRow elements equals to row of finalEdge matrix
time = time(1:sizeRow);

% Plot x, y, and z velocities
% figure(1)
% plot(time, finalEdgePos(:,2), "-")
% hold on
% plot(time, finalEdgePos(:,3), "-")
% plot(time, finalEdgePos(:,4), "-")
% hold off
% legend("X", "Y", "Z")
% title("Linear Velocity vs. Time");
% xlabel("Time (s)")
% ylabel("Velocity (mm/s)")
% 
% Plot angular velocities
% figure(2)
% pVel = finalEdgePos(:,5);
% tVel = finalEdgePos(:,6);
% psVel = finalEdgePos(:,7);
% subtract 1 row
% time = time(1:sizeRow - 1);
% plot(time, pVel(1:sizeRow-1, :), "-")
% hold on
% plot(time, tVel(1:sizeRow-1, :), "-")
% plot(time, psVel(1:sizeRow-1, :), "-")
% hold off
% legend("Omega X", "Omega Y", "Omega Z")
% title("Angular Velocity vs. Time");
% xlabel("Time (s)")
% ylabel("Velocity (mm/s)")
% 
% Plot scalar velocity
% figure(3)
% time = time(1:sizeRow - 2);
% scalarMat = zeros(sizeRow-2, 3);
% for n = 1:sizeRow-2
%     scalarMat(n,:) = norm(finalEdgePos(n, 2:4));
% end
% plot(time, scalarMat, "-")
% hold off
% title("Scalar Magnitude vs Time");
% xlabel("Time (s)")
% ylabel("Linear Velocity (mm/s)")


robot.shutdown()

