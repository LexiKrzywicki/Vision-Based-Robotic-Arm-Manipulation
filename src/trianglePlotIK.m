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
  posMatrix = zeros(600, 3);
  timeMatrix = zeros(600, 1);
  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  i = 0;
  counter = 0;
  tic
  while(i < 3)
      
      %Draw the triangle
      for k = 1:4
          currPos = [0 0 0];
          pos1 = robot.ik3001([100 0 191]);
          pos2 = robot.ik3001([100 0 100]);
          pos3 = robot.ik3001([60 -50 145]);
          packet(1) = 00;
          packet(2) = 0;
          if(k == 1 || k == 4)
              packet(3) = pos1(1);
              packet(4) = pos1(2);
              packet(5) = pos1(3);
          end
          if(k == 2)
              packet(3) = pos2(1);
              packet(4) = pos2(2);
              packet(5) = pos2(3);
          end
          if(k == 3)
              packet(3) = pos3(1);
              packet(4) = pos3(2);
              packet(5) = pos3(3);
          end
          % Send packet to the server and get the response      
          %robot.write sends a 15 float packet to the micro controller
           robot.write(SERV_ID, packet);
           %robot.read reads a returned 15 float backet from the micro controller.
           returnPacket = robot.read(SERVER_ID_READ);
           x = 0;

           %Record ~50 points per second for position and time
           while(x < 50)
             counter = counter + 1;
             currPos = robot.setpoint_js()
             if(currPos(3) ~= 0)
                posMatrix(counter, :) = robot.ik3001(currPos);
             end
                 
             timeMatrix(counter) = toc;
             x = x + 1;
             pause(0.01);
           end
          toc
          pause(1)
         
      end
      i = i + 1;
      
  end
  robot.closeGripper()
  pause(1)
  robot.openGripper()
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
%Concatenate matrices
finalMatrix = [timeMatrix posMatrix];

%Write to csv
writematrix(finalMatrix, 'Section3.csv');

%Plot X, Y, and Z positions to create the drawing of the triangle
plot(finalMatrix(:, 1), finalMatrix(:,2), "-",finalMatrix(:, 1), finalMatrix(:,3), "-",finalMatrix(:, 1), finalMatrix(:,4), "-")
%plot3(finalMatrix(:, 3), finalMatrix(:, 2), finalMatrix(:, 4), "-");
legend("Joint 1", "Joint 2", "Joint 3")
%title("Position (mm)")
xlabel("Time (s)")
ylabel("Joint angle (deg)")
%zlabel("Z")
% Clear up memory upon termination
robot.shutdown()

toc