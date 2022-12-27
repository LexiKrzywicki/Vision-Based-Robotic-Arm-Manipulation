
clear
clear java;
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
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
  m = zeros(515, 7);
  i = 0;
  tic
  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  while(toc < 1)
      packet = zeros(15, 1, 'single');
      packet(1) = 000;%one second time
      packet(2) = 0;%linear interpolation
      packet(3) = 0;
      packet(4) = 55.18;% Second link to 0
      packet(5) = -20.05;% Third link to 0
    
      % Send packet to the server and get the response      
      %robot.write sends a 15 float packet to the micro controller
       robot.write(SERV_ID, packet);
       %robot.read reads a returned 15 float backet from the micro controller.
       returnPacket = robot.read(SERVER_ID_READ);
       n = robot.setpoint_js();
       while(n(2) < 53 || n(3) > -19)
         i = i + 1;
         n = robot.setpoint_js();
         T= robot.fk3001(n);
         o = T(1:3,4);
         m(i, :) = [toc o' n];
         pause(0.001)
       end
       toc
      packet = zeros(15, 1, 'single');
      packet(1) = 000;%one second time
      packet(2) = 0;%linear interpolation
      packet(3) = 0;
      packet(4) = 60.22;% Second link to 0
      packet(5) = 29.15;% Third link to 0
      % Send packet to the server and get the response      
      %robot.write sends a 15 float packet to the micro controller
       robot.write(SERV_ID, packet);
       %robot.read reads a returned 15 float backet from the micro controller.
       returnPacket = robot.read(SERVER_ID_READ);
       n = robot.setpoint_js();
       while(n(2) < 58 || n(3) < 28)
         i = i + 1;
         n = robot.setpoint_js();
         T= robot.fk3001(n);
         o = T(1:3,4);
         m(i, :) = [toc o' n];
         pause(0.001)
       end
       toc
      packet = zeros(15, 1, 'single');
      packet(1) = 000;%one second time
      packet(2) = 0;%linear interpolation
      packet(3) = 0;
      packet(4) = 18.94;% Second link to 0
      packet(5) = -21.73;% Third link to 0
      % Send packet to the server and get the response      
      %robot.write sends a 15 float packet to the micro controller
       robot.write(SERV_ID, packet);
       %robot.read reads a returned 15 float backet from the micro controller.
       returnPacket = robot.read(SERVER_ID_READ);
       n = robot.setpoint_js();
       while(n(2) < 17 || n(3) > -20)
        i = i + 1;
        n = robot.setpoint_js();
        T= robot.fk3001(n);
        o = T(1:3,4);
        m(i, :) = [toc o' n];
        pause(0.001)
       end
       toc
      
  end
  writematrix(m, 'Section8.csv')
  robot.closeGripper()
  pause(1)
  robot.openGripper()
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
T = readmatrix('Section8.csv');
figure(1)
plot(T(:,1), T(:,5), "-", T(:,1), T(:,6), "-", T(:,1),T(:,7))
legend("Joint 1", "Joint 2", "Joint 3")
xlabel("Time (s)")
ylabel("Rotation (degrees)")
hold off

figure(2)
plot(T(:,2), T(:,4), "-")
hold on
plot(T(:,2), T(:,4), ".")
legend("Trajectory")
xlabel("X Position (mm)")
ylabel("Z Position (mm)")
hold off

figure(3)
plot(T(:,2), T(:,3), "-")
hold on
plot(T(:,2), T(:,3), ".")

legend("Trajectory")
xlabel("X Position (mm)")
ylabel("Y Position (mm)")
hold off
% Clear up memory upon termination
robot.shutdown()

toc