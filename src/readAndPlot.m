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
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  viaPts = [0,45];

  for k = viaPts
      tic
      packet = zeros(15, 1, 'single');
      packet(1) = 2000;%one second time
      packet(2) = 0;%linear interpolation
      packet(3) = k;
      packet(4) = 0;% Second link to 0
      packet(5) = 0;% Third link to 0
      
      % Send packet to the server and get the response      
      %robot.write sends a 15 float packet to the micro controller
      robot.write(SERV_ID, packet);

      %Create first row of arrays and initialize counter
      startArray = [0 0 0];
      startTimeArray = [0];
      counter = 0;

      %Record data while time is less than 2.05 seconds
      while(toc < 2.05)
        counter = counter + 1;
       
        %robot.read reads a returned 15 float backet from the micro controller.
        returnPacket = robot.read(SERVER_ID_READ);

        %Read correct packets from return packet to get position and toc to
        %get time
        posArray = [returnPacket(3) returnPacket(5) returnPacket(7)];
        timeArray = [toc];
        
        %Concatenate the arrays and set the start array to the final array
        finalArray = [startArray; posArray];
        startArray = finalArray;

        %Concatenate the arrays and set the start array to the final array
        finalTimeArray = [startTimeArray; timeArray];
        startTimeArray = finalTimeArray;
      
      end
      %Pause so the bot has enough time to fully reach the set position
      pause(2);
      toc

      if DEBUG
          disp('Sent Packet:')
          disp(packet);
          disp('Received Packet:');
          disp(returnPacket);
      end
      
      toc
      pause(1) 
      
  end
  %Add the time array to the first column of the full array
  finalArray = [finalTimeArray finalArray];

  % Closes then opens the gripper
  robot.closeGripper()
  pause(1)
  robot.openGripper()
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
%Save to csv
writematrix(finalArray, 'readAndPlot.csv');

% Clear up memory upon termination
robot.shutdown()

toc
%Create a plot with all the joint angles
plot(finalArray(:,1), finalArray(:,2))
hold on;
plot(finalArray(:,1), finalArray(:,3))
plot(finalArray(:,1), finalArray(:,4))
legend("Joint 1", "Joint 2", "Joint 3");
title("Joint Rotation vs Time");
xlabel("Time (s)");
ylabel("Angle (deg)");
hold off;

%Create a histogram of all the points
histogram(finalArray(:,2), 50)
legend("Joint 1 Readings");
title("# of Readings vs Angle");
ylabel("Readings");
xlabel("Angle (deg)");
hold off;

%Record the difference between reading times of all array indices
for n = 1:2100
    diffArray(n,1) = finalArray(n+1, 1) - finalArray(n, 1);
end

%Create a histogram of the time difference between points
histogram(diffArray, 50)
legend("Joint 1 Readings");
title("# of Readings vs Time");
ylabel("Readings");
xlabel("Time(s)");
xlim([0, 0.005]);
%}