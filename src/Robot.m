classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
        finalPosition = [0 0 0];
    end
    
    methods
        function T = jacob3001(bot, positions)
            t1 = positions(1);
            t2 = positions(2);
            t3 = positions(3);
            T = [-100*sind(t1)*cosd(t2)*cosd(t3)-100*sind(t1)*sind(t2)+100*sind(t1)*sind(t2)*sind(t3) -100*cosd(t1)*sind(t2)*cosd(t3)+100*cosd(t1)*cosd(t2)-100*cosd(t1)*cosd(t2)*sind(t3) -100*cosd(t1)*cosd(t2)*sind(t3)-100*cosd(t1)*sind(t2)*cosd(t3);
                100*cosd(t1)*cosd(t2)*cosd(t3)+100*cosd(t1)*sind(t2)-100*cosd(t1)*sind(t2)*sind(t3) -100*sind(t1)*sind(t2)*cosd(t3)+100*sind(t1)*cosd(t2)-100*sind(t1)*cosd(t2)*sind(t3) -100*sind(t1)*cosd(t2)*sind(t3)-100*sind(t1)*sind(t2)*cosd(t3);
                0 -100*sind(t2)+100*sind(t2)*sind(t3)-100*cosd(t2)*cosd(t3) -100*cosd(t2)*cosd(t3)+100*sind(t2)*sind(t3);
                0 -sind(t1) -sind(t1);
                0 cosd(t1) cosd(t1);
                1 0 0];
        end

        %Runs the trajectory planning for our quintic polynomial
        function T = run_trajectory(bot, coefficients, time, joint)
            tic
            counter = 2;
            %Preallocate space
            posMatrix = zeros(64, 3);
            velMatrixl = zeros(64, 3);
            velMatrixa = zeros(64, 3);
            timeMatrix = zeros(66, 1);
            detjMatrix = zeros(64, 1);
                while(toc <= time)
                    if(joint)
                        %In joint space, use polynomial formula provided 
                        positions = [coefficients(1,1)+(coefficients(1,2)*time) + coefficients(1,3)*time^2 + coefficients(1,4)*time^3 + coefficients(1,5)*time^4 + coefficients(1,6)*time^5, coefficients(2,1)+(coefficients(2,2)*time) + coefficients(2,3)*time^2 + coefficients(2,4)*time^3 + coefficients(2,5)*time^4 + coefficients(2,6)*time^5, coefficients(3,1)+(coefficients(3,2)*time) + coefficients(3,3)*time^2 + coefficients(3,4)*time^3 + coefficients(3,5)*time^4 + coefficients(3,6)*time^5];
                    else
                        %In task space, use polynomial formula provided then convert to joint angles
                        positions = [coefficients(1,1)+(coefficients(1,2)*time) + coefficients(1,3)*time^2 + coefficients(1,4)*time^3 + coefficients(1,5)*time^4 + coefficients(1,6)*time^5, coefficients(2,1)+(coefficients(2,2)*time) + coefficients(2,3)*time^2 + coefficients(2,4)*time^3 + coefficients(2,5)*time^4 + coefficients(2,6)*time^5, coefficients(3,1)+(coefficients(3,2)*time) + coefficients(3,3)*time^2 + coefficients(3,4)*time^3 + coefficients(3,5)*time^4 + coefficients(3,6)*time^5];
                        positions = bot.ik3001(positions);
                    end
                    %Move to and record positions
                    bot.interpolate_jp(1000, positions);
                    mVal = bot.measured_js(1,1);
                    posMatrix(counter, :) = mVal(1,:);
                    jacobSetpoint = bot.jacob3001(mVal(1,:));
                    detj = det(jacobSetpoint(1:3,:));
                    detjMatrix(counter,1) = detj;
                    timeMatrix(counter, 1) = toc;
                    if (detj < 100000 && detj > -100000)
                        figure(1)
                        bot.plot_arm(mVal(1,:));
                        text(0, 0, 200, "Reached a singularity!");
                        figure(2)
                        plot3(posMatrix(:,1), posMatrix(:,2), posMatrix(:,3), "-")
                        title("Motion of the End Effector")
                        xlabel("X")
                        ylabel("Y")
                        zlabel("Z")
                        figure(3)
                        timeMatrix = [readmatrix('Time4.csv'); timeMatrix]
                        timeMatrix(70,1) = 1 + timeMatrix(70,1);
                        detjMatrix = [readmatrix('Lab4.csv'); detjMatrix]
                        [sizerow, sizecol] = size(detjMatrix);
                        plot(timeMatrix(1:70,1), detjMatrix(1:70,1), "-")
                        title("Determinant of Jp vs Time")
                        xlabel("time (s)")
                        ylabel("Determinant")
                        error("Reached a singularity");
                    end
                    q = mVal(2,:);
                    p = bot.fdk3001(posMatrix(counter,:), q);
                    velMatrixa(counter,:) = p(4:6)';
                    velMatrixl(counter,:) = p(1:3)';
                    counter = counter + 1;
                    
                    %bot.plot_arm(mVal(1,:));
                    %hold on
                    %scale = double(sqrt(mVal(1,1)^2 + mVal(1,2)^2 + mVal(1,3)^2))/10;
                    %position = bot.fk3001(mVal(1,:));
                    %quiver3(position(1,4), position(2,4), position(3,4), mVal(1,1), mVal(1,2), mVal(1,3), scale, 'b')
                    %hold off
                
                
                    pause(0.01);
                    
                end
            %Concatenate matrices
            T = [timeMatrix, velMatrixl, velMatrixa, posMatrix];
            writematrix(detjMatrix, 'Lab4.csv')
             writematrix(timeMatrix, 'Time4.csv')
        end 

        %Performs inverse kinematics to find joint angles
        function T = ik3001(bot, input)
            X = input(1);
            Y = input(2);
            Z = input(3);
            a1 = 100;
            a2 = 100;

            %Get R vector as function of X and Y
            R = sqrt(X^2 +Y^2);
            %Establish the ZR plane
            R1 = sqrt(X^2 +Y^2 + (Z-95)^2);
            if(R1 > 200)
                %bot.shutdown()
                error("outside of workspace")
            end

            %Joint 1 angle
            C1 = sqrt(1-(X/R)^2);
            D1 = X/R;
            if(input(2) > 0)
                theta1 = atan2d(C1, D1);
            else
                theta1 = atan2d(-C1, D1);
            end

            %Joint 2 angle
            S = Z - 95;
            D2 = (a1^2 +R^2 +S^2-a2^2)/(2*a1*sqrt(R^2+S^2));
            D2a = R/(sqrt(R^2+S^2));
            C2 = sqrt(1-D2^2);
            if(S > 0)
                alpha = atan2d(-sqrt(1-D2a^2), D2a);
                beta = atan2d(C2, D2);
                theta2 = alpha - beta + 90;
            else
                alpha = atan2d(-sqrt(1-D2a^2), D2a);
                beta = atan2d(-C2, D2);
                theta2 = 90 - (alpha - beta);
            end
            

            %Joint 3 angle
            D3 = - ((a1^2 + a2^2 -(R^2+S^2))/(2*a1*a2));
            C3 = sqrt(1-D3^2);
            theta3 = atan2d(C3, D3)-90;
            
            %Checks bounds
            %if(theta1 <= -90 || theta1 >= 90 || theta2<=45 || theta2>=100 || theta3 <=-90 || theta3 >= 63)
                %bot.shutdown()
             %   error("beyond joint limits")
            %end

           %Return value
           T = [theta1, theta2, theta3];
        end

        function T = dh2mat(bot, input) %DH calc
            vals = [input(1), input(2), input(3), input(4)];
            T = [cosd(vals(1)) -sind(vals(1))*cosd(vals(4)) sind(vals(1))*sind(vals(4)) vals(3)*cosd(vals(1));
                sind(vals(1)) cosd(vals(1))*cosd(vals(4)) -cosd(vals(1))*sind(vals(4)) vals(3)*sind(vals(1));
                0 sind(vals(4)) cosd(vals(4)) vals(2);
                0 0 0 1];
        end

        function T = dh2fk(bot, input)
            T = dh2mat(bot, input(1, :));
            m = size(input);
            for n = 2:m(1)
                T = T * dh2mat(bot, input(n, :));
            end
        end

        function T = fdk3001(bot, angles, velocities)
            T = bot.jacob3001(angles) * velocities';
        end

        function T = fk3001(bot, input)
            DH1 = [input(1), 95, 0, -90];
            DH2 = [input(2)-90, 0, 100, 0];
            DH3 = [input(3)+90, 0, 100, 0];
            T = [dh2fk(bot, [DH1; DH2; DH3])];
        end

        function plot_arm(bot, q)
            base = [0 0 0];
            baseMat = dh2mat(bot, [base 0]);
            link1 = [0 0 55];
            link1Mat = dh2mat(bot, [link1 0]);
            link2Mat = dh2mat(bot, [q(1), 40, 0, -90]);
            link2 = [link2Mat(1,4) link2Mat(2,4) link2Mat(3,4)+55];
            link3Mat = link2Mat*dh2mat(bot, [q(2)-90, 0, 100, 0]);
            link3 = [link3Mat(1,4) link3Mat(2,4) link3Mat(3,4)+55];
            eeMat = fk3001(bot, q);
            ee = [eeMat(1,4) eeMat(2,4) eeMat(3,4)];
            posMatrix = [base;link1;link2;link3;ee];
            plot3(posMatrix(:,1), posMatrix(:,2), posMatrix(:,3), "-")
            hold on;
            plot3(posMatrix(:,1), posMatrix(:,2), posMatrix(:,3), ".")
            xlim([-90 150])
            ylim([-90 100])
            zlim([0 250])

            quiver3(0,0,0, baseMat(1,1), baseMat(2,1),baseMat(3,1),30, 'r')
            quiver3(0,0,0, baseMat(2,1), baseMat(2,2),baseMat(2,3),30, 'g')

            quiver3(0,0,55, link1Mat(1,1), link1Mat(2,1), link1Mat(3,1), 30, 'r')
            quiver3(0,0,55, link1Mat(2,1), link1Mat(2,2), link1Mat(2,3), 30, 'g')

            quiver3(link2Mat(1,4), link2Mat(2,4), link2Mat(3,4)+55, link2Mat(1,1), link2Mat(2,1), link2Mat(3,1), 30, 'r')
            quiver3(link2Mat(1,4), link2Mat(2,4), link2Mat(3,4)+55, link2Mat(2,1), link2Mat(2,2), link2Mat(3,3), 60, 'g')

            quiver3(link3Mat(1,4), link3Mat(2,4), link3Mat(3,4)+55, link3Mat(1,1), link3Mat(2,1), link3Mat(3,1), 30, 'r')
            quiver3(link3Mat(1,4), link3Mat(2,4), link3Mat(3,4)+55, link3Mat(2,1), link3Mat(2,2), link3Mat(2,3), 60, 'g')

            quiver3(eeMat(1,4),eeMat(2,4),eeMat(3,4), eeMat(1, 1), eeMat(2, 1), eeMat(3, 1), 30, 'r')
            quiver3(eeMat(1,4),eeMat(2,4),eeMat(3,4), eeMat(2, 1), eeMat(2, 2), eeMat(2, 3), 60, 'g')
            hold off;
        end


        function posMatrix = arm_position(bot, q)
            link1 = [0 0 55];
            T = dh2mat(bot, [q(1), 40, 0, -90]);
            link2 = [T(1,4) T(2,4) T(3,4)+55];
            T = T*dh2mat(bot, [q(2)-90, 0, 100, 0]);
            link3 = [T(1,4) T(2,4) T(3,4)+55];
            T = fk3001(bot, q);
            ee = [T(1,4) T(2,4) T(3,4)];
            posMatrix = [link1;link2;link3;ee];
        end

        %Takes in an array of position values and an interpolation time
        function interpolate_jp(bot, time, array)
            packet(1) = time; %Interpolation time
            packet(2) = 0;
            packet(3) = array(1);%Joint 1 position
            packet(4) = array(2);%Joint 2 position
            packet(5) = array(3);%Joint 3 position
            bot.write(1848, packet)
            bot.finalPosition = [array(1) array(2) array(3)];%set final position variable after move
        end
        
        %Takes in an array of position values only and moves to that
        %position
        function servo_jp(bot, degree_vals)
                %packet = zeros(15, 1, 'single');
                packet(1) = 0;
                packet(2) = 0;
                packet(3) = degree_vals(1,1);%Joint 1 position
                packet(4) = degree_vals(1,2);%Joint 2 position
                packet(5) = degree_vals(1,3);%Joint 3 position
                bot.write(1848, packet)
                %bot.read(1910);
                bot.finalPosition = [degree_vals(1) degree_vals(2) degree_vals(3)];%set final position variable after move
        end
        
        %Reads the current position of the robot joints
        function data = setpoint_cp(bot)
            data = [fk3001(bot, setpoint_js(bot))];
        end

        function array = setpoint_js(bot)
            packet = bot.read(1910);
            x = packet(3);%Joint 1 position value
            y = packet(5);%Joint 2 position value
            z = packet(7);%Joint 3 position value
            array = [x+100, y, z+195];%Set final array
        end
        
        function data = measured_cp(bot)
            data = [fk3001(bot, measured_js(bot))];
        end

        %Reads current position and velocity of the robot joints
        function data = measured_js(bot, GETPOS, GETVEL)
            xpos = 0;
            ypos = 0;
            zpos = 0;
            xvel = 0;
            yvel = 0;
            zvel = 0;
            
            if(GETPOS == true && GETVEL == true)
                posPacket = bot.read(1910);
                velPacket = bot.read(1822);

                xpos = posPacket(3);%Joint 1 position value
                ypos = posPacket(5);%Joint 2 position value
                zpos = posPacket(7);%Joint 3 position value

                xvel = velPacket(3);%Joint 1 velocity value
                yvel = velPacket(6);%Joint 2 velocity value
                zvel = velPacket(9);%Joint 3 velocity value
            end

            if(GETPOS == true && GETVEL == false)
                posPacket = bot.read(1910);
                xpos = posPacket(3);%Joint 1 position value
                ypos = posPacket(5);%Joint 2 position value
                zpos = posPacket(7);%Joint 3 position value
            end

            if(GETPOS == false && GETVEL == true)
                velPacket = bot.read(1822);
                xvel = velPacket(3);%Joint 1 velocity value
                yvel = velPacket(6);%Joint 2 velocity value
                zvel = velPacket(9);%Joint 3 velocity value
            end

            data = [xpos ypos zpos ; xvel yvel zvel];%set the final arrray
        end

        %simply prints the final position value
        function data = goal_cp(bot)
            data = [fk3001(bot, goal_js(bot))];
        end

        function position = goal_js(bot)
            position = bot.finalPosition;
        end

        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
             self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(self, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(self, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(0);
        end

        function moveToBallPoint(self, points, points2)
            %Zero the arm and open gripper 
            self.interpolate_jp(3000, [0 0 0]);
            pause(3);
            self.openGripper()
            pause(1);
            %Move so the arm is directly above the ball
            self.interpolate_jp(1000, [points(1)-10 0 0]);
            pause(1);
            %Pass the ball and come back to grab it from the right side
            self.interpolate_jp(3000, [points(1)-10 points(2) points(3)]);
            pause(3);
            self.closeGripper()
            pause(1);
            %Zero the arm after ball in hand
            self.interpolate_jp(3000, [0 0 0]);
            pause(3);
            %Go directly above the drop point
            self.interpolate_jp(1000, [points2(1) 0 0]);
            pause(1);
            %Drop at the point
            self.interpolate_jp(3000, points2);
            pause(3);
            self.openGripper()
            pause(1);
            %Zero again
            self.interpolate_jp(1000, [0 0 0]);
            pause(1);

        end

        
    end
end
