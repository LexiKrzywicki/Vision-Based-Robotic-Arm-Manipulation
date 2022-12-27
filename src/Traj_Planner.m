classdef Traj_Planner
    methods
        %Trajectory planning for a cubic polynomial
        function T = cubic_traj(self, tStart, tEnd, posStart, posEnd, velStart, velEnd)
            %Create matrix using the position and velocity polynomial formulas
            M = [1 tStart tStart^2 tStart^3;
                0 1 2*tStart 3*tStart^2;
                1 tEnd tEnd^2 tEnd^3;
                0 1 2*tEnd 3*tEnd^2];

            %Initial conditions vector
            B = [posStart;
                velStart;
                posEnd;
                velEnd];

            %Coefficients
            T = inv(M)*B;
        end

        %Trajectory planning for a quintic polynomial
        function T = quintic_traj(self, tStart, tEnd, posStart, posEnd, velStart, velEnd, accStart, accEnd)
            %Create matrix using the position, velocity, and acceleration polynomial formulas
            M = [1 tStart tStart^2 tStart^3 tStart^4 tStart^5; 
                0 1 2*tStart 3*tStart^2 4*tStart^3 5*tStart^4;
                0 0 2 6*tStart 12*tStart^2 20*tStart^3;
                1 tEnd tEnd^2 tEnd^3 tEnd^4 tEnd^5;
                0 1 2*tEnd 3*tEnd^2 4*tEnd^3 5*tEnd^4;
                0 0 2 6*tEnd 12*tEnd^2 20*tEnd^3];

            %Initial conditions vector    
            B = [posStart;
                velStart;
                accStart;
                posEnd;
                velEnd;
                accEnd];

            %Coefficients 
            T = inv(M)*B;  
        end
    end

end