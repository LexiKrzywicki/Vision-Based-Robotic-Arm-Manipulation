classdef detection 
    properties
    end


    methods(Static)

        function [redPoints, orangePoints, yellowPoints, greenPoints, bluePoints] = detect()
            load 'camera.mat';
            
            try
                % Set up camera
                if cam.params == 0
                    error("No camera parameters found!");
                end
                
                % Get the calibration image
                im = cam.getImage();
                
                %Add the red mask and return the location of the red ball
                [bw1, imCoin1] = redMaskHSV(im);
                bw1 = bwareafilt(bw1, [1000 4000]);
                [redPoints, radius] = imfindcircles(bw1,[20 60]);
    
                %The following is used for live tracking since it allows a
                %worse image
                %imageProps = regionprops(bw1);
                %redPoints = imageProps.Centroid;
                
                %Add the orange mask and return the location of the orange ball
                [bw2, imCoin2] = orangeMaskHSV(im);
                bw2 = bwareafilt(bw2, [1000 4000]);
                [orangePoints, radius] = imfindcircles(bw2,[20 60]);
                   
                %Add the yellow mask and return the location of the yellow ball
                [bw3, imCoin3] = yellowMaskHSV(im);
                bw3 = bwareafilt(bw3, [1000 4000]);
                [yellowPoints, radius] = imfindcircles(bw3,[20 60]);
            
                %Add the green mask and return the location of the green ball
                [bw4, imCoin4] = greenMaskHSV(im);
                bw4 = bwareafilt(bw4, [1000 4000]);
                [greenPoints, radius] = imfindcircles(bw4,[20 60]);
            
                %Add the blue mask and return the location of the blue cap
                [bw5, imCap] = blueMaskHSV(im);
                bw5 = bwareafilt(bw5, [1000 4000]);
                [bluePoints, radius] = imfindcircles(bw5, [20 60]);
            catch exception
                fprintf('\n ERROR!!! \n \n');
                disp(getReport(exception));
                disp('Exited on error, clean shutdown');
            end
        
                %% Shutdown Procedure
                cam.shutdown()
            end        
    end
end
