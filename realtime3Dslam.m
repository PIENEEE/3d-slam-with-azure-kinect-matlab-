addpath('../Mex');
clear all
close all

% Create KinZ object and initialize it
% Available options: 
% '720p', '1080p', '1440p', '1535p', '2160p', '3072p'
% 'binned' or 'unbinned'
% 'wfov' or 'nfov'
kz = KinZ('720p', 'binned', 'nfov');

depthWidth = kz.DepthWidth; 
depthHeight = kz.DepthHeight; 
outOfRange = 2000;

% Create matrices for the images
depth = zeros(depthHeight,depthWidth,'uint16');
pc = pointCloud(zeros(depthHeight*depthWidth,3));

% depth stream figure
%figure, 
h1 = imshow(depth,[0 outOfRange]);
%title('Depth Source (close figure to exit)')
%colormap('Jet')
%colorbar

disp('Close any figure to exit')
%downsample = 2; % subsample pointcloud
downsample = 200;
i = 1;

for j = 1:2
    validData = kz.getframes('color','depth');

    if validData
        depth = kz.getdepth;
        pc = kz.getpointcloud('output','pointCloud','color','true');
        c{j} = pc;
    end
% point cloud figure
%pcFig.h = figure;
%pcFig.ax = pcshow(pc);
end
 ptCloudRef = c{1};
 ptCloudCurrent = c{2};

gridSize = 0.1;
%gridSize = 10;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize); 
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
ptCloudAligned = pctransform(ptCloudCurrent,tform);

%mergeSize = 0.015;
mergeSize = 7;
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);

% Store the transformation object that accumulates the transformation.
accumTform = tform; 

%figure
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

% Main Loop
while true
    % Get frames from Kinect and save them on underlying buffer
    validData = kz.getframes('color','depth');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices
        depth = kz.getdepth;
        
        % Display the depth image, 
        % if the user closes the window, the program ends
     %   try
     %       set(h1,'CData',depth); 
     %   catch
     %       break; % break the main loop 
     %   end
          
        % Get the pointcloud with color from the Kinect
        % Select the output 'pointCloud' to use the MATLAB built-in
        % pointCloud object. 
        % For MATLAB versions older than 2015b, use 'output','raw' and use
        % scatter3 to plot the point cloud. See pointCloudDemo1.m
        pc = kz.getpointcloud('output','pointCloud','color','true');
        
        ptCloudCurrent = pc;
        fixed = moving;
        moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

         % Apply ICP registration.
         tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);

         % Transform the current point cloud to the reference coordinate system
        % defined by the first point cloud.
         accumTform = affine3d(tform.T * accumTform.T);
         ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
    
         % Update the world scene.
         ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);

         % Visualize the world scene.
         hScatter.XData = ptCloudScene.Location(:,1);
         hScatter.YData = ptCloudScene.Location(:,2);
         hScatter.ZData = ptCloudScene.Location(:,3);
         hScatter.CData = ptCloudScene.Color;
         drawnow('limitrate')
        

        if i == 100
          break;
        end
        
    end
   
    %pause(0.01);
    pause(0.001);
    i = i + 1;
end
% During the recording, the Kinect was pointing downward. To visualize the
% result more easily, let's transform the data so that the ground plane is
% parallel to the X-Z plane.
angle = -pi/40;
A = [1,0,0,0;...
     0, cos(angle), sin(angle), 0; ...
     0, -sin(angle), cos(angle), 0; ...
     0 0 0 1];
ptCloudScene = pctransform(ptCloudScene, affine3d(A));
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
        'Parent', hAxes)
title('Updated world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

% Close kinect object
%k2.delete;