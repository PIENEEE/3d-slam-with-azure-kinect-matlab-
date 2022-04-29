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
figure, h1 = imshow(depth,[0 outOfRange]);
title('Depth Source (close figure to exit)')
colormap('Jet')
colorbar

% point cloud figure
pcFig.h = figure;
pcFig.ax = pcshow(pc);

disp('Close any figure to exit')
downsample = 200; % subsample pointcloud
i = 1;
% Main Loop
while true

    validData = kz.getframes('color','depth');
    
    if validData
        
        depth = kz.getdepth;
        
        try
            set(h1,'CData',depth); 
        catch
            break; % break the main loop 
        end
          
        pc = kz.getpointcloud('output','pointCloud','color','true');
        c{i}= pc;
        i = i + 1;
        if i == 230
             save('slamming46.mat','c','-v7.3');
         break;
        end
        % Display the point cloud,
        % if the user closes the window, the program ends
        try
            pcshow(pc,'Parent',pcFig.ax,'VerticalAxis','Y');
            title(pcFig.ax,'Point Cloud');
            xlabel(pcFig.ax,'X'); ylabel(pcFig.ax,'Y'); zlabel(pcFig.ax,'Z');
            %axis(pcFig.ax,[-4000 4000 -4000 4000 -4000 4000]);
            axis(pcFig.ax,[-1000 1000 -1000 1000 -1000 1000]);
        catch
            break; % break the main loop
        end
    end
   
    %pause(0.02);
    pause(0.001); %노트북용
end

% Close kinect object
k2.delete;