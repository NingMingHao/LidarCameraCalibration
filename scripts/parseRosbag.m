clear; clc; close all

rosbag_path = '/home/minghao/Documents/UWaterloo/Projects/WhiteBus/sensorcalib_bus_new/back_right/2023-06-19-15-43-57.bag';
save_path = '/home/minghao/Documents/UWaterloo/Projects/WhiteBus/sensorcalib_bus_new/paired_results/back_right';
bag = rosbag(rosbag_path);

imageBag = select(bag,'Topic','/usb_cam_right/image_raw/compressed');
pcBag = select(bag,'Topic','/rslidar_points_BP_F');

downsample_rate = 10;
imageMsgs = readMessages(imageBag);
pcMsgs = readMessages(pcBag);

% To prepare data for lidar camera calibration, the data across both the sensors 
% must be time-synchronized. Create timeseries (ROS Toolbox) objects for the selected
% topics and extract the timestamps.
ts1 = timeseries(imageBag);
ts2 = timeseries(pcBag);
t1 = ts1.Time;
t2 = ts2.Time;

% find the best match
k = 1;
if size(t2,1) > size(t1,1)
    for i = 1:size(t1,1)
        [val,indx] = min(abs(t1(i) - t2));
        if val <= 0.1
            idx(k,:) = [i indx];
            k = k + 1;
        end
    end
else
    for i = 1:size(t2,1)
        [val,indx] = min(abs(t2(i) - t1));
        if val <= 0.1
            idx(k,:) = [indx i];
            k = k + 1;
        end
    end
end

% Create directories to save the valid images and point clouds.
pcFilesPath = fullfile(save_path,'PointClouds');
imageFilesPath = fullfile(save_path,'Images');
if ~exist(imageFilesPath,'dir')
    mkdir(imageFilesPath);
end
if ~exist(pcFilesPath,'dir')
    mkdir(pcFilesPath);
end


% Extract the images and point clouds. Name and save the files in their respective folders. 
% Save corresponding image and point clouds under the same number.

for i = 1:downsample_rate:length(idx)
    I = readImage(imageMsgs{idx(i,1)});
    pc = pointCloud(readXYZ(pcMsgs{idx(i,2)}));
    n_strPadded = sprintf('%04d',i) ;
    pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    imageFileName = strcat(imageFilesPath,'/',n_strPadded,'.png');
    imwrite(I,imageFileName);
    pcwrite(pc,pcFileName);
end
