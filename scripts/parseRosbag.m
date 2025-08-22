clear; clc; close all

parent_folder_path = '/Users/minghao/Documents/UWaterloo/Projects/outdoor/Calibration2508';
node_number = 1;
selected_camera = 'right';

% Build paths dynamically
rosbag_path = sprintf('%s/Bags/merged_node%d.bag', parent_folder_path, node_number);
save_path   = sprintf('%s/paired_results/node%d/front_%s', parent_folder_path, node_number, selected_camera);

% Load rosbag
bag = rosbag(rosbag_path);

% Build topic string dynamically
topic_name = sprintf('/pylon_camera_node_%s/image_raw/compressed', selected_camera);
% Select the image topic from bag
imageBag = select(bag, 'Topic', topic_name);
pcBag = select(bag,'Topic','/rslidar_points_front');

downsample_rate = 2;
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
