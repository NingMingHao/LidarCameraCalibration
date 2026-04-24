clear; clc; close all

parent_folder_path = '/home/minghao/Documents/Gits/OutdoorNodeFu/2026_04_22_17_58_calib';
node_number = 88;
selected_camera = 'center';
enable_pointcloud_stream = false;
target_image_height = [];
target_image_width = [];

% Build paths dynamically
rosbag_path = sprintf('%s/Bags/merged_node%d.bag', parent_folder_path, node_number);
save_path   = sprintf('%s/paired_results/node%d/%s', parent_folder_path, node_number, selected_camera);

% Load rosbag
bag = rosbag(rosbag_path);

% Build topic string dynamically
topic_name = sprintf('/camera/%s/image_raw/compressed', selected_camera);
% Select the image topic from bag
imageBag = select(bag, 'Topic', topic_name);

downsample_rate = 5;
imageMsgs = readMessages(imageBag);

enable_image_resize = ~isempty(target_image_height) && ~isempty(target_image_width);
if enable_image_resize
    if target_image_height <= 0 || target_image_width <= 0
        error('target_image_height and target_image_width must be positive when resizing is enabled.');
    end
end

if enable_pointcloud_stream
    pcBag = select(bag,'Topic','/rslidar_points_front');
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
else
    idx = (1:numel(imageMsgs))';
end

% Create directories to save the valid images and point clouds.
imageFilesPath = fullfile(save_path,'Images');
if ~exist(imageFilesPath,'dir')
    mkdir(imageFilesPath);
end
if enable_pointcloud_stream
    pcFilesPath = fullfile(save_path,'PointClouds');
    if ~exist(pcFilesPath,'dir')
        mkdir(pcFilesPath);
    end
end


% Extract the images and point clouds. Name and save the files in their respective folders. 
% Save corresponding image and point clouds under the same number.

for i = 1:downsample_rate:length(idx)
    I = readImage(imageMsgs{idx(i,1)});
    if enable_image_resize
        I = imresize(I, [target_image_height, target_image_width]);
    end
    n_strPadded = sprintf('%04d',i) ;
    imageFileName = strcat(imageFilesPath,'/',n_strPadded,'.png');
    imwrite(I,imageFileName);
    if enable_pointcloud_stream
        pc = pointCloud(readXYZ(pcMsgs{idx(i,2)}));
        pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
        pcwrite(pc,pcFileName);
    end
end
