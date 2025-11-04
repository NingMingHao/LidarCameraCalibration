%% ROS2 LiDAR–Camera Pair Extractor (PNG + PCD) for your folder layout
% Requires Robotics System Toolbox (ros2bagreader, rosReadImage, rosReadXYZ)
clear; clc; close all

%% -------------------- Config --------------------
parent_folder_path = '/Users/minghao/Documents/UWaterloo/Projects/autobus/orange_bus_calibration_2025_11_04';

cam_id   = 0;                   % 0..5 (front-left -> rear-left, clockwise)
lidar_id = 'front';             % {'front','BP_F','BP_R'}

% Matching & saving
downsample_rate   = 5;          % keep 1 of every N matched pairs
match_tolerance_s = 0.06;       % max time difference (sec) allowed between camera & lidar

%% -------------------- Entry point --------------------
bags_root = fullfile(parent_folder_path, 'Bags');
out_root  = fullfile(parent_folder_path, 'paired_results');


name = sprintf('cam%d_%s', cam_id, lidar_id);
bag_dir = fullfile(bags_root, name);

save_path = fullfile(out_root, name)

if ~isfolder(bag_dir)
    error('Bag folder not found: %s', bag_dir);
end
fprintf('\n=== Processing %s ===\n', name);
run_one_pair(bag_dir, out_root, cam_id, lidar_id, downsample_rate, match_tolerance_s);


fprintf('\nAll done.\n');

%% -------------------- Worker: one pair --------------------
function run_one_pair(bag_dir, out_root, cam_id, lidar_id, downsample_rate, match_tolerance_s)
    % Map topics
    camera_topic = sprintf('/sensing/camera/camera%d/image_raw_color/compressed', cam_id);
    switch lidar_id
        case 'front'
            lidar_topic = '/rslidar_points_front';
        case 'BP_F'
            lidar_topic = '/rslidar_points_BP_F';
        case 'BP_R'
            lidar_topic = '/rslidar_points_BP_R';
        otherwise
            error('Unknown lidar_id: %s (use front, BP_F, BP_R)', lidar_id);
    end

    % Output dirs (exact structure you asked for)
    pair_name      = sprintf('cam%d_%s', cam_id, lidar_id);
    save_path_pair = fullfile(out_root, pair_name);
    image_dir      = fullfile(save_path_pair, 'Images');
    pc_dir         = fullfile(save_path_pair, 'PointClouds');
    req_dirs = {image_dir, pc_dir};
    for i = 1:numel(req_dirs)
        if ~exist(req_dirs{i}, 'dir'), mkdir(req_dirs{i}); end
    end

    % Open bag
    bag = ros2bagreader(bag_dir);

    % Topic presence check
    allTopics = unique(bag.AvailableTopics.Properties.RowNames);
    assert(any(strcmp(allTopics, camera_topic)), 'Camera topic missing: %s', camera_topic);
    assert(any(strcmp(allTopics, lidar_topic)),  'LiDAR topic missing: %s',  lidar_topic);

    selCam   = select(bag, "Topic", camera_topic);
    selLidar = select(bag, "Topic", lidar_topic);

    camTbl   = selCam.MessageList;
    lidarTbl = selLidar.MessageList;

    if isempty(camTbl) || isempty(lidarTbl)
        warning('No messages: camera %d, lidar %d. Skipping.\n', height(camTbl), height(lidarTbl));
        return;
    end

    % ----- Robust time conversion (handles duration OR datetime) -----
    camSec   = to_posix_seconds(camTbl.Time);
    lidarSec = to_posix_seconds(lidarTbl.Time);


    % Greedy, order-preserving nearest-neighbor matching with tolerance
    if numel(lidarSec) >= numel(camSec)
        [idx_cam, idx_lidar] = match_by_time(camSec, lidarSec, match_tolerance_s);
    else
        [idx_lidar, idx_cam] = match_by_time(lidarSec, camSec, match_tolerance_s);
    end
    pairs = [idx_cam(:), idx_lidar(:)];
    if isempty(pairs)
        warning('No matched pairs within %.3fs for %s. Skipping.', match_tolerance_s, pair_name);
        return;
    end
    fprintf('Matched %d pairs (<= %.3fs) for %s\n', size(pairs,1), match_tolerance_s, pair_name);

    for k = 1:downsample_rate:size(pairs,1)
        cRow = pairs(k,1);
        lRow = pairs(k,2);

        camMsgCell   = readMessages(selCam,   cRow);   % was: readMessages(selCam,'Indices',cRow,'DataFormat','struct')
        lidarMsgCell = readMessages(selLidar, lRow);   % was: readMessages(selLidar,'Indices',lRow,'DataFormat','struct')

        camMsg   = camMsgCell{1};
        lidarMsg = lidarMsgCell{1};

        % Decode
        I   = rosReadImage(camMsg);     % CompressedImage -> RGB uint8
        XYZ = rosReadXYZ(lidarMsg);     % PointCloud2    -> Nx3 double
        if isempty(XYZ), continue; end

        stem  = sprintf('%04d', k);

        imwrite(I, fullfile(image_dir, [stem '.png']));
        pcwrite(pointCloud(XYZ), fullfile(pc_dir, [stem '.pcd']));
    end

    fprintf('Saved image/pcd pairs to:\n  %s\n  %s\n', image_dir, pc_dir);
end

%% -------------------- Time conversion helper --------------------
function secs = to_posix_seconds(tcol)
% Normalizes MessageList.Time to a numeric seconds vector for matching.
% Handles: duration (offset), datetime (absolute), or numeric (offset).
    if isduration(tcol)
        secs = seconds(tcol);
    elseif isdatetime(tcol)
        tt = tcol;
        if isempty(tt.TimeZone), tt.TimeZone = 'UTC'; end
        secs = posixtime(tt);
    elseif isnumeric(tcol)  % e.g., double seconds since bag start
        secs = double(tcol);
    else
        error('Unsupported Time column type: %s', class(tcol));
    end
end



%% -------------------- Matching helper --------------------
function [idxA, idxB] = match_by_time(timeA, timeB, tol)
% Match each time in A to nearest in B within tol seconds.
% Returns order-preserving pairs of indices into A and B.
    nA = numel(timeA); nB = numel(timeB);
    i = 1; j = 1;
    idxA = []; idxB = [];
    while i <= nA && j <= nB
        dt = timeA(i) - timeB(j);
        if abs(dt) <= tol
            idxA(end+1) = i; %#ok<AGROW>
            idxB(end+1) = j; %#ok<AGROW>
            i = i + 1; j = j + 1;
        else
            if timeA(i) < timeB(j)
                i = i + 1;
            else
                j = j + 1;
            end
        end
    end
end

