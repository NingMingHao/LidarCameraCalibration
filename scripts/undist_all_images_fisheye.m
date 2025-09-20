% Paths

imageFilesPath = fullfile(save_path, 'Images');
undistImageFilesPath = fullfile(save_path, 'UndistImages');
if ~exist(undistImageFilesPath, 'dir')
    mkdir(undistImageFilesPath);
end

% Get a list of all files in the source folder
file_list = dir(fullfile(imageFilesPath, '*.png'));
for i = 1:length(file_list)
    filename = file_list(i).name;
    filepath = fullfile(imageFilesPath, filename);
    
    % Read the distorted image
    I = imread(filepath);

    % Undistort image using fisheye intrinsics
    [J, undistortedIntrinsics] = undistortFisheyeImage(I, cameraParams.Intrinsics, 'OutputView', 'same', 'ScaleFactor',0.8);

    % Save the undistorted image
    imwrite(J, fullfile(undistImageFilesPath, filename));
end

% Create an "undistorted" pinhole intrinsics approximation for ROS compatibility
imageSize = cameraParams.Intrinsics.ImageSize;

% Create a new cameraParams object with zero distortion coefficients
undistCameraParams = cameraParameters("K", undistortedIntrinsics.K,...
                                      'ImageSize', imageSize);
% Save the new cameraParams to a .mat file
undistCameraParamsPath = fullfile(save_path,'undistCameraParams.mat');
save(undistCameraParamsPath, 'undistCameraParams');