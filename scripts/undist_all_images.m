% save_path = '/Users/minghao/Documents/UWaterloo/Projects/outdoor/Calibration2508/paired_results/node1/front_left';
imageFilesPath = fullfile(save_path,'Images');
undistImageFilesPath = fullfile(save_path, "UndistImages");
if ~exist(undistImageFilesPath,'dir')
    mkdir(undistImageFilesPath);
end

% Get a list of all files in the source folder
file_list = dir(fullfile(imageFilesPath, '*.png')); 

% Undistort first image to determine new intrinsics
if isempty(file_list)
    error('No PNG images found in %s', imageFilesPath);
end

% Read one sample image
I = imread(fullfile(imageFilesPath, file_list(1).name));

% Undistort with 'full' view and get the new intrinsics
[I_undistorted_sample, newIntrinsics] = undistortImage(I, cameraParams, 'OutputView','full');

% Save all undistorted images using the same intrinsics
for i = 1:length(file_list)
    filename = file_list(i).name;
    filepath = fullfile(imageFilesPath, filename);
    
    I = imread(filepath);
    I_undistorted = undistortImage(I, cameraParams, 'OutputView','full');
    
    output_filename = fullfile(undistImageFilesPath, filename);
    imwrite(I_undistorted, output_filename);
end

% Create new cameraParams object with zero distortion and new intrinsics
undistCameraParams = cameraParameters( ...
    'IntrinsicMatrix', newIntrinsics.IntrinsicMatrix, ...
    'RadialDistortion', [0,0], ...
    'TangentialDistortion', [0,0], ...
    'WorldPoints', cameraParams.WorldPoints, ...
    'ImageSize', newIntrinsics.ImageSize, ...
    'NumRadialDistortionCoefficients', cameraParams.NumRadialDistortionCoefficients, ...
    'RotationVectors', cameraParams.RotationVectors, ...
    'TranslationVectors', cameraParams.TranslationVectors);

% Save the undistorted camera parameters
undistCameraParamsPath = fullfile(save_path,'undistCameraParams.mat');
save(undistCameraParamsPath, 'undistCameraParams');