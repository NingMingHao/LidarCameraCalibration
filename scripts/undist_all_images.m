% use camera parameters to undistort images

save_path = '/home/minghao/Documents/UWaterloo/Projects/WhiteBus/sensorcalib_bus_new/paired_results/back_right';
imageFilesPath = fullfile(save_path,'Images');
undistImageFilesPath = fullfile(save_path, "UndistImages");
if ~exist(undistImageFilesPath,'dir')
    mkdir(undistImageFilesPath);
end


% Get a list of all files in the source folder
file_list = dir(fullfile(imageFilesPath, '*.png')); 

% Loop through each image file in the source folder
for i = 1:length(file_list)
    % Get the filename
    filename = file_list(i).name;
    filepath = fullfile(imageFilesPath, filename);
    
    % Read the image
    I = imread(filepath);
    
    % Undistort the image
    I_undistorted = undistortImage(I, cameraParams);
    
    % Get the output filename
    output_filename = fullfile(undistImageFilesPath, filename);
    
    % Save the undistorted image to the destination folder
    imwrite(I_undistorted, output_filename);
end

% Get camera intrinsic parameters
focalLength = cameraParams.Intrinsics.FocalLength;
principalPoint = cameraParams.Intrinsics.PrincipalPoint;
imageSize = cameraParams.Intrinsics.ImageSize;

% Create a new cameraIntrinsics object
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Create a new cameraParams object with zero distortion coefficients
undistCameraParams = cameraParameters('IntrinsicMatrix', intrinsics.IntrinsicMatrix,...
                                      'RadialDistortion', [0,0],...
                                      'TangentialDistortion', [0,0],...
                                      'WorldPoints', cameraParams.WorldPoints,...
                                      'ImageSize', cameraParams.ImageSize,...
                                      'NumRadialDistortionCoefficients', cameraParams.NumRadialDistortionCoefficients,...
                                      'RotationVectors', cameraParams.RotationVectors,...
                                      'TranslationVectors', cameraParams.TranslationVectors);
% Save the new cameraParams to a .mat file
undistCameraParamsPath = fullfile(save_path,'undistCameraParams.mat');
save(undistCameraParamsPath, 'undistCameraParams');