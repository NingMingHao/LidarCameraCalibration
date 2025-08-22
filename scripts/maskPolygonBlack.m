% use camera parameters to undistort images

save_path = '/Users/minghao/Documents/UWaterloo/Projects/indoor/IndoorCalibration2025-04-23/paired_results/node1/front_right';
imageFilesPath = fullfile(save_path,'Images');
undistImageFilesPath = fullfile(save_path, "blackImages");
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

    % Get the output filename
    output_filename = fullfile(undistImageFilesPath, filename);
    
    imwrite(maskPolygonBlackFunc(I), output_filename);
end


function I_masked = maskPolygonBlackFunc(I)
% maskPolygonBlack - Sets a polygon region in the image to black
%
% Inputs:
%   I          - Input image (grayscale or RGB)
%
% Output:
%   I_masked   - Output image with polygon region set to black

    % Create polygon mask
    % Node 1 right
    % polygon_x = [1454, 1771, 1891, 1491];
    % polygon_y = [968, 999, 1200, 1200];

    % Node 1 left
    % polygon_x = [780, 1360, 1400, 1150];
    % polygon_y = [0, 0, 280, 330];

    % Node 1 right raw
    % polygon_x = [1454, 1920, 1920, 1454];
    % polygon_y = [0, 0, 220, 130];

    % % Node 1 left raw
    % polygon_x = [45, 560, 615, 354];
    % polygon_y = [466, 220, 520, 988];


    % Node 1 left raw
    polygon_x = [1350, 1350, 1500, 1500];
    polygon_y = [85,   137,  142, 81];

    mask = poly2mask(polygon_x, polygon_y, size(I, 1), size(I, 2));

    % Apply mask to set polygon region to black
    if size(I, 3) == 3  % RGB image
        I_masked = I;
        for c = 1:3
            channel = I(:,:,c);
            channel(mask) = 0;
            I_masked(:,:,c) = channel;
        end
    else  % Grayscale image
        I_masked = I;
        I_masked(mask) = 0;
    end
end