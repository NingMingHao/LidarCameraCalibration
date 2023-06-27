save_path = '/home/minghao/Documents/UWaterloo/Projects/WhiteBus/sensorcalib_bus_new/paired_results/back_right';
imageFilesPath = fullfile(save_path,'UndistImages');
rawPcFilesPath = fullfile(save_path,'PointClouds');
cropPcFilesPath = fullfile(save_path, 'CropPointClouds');
if ~exist(cropPcFilesPath,'dir')
    mkdir(cropPcFilesPath);
end

%pcshow an example pointcloud to decide the roi
ptCloud = pcread(fullfile(rawPcFilesPath, '0441.pcd'))
pcshow(ptCloud);
xlabel('X');
ylabel('Y');
zlabel('Z');
%x y z
% roi = [0.5 6 -6 1 -0.4 2]; %front lidar
% roi = [-6 6 -6 6 -1 1.6];    %bp_r
roi = [-6 6 -6 6 -1 1.6];    %bp_f
indices = findPointsInROI(ptCloud,roi);
ptCloudB = select(ptCloud,indices);

figure
pcshow(ptCloud.Location,[0.5 0.5 0.5])
hold on
pcshow(ptCloudB.Location,'r');
legend('Point Cloud','Points within ROI','Location','southoutside','Color',[1 1 1])
hold off

% Get a list of all files in the source folder
file_list = dir(fullfile(imageFilesPath, '*.png')); 

% Loop through each image file in the source folder
for i = 1:length(file_list)
    % Get the filename
    filename = file_list(i).name;
    splitnames = strsplit(filename, '.');
    iname = splitnames(1);
    pcfilename = strcat(iname{1}, '.pcd');

    rawfilepath = fullfile(rawPcFilesPath, pcfilename);
    cropfilepath = fullfile(cropPcFilesPath, pcfilename);
    
    % load the pc
    rawpc = pcread(rawfilepath);
    indices = findPointsInROI(rawpc,roi);
    croppc = select(rawpc,indices);
    pcwrite(croppc,cropfilepath);
end


