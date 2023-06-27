load('calibrationSession.mat')

intrinsics = calibrationSession.CameraParameters.Intrinsics;
% Save the camera intrinsics to a YAML file
yamlFile = 'front_left_camera_intrinsics.yaml';


% Open the YAML file for writing
fid = fopen(yamlFile, 'w');
if fid == -1
    error('Failed to open the YAML file for writing.');
end

% Write the camera intrinsics to the YAML file
fprintf(fid, 'image_width: %d\n', intrinsics.ImageSize(2));
fprintf(fid, 'image_height: %d\n', intrinsics.ImageSize(1));
fprintf(fid, 'camera_name: center_cam\n');

% Write the camera matrix
fprintf(fid, 'camera_matrix:\n');
fprintf(fid, '  rows: 3\n');
fprintf(fid, '  cols: 3\n');
fprintf(fid, '  data: [%.6f, 0.000000, %.6f, 0.000000, %.6f, %.6f, 0.000000, 0.000000, 1.000000]\n', ...
    intrinsics.FocalLength(1), intrinsics.PrincipalPoint(1), ...
    intrinsics.FocalLength(2), intrinsics.PrincipalPoint(2));

% Write the distortion model and coefficients
fprintf(fid, 'distortion_model: plumb_bob\n');
fprintf(fid, 'distortion_coefficients:\n');
fprintf(fid, '  rows: 1\n');
fprintf(fid, '  cols: 5\n');
fprintf(fid, '  data: [%.6f, %.6f, %.6f, %.6f, %.6f]\n', ...
    intrinsics.RadialDistortion(1), intrinsics.RadialDistortion(2), ...
    intrinsics.TangentialDistortion(1), intrinsics.TangentialDistortion(2), ...
    0);

% Write the rectification matrix
fprintf(fid, 'rectification_matrix:\n');
fprintf(fid, '  rows: 3\n');
fprintf(fid, '  cols: 3\n');
fprintf(fid, '  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]\n');

% Write the projection matrix
fprintf(fid, 'projection_matrix:\n');
fprintf(fid, '  rows: 3\n');
fprintf(fid, '  cols: 4\n');
fprintf(fid, '  data: [%.6f, %.6f, %.6f, 0.000000, %.6f, %.6f, %.6f, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]\n', ...
    intrinsics.K(1, 1), intrinsics.K(1, 2), intrinsics.K(1, 3), ...
    intrinsics.K(2, 1), intrinsics.K(2, 2), intrinsics.K(2, 3));

% Close the YAML file
fclose(fid);