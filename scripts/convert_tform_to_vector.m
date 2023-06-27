load('tform.mat')

% Save the camera intrinsics to a YAML file
yamlFile = 'front_lidar_to_front_right_cam.yaml';

% Open the YAML file for writing
fid = fopen(yamlFile, 'w');
if fid == -1
    error('Failed to open the YAML file for writing.');
end

invert_tform = invert(tform);
rotation = invert_tform.Rotation';
translation = invert_tform.Translation;
% Write the camera intrinsics to the YAML file
fprintf(fid, 'P_camera = R*P_lidar + t \n');
% Write the projection matrix
fprintf(fid, 'rotation matrix R for verification:\n');
fprintf(fid, '  rows: 3\n');
fprintf(fid, '  cols: 3\n');
fprintf(fid, '  data: [%.6f, %.6f, %.6f, \n %.6f, %.6f, %.6f, \n %.6f, %.6f, %.6f]\n \n', ...
    rotation(1, 1), rotation(1, 2), rotation(1, 3), ...
    rotation(2, 1), rotation(2, 2), rotation(2, 3), ...
    rotation(3, 1), rotation(3, 2), rotation(3, 3));

fprintf(fid, 'ROS: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms \n');
fprintf(fid, '(yaw is rotation about Z, pitch is rotation about Y, and roll is rotation about X) in radians \n');
fprintf(fid, 'The frame_id is the lidar frame, and the child_frame_id is the camera frame.  \n \n');

fprintf(fid, 'translation x y z: %.6f %.6f %.6f\n', translation(1), translation(2), translation(3));

% Write euler angle
% static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
% (yaw is rotation about Z, pitch is rotation about Y, and roll is rotation
% about X) in radians

eulZYX = rotm2eul(rotation);
fprintf(fid, 'rotation angle in radians (yaw pitch roll):%.6f %.6f %.6f\n', eulZYX(1), eulZYX(2), eulZYX(3) );

% Close the YAML file
fclose(fid);
