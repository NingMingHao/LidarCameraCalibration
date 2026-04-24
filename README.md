# LidarCameraCalibration
Steps for using MATLAB for camera calibration (fisheye) and then do the lidar camera calibration via manually selecting the matched points.



It covers the following topics:
- [Single Camera Calibration](#single-camera-calibration)
- [Lidar Camera Calibration](#lidar-camera-calibration)




## Data Preprocessing
### Select Time Range and Merge as One Rosbag
Use the [merge_rosbag.py](./preprocess/merge_bag.py) script to merge the rosbag files. This script will also allow you to select a specific time range for the data you want to include in the merged bag.
Here you need to modify the following parameters:
- `SENSOR_ID`: a integer representing the sensor ID
- `BAG_DIR`: the directory containing the rosbag files
- `OUTPUT_BAG`: the output merged rosbag file
- `TIME_SLICES`: a list of tuples representing the time ranges to include (start_time, end_time)

You can find the time ranges via playing the bags and check if the checkerboard is visible in Rviz.
```bash
rosbag play *.bag --clock --skip-empty 2
```

### Data Organization
Folder Structure is as follows:

```bash
parent_folder_path
├── Bags
│   ├── merged_node1.bag
│   ├── merged_node2.bag
│   └── ...
└── paired_results (this folder is generated via the codes)
    ├── node1
    │   ├── front_left
    │   │   ├── Images (raw images)
    │   │   ├── PointClouds (raw point clouds) (only generated when `enable_pointcloud_stream` is set to true)
    │   │   ├── UndistImages (rectified images)
    │   │   ├── CropPointClouds (cropped point clouds in region of interest) (only generated when `enable_pointcloud_stream` is set to true)
    │   │   ├── calibrationSession.mat (Saved camera calibration session)
    │   │   ├── lccSession.mat (Saved lidar camera calibration session) (only generated when `enable_pointcloud_stream` is set to true)
    │   │   └──  ...
    │   └── front_right
    ├── node2
    └── ...
```

Use this [parseRosbag.m](./scripts/parseRosbag.m) to convert the rosbag file to png and pcd files. This will synchronize the lidar point cloud and camera image, and downsample the point cloud and image to the specified rate. This script is based on [this link](https://www.mathworks.com/help/lidar/ug/read-lidar-and-camera-data-from-rosbag.html).
Here you need to modify the following parameters:
- `parent_folder_path`: the folder for the whole project, like shown in the folder structure
- `node_number`: the node number (e.g. 1, 2, etc.)
- `selected_camera`: the selected camera (e.g. left or right.)
- `downsample_rate`: the downsample rate for the lidar point cloud and image, typically the lidar runs at 10Hz, it's not necessary to save all the point clouds and images.
- `enable_pointcloud_stream`: for this camera calibration case, we can set it to false. But for the case that there is both the camera and lidar data capturing the same checkerboard, you can set it to true and use the [Lidar Camera Calibration](#lidar-camera-calibration) part to calibrate the lidar and camera together.



## Single Camera Calibration
### Camera Calibration
- Launch the single camera calibration app
- Load the images from files, use `Checkerboard` as the pattern, and `90mm` as the square size, choose `Image distortion` as `High`
- Change the `Camear Model` from `Standard` to `Fisheye`, and make sure the `Estimate Alignment` option is **not checked**.
- Remove the unwanted images in `Data Browser`, and then `Calibrate`
- After the calibration, you can check the reprojection error, and then export the calibration result
- It's recommended to save the calibration session, so that you can load it later and check the reprojection error again


### Convert the Camera Calibration to ROS format

After saving the MATLAB fisheye calibration session, convert it to a ROS/OpenCV
`equidistant` camera YAML:

```bash
cd /path/to/paired_results/node88/center
matlab -batch "run('/home/minghao/Documents/Gits/OutdoorNodeFu/LidarCameraCalibration/scripts/convert_matlab_fisheye_to_ros_equdist.m')"
```

The script expects the following files/paths relative to the current folder:

- `calibrationSession.mat`: MATLAB camera calibration session or exported `cameraParams`
- `Images/0036.png`: optional example image for visual overlay and undistortion preview

Main settings to check in
[`convert_matlab_fisheye_to_ros_equdist.m`](./scripts/convert_matlab_fisheye_to_ros_equdist.m):

- `cameraName`: camera name written into the YAML, e.g. `center_cam`
- `fitHorizontalFovDeg` / `fitVerticalFovDeg`: useful FOV used for fitting the OpenCV fisheye model
- `exampleImagePath`: optional raw image used for visual checks
- `undistortedFocalScale`: virtual focal scale for the preview undistorted image
- `subtractOneForOpenCV`: keep this `true` for ROS/OpenCV 0-based pixel coordinates

Outputs:

- `ros_equdist_fisheye.yaml`: ROS camera calibration YAML
- `ros_equdist_undistorted_preview.png`: optional undistorted preview image

The YAML uses:

```yaml
distortion_model: equidistant
camera_matrix: K
distortion_coefficients: [k1, k2, k3, k4]
```

Check the printed fit error before using the YAML for lidar-camera calibration.
A good conversion should have small reprojection error relative to MATLAB's
`fisheyeIntrinsics`.


## Lidar Camera Calibration
### Manually Select the Matched Points

Use the multi-timestamp manual pairing tool:

```bash
rosparam set /use_sim_time false

python3 scripts/fisheye_lidar_cam_pair_tool.py \
  --bag /path/to/Bags/merged_node88.bag \
  --camera-yaml /path/to/paired_results/node88/center/ros_equdist_fisheye.yaml
```

The input bag should contain one camera stream, one camera info stream, one
lidar stream, and optionally `/tf`, for example:

```text
/camera/center/image_raw/compressed
/camera/center/camera_info
/rslidar_points_front
/tf
```

By default the tool auto-detects these topics. If needed, override them:

```bash
python3 scripts/fisheye_lidar_cam_pair_tool.py \
  --bag /path/to/merged_node88.bag \
  --camera-yaml /path/to/ros_equdist_fisheye.yaml \
  --lidar-topic /rslidar_points_front \
  --image-topic /camera/center/image_raw/compressed \
  --camera-info-topic /camera/center/camera_info
```

The tool publishes only calibration topics:

- `/calibration/rslidar_points_front`
- `/calibration/camera/image_raw`
- `/calibration/camera/image_rect`
- `/calibration/camera/camera_info`
- `/calibration/pairs_markers`

Recommended RViz setup:

- Fixed Frame: the lidar frame, e.g. `rslidar_front`
- PointCloud2: `/calibration/rslidar_points_front`
- Image: `/calibration/camera/image_raw` or `/calibration/camera/image_rect`
- MarkerArray: `/calibration/pairs_markers`
- Use RViz `Publish Point` to publish selected 3D points to `/clicked_point`

The tool uses lidar/cloud timestamps as the sync reference. It matches each
pointcloud to the nearest image and camera info, then downsamples the valid
timestamps for easier browsing.

Useful sync options:

- `--sync-tolerance 0.05`: maximum allowed nearest-neighbor time difference, in seconds
- `--frame-step 10`: default timestamp downsampling; for a 10 Hz bag this shows about 1 Hz
- `--target-sync-hz 1.0`: alternative downsampling by target display rate
- `--max-sync-frames 100`: cap the displayed synchronized frames

Pair selection workflow:

1. Start `roscore`.
2. Run the pairing tool.
3. Open RViz and subscribe to the `/calibration/*` topics.
4. Select a synchronized frame in the UI.
5. Pick a 3D point in RViz with `Publish Point`.
6. Click the matching pixel in the tool's `Raw image` tab.
7. Repeat across multiple timestamps. Current-frame markers are strong; other-frame markers are ghosted.
8. Save pairs with `Save pairs`.

The raw fisheye image is the authoritative pixel selection view. The rectified
image is only a preview/checking aid.

If a YAML file is passed with `--camera-yaml`, the tool overrides the bag
`CameraInfo`, resizes the raw image to the YAML resolution, and rectifies using
the YAML fisheye model. If no YAML is passed, the bag `CameraInfo` is used.

The `TF guess` tab shows the current lidar-to-camera transform in ROS static
transform convention:

```text
x y z yaw pitch roll lidar_frame camera_frame
```

Use this tab to manually tune the transform before optimization:

- `Apply TF guess`: broadcasts the edited TF and republishes the current frame
- `Reset from bag TF`: reloads the initial transform from `/tf`
- `Evaluate current TF`: computes reprojection error using the current TF without optimizing
- `Run calibration`: uses the current TF as the initial guess

After calibration, the tool:

- broadcasts the calibrated TF live
- prints `static_transform_publisher` arguments
- saves a result YAML, normally `fisheye_lidar_camera_calibration_result.yaml`
- writes per-pair reprojection errors and overall RMS/p95/max error

Notes:

- Prefer `rosparam set /use_sim_time false` for this tool. It reads the bag
  internally and republishes selected frames live; using sim time without
  publishing `/clock` can make RViz ignore fresh messages.
- If RViz does not visually update after editing the TF, press `Apply TF guess`.
  The tool republishes the current pointcloud/image/camera_info with fresh
  timestamps so RViz uses the new transform immediately.
