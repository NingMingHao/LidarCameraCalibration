%% convert_matlab_fisheye_to_ros_equdist.m
%
% Convert MATLAB fisheyeIntrinsics / Scaramuzza calibration into an
% approximate standard ROS/OpenCV fisheye CameraInfo YAML.
%
% Output:
%   ros_equdist_fisheye.yaml
%
% ROS/OpenCV output model:
%   distortion_model: equidistant
%   camera_matrix: K
%   distortion_coefficients: [k1, k2, k3, k4]
%
% This script fits the OpenCV fisheye model to MATLAB's own fisheyeIntrinsics
% projection. After fixing the image resize / x-y scale issue, this should
% give very small errors, e.g. RMS around 0.03 px.

clear; clc; close all;

%% ---------------- USER SETTINGS ----------------

calibMatPath = 'calibrationSession.mat';

save_path = pwd;
yamlFilePath = fullfile(save_path, 'ros_equdist_fisheye.yaml');

cameraName = 'center_cam';

% Your useful camera FOV.
fitHorizontalFovDeg = 150;
fitVerticalFovDeg   = 100;

% More samples give smoother validation but take slightly longer.
numYawSamples   = 181;
numPitchSamples = 101;

% Residual weighting.
% 'image_uniform' avoids over-weighting dense regions of the synthetic grid.
% 'none' is also fine if your fit is already excellent.
weightMode = 'image_uniform';
weightBinsU = 32;
weightBinsV = 18;

% Optional raw image for visual overlay.
% Leave as '' if you do not want background image.
exampleImagePath = fullfile(save_path, 'Images/0036.png');

% Show an undistorted preview generated from the fitted OpenCV fisheye params.
showUndistortedPreview = true;

% Virtual pinhole focal scale for the undistorted preview.
% Smaller values show more FOV with more black border; larger values zoom in.
undistortedFocalScale = 0.8;

% Optional preview output path. Leave as '' if you only want the figure.
undistortedPreviewPath = fullfile(save_path, 'ros_equdist_undistorted_preview.png');

% MATLAB pixel coordinate origin is effectively 1-based.
% OpenCV/ROS pixel coordinate origin is 0-based.
% Keep this true for ROS YAML export.
subtractOneForOpenCV = true;

% Optimization settings.
maxIterations = 500;
maxFunctionEvaluations = 80000;

%% ---------------- LOAD MATLAB CALIBRATION ----------------

S = load(calibMatPath);

if isfield(S, 'calibrationSession')
    cameraParams = S.calibrationSession.CameraParameters;
elseif isfield(S, 'cameraParams')
    cameraParams = S.cameraParams;
else
    error('Cannot find calibrationSession or cameraParams in %s', calibMatPath);
end

intr = cameraParams.Intrinsics;

assert(isa(intr, 'fisheyeIntrinsics'), ...
    'Expected cameraParams.Intrinsics to be fisheyeIntrinsics.');

imageSize = intr.ImageSize;  % [height, width]
height = imageSize(1);
width  = imageSize(2);

mc = intr.MappingCoefficients;
dc = intr.DistortionCenter;
SM = intr.StretchMatrix;

fprintf('\nLoaded MATLAB fisheyeIntrinsics:\n');
disp(intr);

fprintf('Image size: height=%d, width=%d\n', height, width);
fprintf('MappingCoefficients: [%.12g %.12g %.12g %.12g]\n', mc);
fprintf('DistortionCenter:    [%.12g %.12g]\n', dc);
fprintf('StretchMatrix:\n');
disp(SM);

if norm(SM - eye(2), 'fro') > 1e-3
    warning(['StretchMatrix is not close to identity. ', ...
             'Standard ROS/OpenCV equidistant may still fit, but check errors carefully.']);
end

%% ---------------- SAMPLE 3D RAYS ----------------
% Camera coordinate convention:
%   x right, y down, z forward.
%
% We generate synthetic camera-frame rays and use MATLAB's fisheyeIntrinsics
% as the reference projection model.

yawVec = linspace(-deg2rad(fitHorizontalFovDeg)/2, ...
                   deg2rad(fitHorizontalFovDeg)/2, numYawSamples);

pitchVec = linspace(-deg2rad(fitVerticalFovDeg)/2, ...
                     deg2rad(fitVerticalFovDeg)/2, numPitchSamples);

[YAW, PITCH] = meshgrid(yawVec, pitchVec);

X = sin(YAW) .* cos(PITCH);
Y = sin(PITCH);
Z = cos(YAW) .* cos(PITCH);

rays = [X(:), Y(:), Z(:)];
rays = rays ./ vecnorm(rays, 2, 2);

% Keep only forward-facing rays.
rays = rays(rays(:,3) > 1e-8, :);

%% ---------------- PROJECT RAYS USING MATLAB MODEL ----------------

pixMatlab = projectMatlabFisheyeModel(intr, rays);
validIndex = all(isfinite(pixMatlab), 2);

inside = validIndex(:) & ...
         pixMatlab(:,1) >= 1 & pixMatlab(:,1) <= width & ...
         pixMatlab(:,2) >= 1 & pixMatlab(:,2) <= height;

raysFit = rays(inside, :);
pixMatlabFit = pixMatlab(inside, :);

fprintf('\nUsing %d valid rays for fitting.\n', size(raysFit, 1));

if size(raysFit, 1) < 100
    error('Too few valid rays. Increase FOV sampling or check calibration.');
end

%% ---------------- BUILD RESIDUAL WEIGHTS ----------------

weights = buildResidualWeights(pixMatlabFit, width, height, ...
                               weightMode, weightBinsU, weightBinsV);

fprintf('Weight mode: %s\n', weightMode);
fprintf('Weight range: min %.4f, max %.4f, mean %.4f\n', ...
    min(weights), max(weights), mean(weights));

%% ---------------- INITIAL GUESS ----------------
% Since your corrected calibration has StretchMatrix close to identity,
% use MappingCoefficients(1) as the initial focal length.

a0 = abs(mc(1));

fx0 = a0;
fy0 = a0;
cx0 = dc(1);
cy0 = dc(2);

k10 = 0;
k20 = 0;
k30 = 0;
k40 = 0;

% Parameter layout:
% p = [fx, fy, cx, cy, k1, k2, k3, k4]
p0 = [fx0, fy0, cx0, cy0, k10, k20, k30, k40];

lb = [ ...
    0.05*fx0, 0.05*fy0, ...
    -0.25*width, -0.25*height, ...
    -50, -50, -50, -50];

ub = [ ...
    5.0*fx0, 5.0*fy0, ...
    1.25*width, 1.25*height, ...
    50, 50, 50, 50];

fprintf('\nInitial OpenCV fisheye guess:\n');
printOpenCVParams(p0);

%% ---------------- FIT STANDARD OPENCV FISHEYE MODEL ----------------

hasLsqnonlin = exist('lsqnonlin', 'file') == 2;

if hasLsqnonlin
    opts = optimoptions('lsqnonlin', ...
        'Display', 'iter', ...
        'MaxIterations', maxIterations, ...
        'MaxFunctionEvaluations', maxFunctionEvaluations, ...
        'FunctionTolerance', 1e-12, ...
        'StepTolerance', 1e-12, ...
        'OptimalityTolerance', 1e-12);
else
    warning('lsqnonlin not found. Falling back to fminsearch without bounds.');
end

fprintf('\nFitting standard OpenCV/ROS fisheye model...\n');

% Stage 1: robust residual for stable initialization.
residualRobust = @(p) projectionResidualsOpenCVFisheye( ...
    p, raysFit, pixMatlabFit, weights, true);

% Stage 2: true pixel residual for final RMS minimization.
residualRaw = @(p) projectionResidualsOpenCVFisheye( ...
    p, raysFit, pixMatlabFit, weights, false);

if hasLsqnonlin
    fprintf('\nStage 1: robust initialization...\n');
    pStage1 = lsqnonlin(residualRobust, p0, lb, ub, opts);

    fprintf('\nStage 2: raw pixel-error refinement...\n');
    pFit = lsqnonlin(residualRaw, pStage1, lb, ub, opts);
else
    costRobust = @(p) sum(residualRobust(p).^2);
    pStage1 = fminsearch(costRobust, p0);

    costRaw = @(p) sum(residualRaw(p).^2);
    pFit = fminsearch(costRaw, pStage1);
end

%% ---------------- EVALUATE FIT ----------------

pixOpenCVFit = projectOpenCVFisheyeMatlab(raysFit, pFit);
err = pixOpenCVFit - pixMatlabFit;
errNorm = vecnorm(err, 2, 2);

stats.mean   = mean(errNorm);
stats.median = median(errNorm);
stats.p90    = prctile(errNorm, 90);
stats.p95    = prctile(errNorm, 95);
stats.max    = max(errNorm);
stats.rms    = sqrt(mean(errNorm.^2));

fprintf('\nFinal fitted OpenCV/ROS fisheye parameters in MATLAB pixel coordinates:\n');
printOpenCVParams(pFit);

fprintf('\nApproximation error relative to MATLAB fisheyeIntrinsics:\n');
fprintf('  mean   = %.6f px\n', stats.mean);
fprintf('  median = %.6f px\n', stats.median);
fprintf('  p90    = %.6f px\n', stats.p90);
fprintf('  p95    = %.6f px\n', stats.p95);
fprintf('  max    = %.6f px\n', stats.max);
fprintf('  RMS    = %.6f px\n', stats.rms);

if stats.rms < 1.0 && stats.p95 < 2.0
    fprintf('\nResult: GOOD. Standard ROS equidistant model is accurate enough.\n');
else
    warning('Fit error is larger than expected. Check image size, resize/crop, and FOV settings.');
end

%% ---------------- EXPORT ROS YAML ----------------

[K_ros, D_ros, P_ros] = openCVParamsToRosKDP(pFit, subtractOneForOpenCV);

writeRosEquidistantYaml(yamlFilePath, cameraName, width, height, ...
                        K_ros, D_ros, P_ros);

fprintf('\nSaved ROS equidistant YAML:\n  %s\n', yamlFilePath);

fprintf('\nROS/OpenCV exported parameters:\n');
fprintf('K = \n');
disp(K_ros);
fprintf('D = [%.12g %.12g %.12g %.12g]\n', D_ros);

%% ---------------- OPTIONAL MATLAB cameraIntrinsicsKB CHECK ----------------
% Useful for MATLAB-side validation of the exported OpenCV/ROS parameters.

try
    intrKB = cameraIntrinsicsFromOpenCV(K_ros, D_ros, imageSize);
    fprintf('\nCreated MATLAB cameraIntrinsicsKB from exported OpenCV parameters:\n');
    disp(intrKB);
catch ME
    warning('Could not create cameraIntrinsicsKB: %s', ME.message);
end

%% ---------------- VISUALIZATION ----------------

I_bg = [];
if ~isempty(exampleImagePath) && exist(exampleImagePath, 'file')
    I_bg = imread(exampleImagePath);
end

plotOverlayComparison(pixMatlabFit, pixOpenCVFit, err, errNorm, ...
                      I_bg, width, height, stats);

if showUndistortedPreview
    if isempty(I_bg)
        warning('Cannot show undistorted preview because exampleImagePath is empty or missing.');
    else
        [I_undist, K_undist] = undistortImageWithOpenCVFisheye( ...
            I_bg, pFit, imageSize, undistortedFocalScale);

        plotUndistortedPreview(I_bg, I_undist, K_undist, undistortedFocalScale);

        if ~isempty(undistortedPreviewPath)
            imwrite(I_undist, undistortedPreviewPath);
            fprintf('\nSaved undistorted preview image:\n  %s\n', undistortedPreviewPath);
        end
    end
end

plotErrorHeatmap(pixMatlabFit, errNorm, width, height, stats);
plotErrorHistogram(errNorm, stats);

fprintf('\nDone.\n');

%% ============================================================
%% Local functions
%% ============================================================

function weights = buildResidualWeights(pix, width, height, mode, binsU, binsV)
    n = size(pix, 1);

    switch lower(mode)
        case 'none'
            weights = ones(n, 1);

        case 'image_uniform'
            u = pix(:,1);
            v = pix(:,2);

            iu = floor((u - 1) ./ max(width - 1, 1) .* binsU) + 1;
            iv = floor((v - 1) ./ max(height - 1, 1) .* binsV) + 1;

            iu = max(1, min(binsU, iu));
            iv = max(1, min(binsV, iv));

            linearIdx = sub2ind([binsV, binsU], iv, iu);
            counts = accumarray(linearIdx, 1, [binsU*binsV, 1], @sum, 0);

            weights = 1 ./ max(counts(linearIdx), 1);
            weights = weights ./ mean(weights);

        otherwise
            error('Unknown weight mode: %s', mode);
    end
end

function residual = projectionResidualsOpenCVFisheye(p, rays, targetPix, weights, robust)
    predPix = projectOpenCVFisheyeMatlab(rays, p);
    e = predPix - targetPix;

    rx = e(:,1);
    ry = e(:,2);

    if robust
        rx = charbonnierResidual(rx, 3.0);
        ry = charbonnierResidual(ry, 3.0);
    end

    sw = sqrt(weights(:));
    residual = [sw .* rx; sw .* ry];
end

function r = charbonnierResidual(x, delta)
    % Smooth robust residual, approximately L2 near zero and L1 for large x.
    r = sign(x) .* sqrt(2 * delta^2 .* (sqrt(1 + (x ./ delta).^2) - 1));
end

function pix = projectMatlabFisheyeModel(intr, rays)
    % Use MATLAB's own fisheyeIntrinsics projection as the reference.
    try
        tformIdentity = rigidtform3d(eye(3), [0 0 0]);

        try
            [pix, ~] = world2img(rays, tformIdentity, intr);
        catch
            pix = world2img(rays, tformIdentity, intr);
        end

    catch MEWorld2Img
        warning('world2img failed, trying worldToImage fallback: %s', MEWorld2Img.message);
        pix = projectWithWorldToImage(intr, rays);
    end
end

function pix = projectWithWorldToImage(intr, rays)
    R = eye(3);
    t = [0 0 0];

    try
        [pix, ~] = worldToImage(intr, R, t, rays);
    catch
        pix = worldToImage(intr, R, t, rays);
    end
end

function pix = projectOpenCVFisheyeMatlab(pointsCam, p)
    % OpenCV fisheye projection implemented in MATLAB pixel coordinates.
    %
    % p:
    %   [fx, fy, cx, cy, k1, k2, k3, k4]
    %
    % OpenCV fisheye model:
    %   x = X/Z
    %   y = Y/Z
    %   r = sqrt(x^2 + y^2)
    %   theta = atan(r)
    %   theta_d = theta * (1 + k1*theta^2 + k2*theta^4
    %                        + k3*theta^6 + k4*theta^8)
    %   xd = theta_d/r * x
    %   yd = theta_d/r * y
    %   u = fx * xd + cx
    %   v = fy * yd + cy

    fx = p(1);
    fy = p(2);
    cx = p(3);
    cy = p(4);

    k1 = p(5);
    k2 = p(6);
    k3 = p(7);
    k4 = p(8);

    X = pointsCam(:,1);
    Y = pointsCam(:,2);
    Z = pointsCam(:,3);

    x = X ./ Z;
    y = Y ./ Z;

    r = sqrt(x.^2 + y.^2);
    theta = atan(r);

    theta2 = theta.^2;
    theta4 = theta2.^2;
    theta6 = theta4 .* theta2;
    theta8 = theta4.^2;

    theta_d = theta .* (1 + ...
        k1 .* theta2 + ...
        k2 .* theta4 + ...
        k3 .* theta6 + ...
        k4 .* theta8);

    scale = ones(size(r));
    nz = r > 1e-12;
    scale(nz) = theta_d(nz) ./ r(nz);

    xd = scale .* x;
    yd = scale .* y;

    u = fx .* xd + cx;
    v = fy .* yd + cy;

    pix = [u, v];
end

function [K_ros, D_ros, P_ros] = openCVParamsToRosKDP(p, subtractOneForOpenCV)
    fx = p(1);
    fy = p(2);
    cx = p(3);
    cy = p(4);

    if subtractOneForOpenCV
        cx = cx - 1;
        cy = cy - 1;
    end

    D_ros = p(5:8);

    K_ros = [fx, 0.0, cx;
             0.0, fy, cy;
             0.0, 0.0, 1.0];

    P_ros = [fx, 0.0, cx, 0.0;
             0.0, fy, cy, 0.0;
             0.0, 0.0, 1.0, 0.0];
end

function writeRosEquidistantYaml(filename, cameraName, width, height, K, D, P)
    fid = fopen(filename, 'w');
    assert(fid > 0, 'Cannot open YAML file for writing: %s', filename);

    fprintf(fid, 'image_width: %d\n', width);
    fprintf(fid, 'image_height: %d\n', height);
    fprintf(fid, 'camera_name: %s\n', cameraName);
    fprintf(fid, 'distortion_model: equidistant\n');

    fprintf(fid, 'camera_matrix:\n');
    fprintf(fid, '  rows: 3\n');
    fprintf(fid, '  cols: 3\n');
    fprintf(fid, '  data: [%.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g]\n', ...
        K(1,1), K(1,2), K(1,3), ...
        K(2,1), K(2,2), K(2,3), ...
        K(3,1), K(3,2), K(3,3));

    fprintf(fid, 'distortion_coefficients:\n');
    fprintf(fid, '  rows: 1\n');
    fprintf(fid, '  cols: 4\n');
    fprintf(fid, '  data: [%.12g, %.12g, %.12g, %.12g]\n', ...
        D(1), D(2), D(3), D(4));

    fprintf(fid, 'rectification_matrix:\n');
    fprintf(fid, '  rows: 3\n');
    fprintf(fid, '  cols: 3\n');
    fprintf(fid, '  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n');

    fprintf(fid, 'projection_matrix:\n');
    fprintf(fid, '  rows: 3\n');
    fprintf(fid, '  cols: 4\n');
    fprintf(fid, '  data: [%.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g]\n', ...
        P(1,1), P(1,2), P(1,3), P(1,4), ...
        P(2,1), P(2,2), P(2,3), P(2,4), ...
        P(3,1), P(3,2), P(3,3), P(3,4));

    fclose(fid);
end

function printOpenCVParams(p)
    fprintf('  fx = %.12g\n', p(1));
    fprintf('  fy = %.12g\n', p(2));
    fprintf('  cx = %.12g   [MATLAB coordinate convention]\n', p(3));
    fprintf('  cy = %.12g   [MATLAB coordinate convention]\n', p(4));
    fprintf('  k1 = %.12g\n', p(5));
    fprintf('  k2 = %.12g\n', p(6));
    fprintf('  k3 = %.12g\n', p(7));
    fprintf('  k4 = %.12g\n', p(8));
end

function [I_undist, K_undist] = undistortImageWithOpenCVFisheye(I, p, imageSize, focalScale)
    % Inverse-map a pinhole output image through the fitted OpenCV fisheye model.
    height = imageSize(1);
    width = imageSize(2);

    fx = p(1) * focalScale;
    fy = p(2) * focalScale;
    cx = p(3);
    cy = p(4);

    K_undist = [fx, 0.0, cx;
                0.0, fy, cy;
                0.0, 0.0, 1.0];

    [U, V] = meshgrid(1:width, 1:height);

    x = (U(:) - cx) ./ fx;
    y = (V(:) - cy) ./ fy;

    pinholeRays = [x, y, ones(size(x))];
    sourcePix = projectOpenCVFisheyeMatlab(pinholeRays, p);

    sourceU = reshape(sourcePix(:,1), height, width);
    sourceV = reshape(sourcePix(:,2), height, width);

    I_work = double(I);
    I_undist_work = zeros(height, width, size(I_work, 3));

    for c = 1:size(I_work, 3)
        I_undist_work(:,:,c) = interp2(I_work(:,:,c), sourceU, sourceV, ...
                                       'linear', 0);
    end

    if isfloat(I)
        I_undist = cast(I_undist_work, class(I));
    elseif islogical(I)
        I_undist = I_undist_work > 0.5;
    else
        minValue = double(intmin(class(I)));
        maxValue = double(intmax(class(I)));
        I_undist_work = min(max(I_undist_work, minValue), maxValue);
        I_undist = cast(round(I_undist_work), class(I));
    end
end

function plotUndistortedPreview(I_raw, I_undist, K_undist, focalScale)
    figure('Name', 'Undistorted image from fitted ROS/OpenCV equidistant params');

    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    imshow(I_raw);
    title('Original fisheye image');

    nexttile;
    imshow(I_undist);
    title(sprintf('Undistorted preview, focal scale %.3g', focalScale));

    fprintf('\nUndistorted preview pinhole camera matrix:\n');
    disp(K_undist);
end

function plotOverlayComparison(pixMatlab, pixOpenCV, err, errNorm, I_bg, width, height, stats)
    figure('Name', 'MATLAB fisheye vs fitted ROS/OpenCV equidistant');

    if ~isempty(I_bg)
        imshow(I_bg); hold on;
    else
        axis ij;
        xlim([1 width]);
        ylim([1 height]);
        grid on; hold on;
        xlabel('u [px]');
        ylabel('v [px]');
    end

    plot(pixMatlab(:,1), pixMatlab(:,2), '.', 'MarkerSize', 5);
    plot(pixOpenCV(:,1), pixOpenCV(:,2), '.', 'MarkerSize', 5);

    step = max(1, round(size(pixMatlab, 1) / 600));
    idx = 1:step:size(pixMatlab, 1);

    quiver(pixMatlab(idx,1), pixMatlab(idx,2), ...
           err(idx,1), err(idx,2), 0, 'LineWidth', 1);

    legend('MATLAB fisheyeIntrinsics', ...
           'Fitted ROS/OpenCV equidistant', ...
           'OpenCV - MATLAB error');

    title(sprintf('Projection comparison: RMS %.4f px, p95 %.4f px', ...
        stats.rms, stats.p95));
end

function plotErrorHeatmap(pixMatlab, errNorm, width, height, stats)
    figure('Name', 'Projection error heatmap');

    scatter(pixMatlab(:,1), pixMatlab(:,2), 12, errNorm, 'filled');
    axis ij;
    axis equal;
    xlim([1 width]);
    ylim([1 height]);
    colorbar;
    xlabel('u [px]');
    ylabel('v [px]');
    title(sprintf('ROS/OpenCV equidistant error: RMS %.4f px, p95 %.4f px', ...
        stats.rms, stats.p95));
end

function plotErrorHistogram(errNorm, stats)
    figure('Name', 'Projection error histogram');

    histogram(errNorm, 80);
    grid on;
    xlabel('Pixel error [px]');
    ylabel('Count');
    title(sprintf('Projection error distribution: RMS %.4f px, p95 %.4f px', ...
        stats.rms, stats.p95));
end
