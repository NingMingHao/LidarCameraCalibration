
%save_path = '/Users/minghao/Documents/UWaterloo/Projects/outdoor/Calibration2508/paired_results/node1/front_left';

% First time: fit plane + draw ROI; keep points 5 cm–2 m above plane
crop_pointclouds_with_ground_roi(save_path, ...
    'RawPcDir','PointClouds', ...
    'OutPcDir','CropPointClouds', ...
    'MinAbove',-2.0, 'MaxAbove',2.0, ...
    'RansacDist',0.08, 'MaxAngDist',15, ...
    'SelectNewROI', true);

function crop_pointclouds_with_ground_roi(save_path, varargin)
% crop_pointclouds_with_ground_roi(save_path, 'SelectNewROI', true, 'MinAbove', 0.05, 'MaxAbove', 2.0)
%
% Inputs
%   save_path: root containing UndistImages/, PointClouds/, CropPointClouds/
%
% Key options (name/value):
%   'RawPcDir'       : folder name with raw PCDs (default 'PointClouds')
%   'OutPcDir'       : output folder name (default 'CropPointClouds')
%   'GridStep'       : voxel downsample for fitting (default 0.05 m)
%   'RansacDist'     : pcfitplane maxDistance (default 0.08 m)
%   'RefNormal'      : approximate ground normal (default [0 0 1])
%   'MaxAngDist'     : max angular dev to RefNormal (deg, default 15)
%   'MinAbove'       : keep points with signed height >= MinAbove (default 0.05 m)
%   'MaxAbove'       : also require height <= MaxAbove (default inf)
%   'SelectNewROI'   : force ROI reselection (default false)
%
% Behavior
%   - Fits a single ground plane from the first available PCD.
%   - Removes ground and near-ground points using signed distance to plane.
%   - Projects remaining points onto the plane (u,v basis).
%   - Lets you draw a polygon ROI once; applies it to all clouds.
%   - Saves plane & ROI in save_path/ground_roi_cache.mat

opts = inputParser;
opts.addParameter('RawPcDir', 'PointClouds', @ischar);
opts.addParameter('OutPcDir', 'CropPointClouds', @ischar);
opts.addParameter('GridStep', 0.01, @isnumeric);
opts.addParameter('RansacDist', 0.2, @isnumeric);
opts.addParameter('RefNormal', [0 0 1], @(x)isnumeric(x)&&numel(x)==3);
opts.addParameter('MaxAngDist', 10, @isnumeric);
opts.addParameter('MinAbove', 0.05, @isnumeric);
opts.addParameter('MaxAbove', inf, @isnumeric);
opts.addParameter('SelectNewROI', false, @islogical);
opts.parse(varargin{:});
P = opts.Results;

imageFilesPath = fullfile(save_path,'UndistImages'); %#ok<NASGU> % not strictly needed but kept for your flow
rawPcFilesPath = fullfile(save_path, P.RawPcDir);
cropPcFilesPath = fullfile(save_path, P.OutPcDir);
if ~exist(cropPcFilesPath,'dir'); mkdir(cropPcFilesPath); end

cacheFile = fullfile(save_path, 'ground_roi_cache.mat');

% ---- Collect PCDs based on UndistImages ----
undist_img_list = dir(fullfile(imageFilesPath, '*.png'));

pcd_list = [];
for k = 1:numel(undist_img_list)
    [~, stem, ~] = fileparts(undist_img_list(k).name);
    pcd_path = fullfile(rawPcFilesPath, [stem '.pcd']);
    if exist(pcd_path, 'file')
        pcd_list(end+1).name = [stem '.pcd']; %#ok<AGROW>
        pcd_list(end).folder = rawPcFilesPath;
    else
        warning('No matching PCD for image %s', undist_img_list(k).name);
    end
end

if isempty(pcd_list)
    error('No PCD files found in %s', rawPcFilesPath);
end

% ---- Load or estimate plane & ROI ----
planeModel = [];
u = []; v = []; n = []; p0 = [];
poly_uv = [];

if ~P.SelectNewROI && exist(cacheFile, 'file')
    S = load(cacheFile, 'planeModel', 'u', 'v', 'n', 'p0', 'poly_uv', 'params');
    if isfield(S, 'planeModel') && isfield(S, 'poly_uv')
        planeModel = S.planeModel; u = S.u; v = S.v; n = S.n; p0 = S.p0; poly_uv = S.poly_uv;
        fprintf('[cache] Loaded plane & ROI from %s\n', cacheFile);
    end
end

% ---- Fit plane (if needed) ----
if isempty(planeModel)
    fprintf('Fitting ground plane using first cloud: %s\n', pcd_list(1).name);
    pc0 = pcread(fullfile(rawPcFilesPath, pcd_list(1).name));

    % Downsample for speed/robustness
    pc0_ds = pcdownsample(pc0, 'gridAverage', P.GridStep);

    % Plane fit with normal constraint
    try
        [planeModel, inlierIdx, ~] = pcfitplane(pc0_ds, P.RansacDist);
    catch ME
        if contains(ME.message,'Could not find enough inliers')
            error(['RANSAC failed: insufficient inliers. Try increasing ''RansacDist'' ', ...
                   'and/or ''MaxAngDist'', or use a denser GridStep.']);
        else
            rethrow(ME);
        end
    end

    % Plane model: planeModel.Parameters = [a b c d] for ax+by+cz+d=0
    % Point on plane p0 and normal n
    params = planeModel.Parameters; % [a b c d]
    n = params(1:3) / norm(params(1:3)); % unit normal
    % pick a point on plane: any solution to ax+by+cz+d=0, e.g. along normal from origin
    p0 = -params(4) * n; % since a*(n_x*k)+b*(n_y*k)+c*(n_z*k)+d=0 -> k = -d

    % ---- Force normal to point toward side with most points > 0.5 m away ----
    distances = (pc0_ds.Location - p0) * n';  % signed distances to plane
    count_pos = sum(distances > 0.5);         % above plane
    count_neg = sum(distances < -0.5);        % below plane

    if count_neg > count_pos
        % Flip normal and parameters
        n = -n;
        params = [-params(1), -params(2), -params(3), -params(4)];
        p0 = -params(4) * n; % recompute p0 after flip
    end


    % Build orthonormal basis (u,v) spanning the plane
    upGuess = [0 0 1];
    if abs(dot(n, upGuess)) > 0.95
        upGuess = [1 0 0]; % avoid degeneracy
    end
    u = cross(upGuess, n); u = u / norm(u);
    v = cross(n, u);       v = v / norm(v);

    % Quick visualization (downsampled)
    figure('Name','Plane fit (downsampled cloud)');
    pcshow(pc0_ds); hold on; title('Estimated ground plane'); grid on
    plot(planeModel);
    legend('Points','Ground plane'); drawnow;
end

% ---- Select/draw ROI (if needed) ----
if isempty(poly_uv)
    fprintf('Projecting points and requesting ROI polygon...\n');
    % Use a subset for display: non-ground band for the same first cloud
    points = pc0.Location;
    % signed height above plane
    h = (points - p0) * n';
    keep = (h >= P.MinAbove) & (h <= P.MaxAbove);
    points_ng = points(keep,:);

    % Project to (u,v)
    rel = points_ng - p0;
    U = rel * u';
    V = rel * v';

    % Plot 2D scatter for ROI selection
    figure('Name','Draw ROI on projected points');
    scatter(U, V, 1); axis equal; grid on
    xlabel('u (along plane)'); ylabel('v (along plane)');
    title('Draw polygon ROI (double-click to finish)');

    roi = drawpolygon('LineWidth',1.5);
    poly_uv = roi.Position; % Nx2: columns u,v
    close(gcf);
end

% ---- Save cache ----
paramsStruct = struct('GridStep', P.GridStep, 'RansacDist', P.RansacDist, ...
    'RefNormal', P.RefNormal, 'MaxAngDist', P.MaxAngDist, ...
    'MinAbove', P.MinAbove, 'MaxAbove', P.MaxAbove);
save(cacheFile, 'planeModel','u','v','n','p0','poly_uv','paramsStruct');

% ---- Process all PCDs with the plane & ROI ----
fprintf('Processing %d clouds...\n', numel(pcd_list));
for i = 1:numel(pcd_list)
    iname = pcd_list(i).name;
    rawfilepath  = fullfile(rawPcFilesPath, iname);
    cropfilepath = fullfile(cropPcFilesPath, iname);
    try
        rawpc = pcread(rawfilepath);
    catch ME
        warning('Skip %s (read error: %s)', iname, ME.message);
        continue
    end

    Pxyz = rawpc.Location;
    % signed height above plane
    h = (Pxyz - p0) * n';
    above = (h >= 1.5) & (h <= 4);

    if ~any(above)
        warning('No points survive height filter in %s', iname);
        pcwrite(pointCloud([]), cropfilepath);
        continue
    end

    Pkeep = Pxyz(above, :);

    % project to (u,v)
    rel = Pkeep - p0;
    U = rel * u';
    V = rel * v';

    % ROI test
    [in, on] = inpolygon(U, V, poly_uv(:,1), poly_uv(:,2));
    roi_mask = (in | on);

    Pfinal = Pkeep(roi_mask, :);

    % write
    croppc = pointCloud(Pfinal);
    try
        pcwrite(croppc, cropfilepath);
    catch ME
        warning('Write failed for %s: %s', cropfilepath, ME.message);
    end

    if mod(i,10)==0 || i==numel(pcd_list)
        fprintf('  %4d/%4d saved (%d pts)\n', i, numel(pcd_list), size(Pfinal,1));
    end
end

fprintf('Done. Results in %s\n', cropPcFilesPath);
end