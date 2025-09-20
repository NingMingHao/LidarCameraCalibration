function interactive_rigid3d_demo(varname)
% Interactive visualization of a rigid3d transform with XYZ + Yaw/Pitch/Roll.
% Rotation convention: ZYX (yaw, pitch, roll), degrees.
% Saves the current transform as a rigid3d to the base workspace.
%
% Usage:
%   interactive_rigid3d_demo            % saves to 'T_rigid'
%   interactive_rigid3d_demo('T_body')  % saves to 'T_body'

    if nargin < 1 || ~ischar(varname)
        varname = 'INIT_TRANSFORM';
    end

    % ------- Figure & Axes -------
    fig = figure('Name','rigid3d: interactive frame', 'NumberTitle','off', ...
                 'Position',[100 100 980 620], 'Color','w');
    ax = axes('Parent',fig, 'Position',[0.08 0.15 0.62 0.80]); %#ok<LAXES>
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z'); view(ax, 135, 25);
    axis(ax, [-2 2 -2 2 -2 2]);

    % ------- World Frame (fixed) -------
    s = 0.7;  % arrow length
    drawFrame(ax, eye(3), [0 0 0], s, 2.5, true); % world frame (thicker)

    % ------- UI Controls -------
    % Slider specs: [min max init]
    specs = struct( ...
        'x',     [-1.5 1.5 0], ...
        'y',     [-1.5 1.5 0], ...
        'z',     [-1.5 1.5 0], ...
        'yaw',   [-180 180 0], ...
        'pitch', [-90  90  0], ...
        'roll',  [-180 180 0] );

    names = {'x','y','z','yaw','pitch','roll'};
    labels= {'x (m)','y (m)','z (m)','yaw (deg)','pitch (deg)','roll (deg)'};

    controls = struct();
    y0 = 0.87; dy = 0.11;

    for i = 1:numel(names)
        nm = names{i};
        yPos = y0 - (i-1)*dy;
        % label
        uicontrol(fig,'Style','text','String',labels{i},'Units','normalized', ...
            'Position',[0.73 yPos 0.10 0.05],'HorizontalAlignment','left', ...
            'FontWeight','bold','BackgroundColor','w');
        % slider
        controls.(nm) = uicontrol(fig,'Style','slider','Units','normalized', ...
            'Min',specs.(nm)(1), 'Max',specs.(nm)(2), 'Value',specs.(nm)(3), ...
            'Position',[0.83 yPos 0.13 0.05], 'Callback',@(~,~) updatePlot());
        % value box
        controls.([nm '_val']) = uicontrol(fig,'Style','edit','Units','normalized', ...
            'String',num2str(specs.(nm)(3)), 'Position',[0.83 yPos-0.045 0.13 0.045], ...
            'BackgroundColor',[1 1 1], 'Callback',@(h,~) editToSlider(nm,h));
    end

    % Variable name label + edit
    uicontrol(fig,'Style','text','String','Var name','Units','normalized', ...
        'Position',[0.73 0.12 0.10 0.05],'HorizontalAlignment','left', ...
        'FontWeight','bold','BackgroundColor','w');
    controls.varname = uicontrol(fig,'Style','edit','Units','normalized', ...
        'String',varname, 'Position',[0.83 0.12 0.13 0.05], ...
        'BackgroundColor',[1 1 1], 'Callback',@(~,~) updatePlot());

    % Auto-save checkbox
    controls.autosave = uicontrol(fig,'Style','checkbox','Units','normalized', ...
        'String','Auto-save to base', 'Value',1, ...
        'Position',[0.73 0.07 0.23 0.05], 'BackgroundColor','w', ...
        'Callback',@(~,~) updatePlot());

    % Save snapshot button
    uicontrol(fig,'Style','pushbutton','String','Save Snapshot','Units','normalized', ...
        'Position',[0.73 0.02 0.10 0.06], 'Callback',@(~,~) saveSnapshot());

    % Info text
    uicontrol(fig,'Style','text','Units','normalized','BackgroundColor','w', ...
        'Position',[0.84 0.02 0.12 0.06], 'HorizontalAlignment','left', ...
        'String', sprintf('Order: Z-Y-X\n(yaw, pitch, roll)'));

    % ------- Transformed Frame (will be updated) -------
    tfHandles = struct();  % will store graphics handles
    updatePlot();          % initial draw

    % ---------- Nested helpers ----------
    function resetAll() %#ok<DEFNU>
        for j = 1:numel(names)
            nm = names{j};
            set(controls.(nm), 'Value', specs.(nm)(3));
            set(controls.([nm '_val']), 'String', num2str(specs.(nm)(3)));
        end
        updatePlot();
    end

    function editToSlider(nm, hEdit)
        v = str2double(get(hEdit,'String'));
        if isnan(v), v = get(controls.(nm),'Value'); end
        v = max(min(v, get(controls.(nm),'Max')), get(controls.(nm),'Min'));
        set(controls.(nm),'Value', v);
        set(hEdit,'String', num2str(v));
        updatePlot();
    end

    function updatePlot()
        % read values
        x     = get(controls.x,    'Value'); set(controls.x_val,    'String',num2str(x));
        y     = get(controls.y,    'Value'); set(controls.y_val,    'String',num2str(y));
        z     = get(controls.z,    'Value'); set(controls.z_val,    'String',num2str(z));
        yaw   = get(controls.yaw,  'Value'); set(controls.yaw_val,  'String',num2str(yaw));
        pitch = get(controls.pitch,'Value'); set(controls.pitch_val,'String',num2str(pitch));
        roll  = get(controls.roll, 'Value'); set(controls.roll_val, 'String',num2str(roll));

        % rotation (ZYX, degrees)
        R = eulZYX_deg_to_rotm([yaw pitch roll]); % 3x3
        t = [x y z];

        % draw/update the transformed frame
        tfHandles = drawFrame(ax, R, t, s, 1.8, false, tfHandles);
        title(ax, sprintf('t=[%.2f, %.2f, %.2f], yaw=%.1f°, pitch=%.1f°, roll=%.1f°', ...
                          x,y,z, yaw,pitch,roll));

        % create a rigid3d and (optionally) save to base workspace
        T = rigid3d(R, t);
        if get(controls.autosave,'Value') == 1
            vname = strtrim(get(controls.varname,'String'));
            if isempty(vname), vname = 'T_rigid'; set(controls.varname,'String',vname); end
            try
                assignin('base', vname, T);
            catch ME
                warning('Could not assign variable to base workspace: %s', ME.message);
            end
        end

        drawnow;
    end

    function saveSnapshot()
        % Always save a single snapshot to the base workspace (ignores autosave state)
        % Recompute T to avoid race with slider callbacks.
        x     = get(controls.x,    'Value');
        y     = get(controls.y,    'Value');
        z     = get(controls.z,    'Value');
        yaw   = get(controls.yaw,  'Value');
        pitch = get(controls.pitch,'Value');
        roll  = get(controls.roll, 'Value');

        R = eulZYX_deg_to_rotm([yaw pitch roll]);
        t = [x y z];
        T = rigid3d(R, t);

        vname = strtrim(get(controls.varname,'String'));
        if isempty(vname), vname = 'T_rigid'; set(controls.varname,'String',vname); end
        assignin('base', vname, T);
        fprintf('Saved snapshot to base workspace as %s (rigid3d)\n', vname);
    end
end

% ---------------- Utility: draw a frame as 3 colored arrows ----------------
function handles = drawFrame(ax, R, t, scale, headSize, isWorld, handles)
% R (3x3), t (1x3). Draw arrows for X(red), Y(green), Z(blue) starting at t.
% If 'handles' provided, update them; else create new.
    if nargin < 7, handles = struct(); end
    if nargin < 6, isWorld = false; end

    O = t(:).';                 % origin of the frame
    X = O + (R(:,1)*scale).';   % endpoint of x-axis
    Y = O + (R(:,2)*scale).';   % endpoint of y-axis
    Z = O + (R(:,3)*scale).';   % endpoint of z-axis

    % Components for quiver3: start (x0,y0,z0) and delta (u,v,w)
    vecs = [X - O; Y - O; Z - O];   % 3x3
    cols = [1 0 0; 0 0.6 0; 0 0 1]; % X=red, Y=green, Z=blue

    % Create or update the three arrows
    names = {'x','y','z'};
    for i = 1:3
        if ~isfield(handles, names{i}) || ~isvalidHandle(handles.(names{i}))
            handles.(names{i}) = quiver3(ax, O(1),O(2),O(3), vecs(i,1),vecs(i,2),vecs(i,3), ...
                                         0, 'LineWidth', isWorld*2 + (~isWorld)*2, ...
                                         'MaxHeadSize', headSize, 'Color', cols(i,:));
        else
            set(handles.(names{i}), 'XData', O(1), 'YData', O(2), 'ZData', O(3), ...
                                    'UData', vecs(i,1), 'VData', vecs(i,2), 'WData', vecs(i,3), ...
                                    'Color', cols(i,:), 'MaxHeadSize', headSize, ...
                                    'LineWidth', isWorld*2 + (~isWorld)*2);
        end
    end

    % Label at the frame origin
    if ~isfield(handles,'lbl') || ~isvalidHandle(handles.lbl)
        tag = ternary(isWorld, 'World', 'Frame''');
        handles.lbl = text(ax, O(1),O(2),O(3), ['  ' tag], 'FontWeight','bold', 'Color',[0 0 0]);
    else
        set(handles.lbl, 'Position', [O(1),O(2),O(3)]);
    end
end

% --------------- Utility: ZYX Euler (deg) -> Rotation Matrix ---------------
function R = eulZYX_deg_to_rotm(eulZYX_deg)
% eulZYX_deg = [yaw pitch roll] (degrees), applied Rz(yaw)*Ry(pitch)*Rx(roll)
    yaw   = deg2rad(eulZYX_deg(1));
    pitch = deg2rad(eulZYX_deg(2));
    roll  = deg2rad(eulZYX_deg(3));

    cz = cos(yaw);   sz = sin(yaw);
    cy = cos(pitch); sy = sin(pitch);
    cx = cos(roll);  sx = sin(roll);

    Rz = [cz -sz 0; sz cz 0; 0 0 1];
    Ry = [cy 0 sy; 0 1 0; -sy 0 cy];
    Rx = [1 0 0; 0 cx -sx; 0 sx cx];

    R = Rz * Ry * Rx;
end

% --------------- Small helpers ---------------
function tf = isvalidHandle(h)
    tf = ~isempty(h) && isgraphics(h) && isvalid(h);
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end