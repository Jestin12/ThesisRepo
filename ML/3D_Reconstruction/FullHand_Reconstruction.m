%% ============================================================
%  Full Hand Reconstruction — MATLAB port of FullHand_Reconstruction.ipynb
%  Quaternion convention: XYZW (vector part first, scalar part last)
%  Sensor frame: -Y along finger (distal), +Z out of dorsum, +X across hand
% ============================================================

clear; clc; close all;

%% Cell 0 — Imports (no-op in MATLAB; tables/plot tools are built in)
% Python: pandas, numpy, matplotlib
% MATLAB equivalents: readtable, built-in arrays, plot3/quiver3

%% Cell 1 — quat_to_rotmat is defined as a local function at bottom

%% Cell 2 — T_translate / T_rotate are defined as local functions at bottom

%% Cell 3 — Fixed 90° clockwise rotation about Z (sensor-to-bone alignment)
Rz_cw_90 = [ 0.0,  1.0, 0.0; ...
            -1.0,  0.0, 0.0; ...
             0.0,  0.0, 1.0 ];

%% Cell 4 — Load CSV
csv_path = '/home/jestin/ThesisRepo/ML/3D_Reconstruction/glove_data_flat2fist_2s_1_2026-04-26_16-41-36_filtered_butterworth_lp.csv';
df = readtable(csv_path);
fprintf('Loaded: %s\n', csv_path);
fprintf('Shape: %d rows x %d columns\n', height(df), width(df));
disp(head(df));

%% Cell 5 — Build LeftHandQuaternions struct (XYZW per joint)
ROW_SELECT = 2;   % MATLAB is 1-indexed (Python's 1 -> MATLAB's 2)

LeftHandQuaternions = struct();
LeftHandQuaternions.Wrist = readQuat(df, 'left_wrist',       ROW_SELECT);
LeftHandQuaternions.Palm  = readQuat(df, 'left_palm_prox',   ROW_SELECT);

LeftHandQuaternions.Thumb.Proximal  = readQuat(df, 'left_thumb_prox',  ROW_SELECT);
LeftHandQuaternions.Thumb.Mid       = readQuat(df, 'left_thumb_mid',   ROW_SELECT);

LeftHandQuaternions.Index.Proximal  = readQuat(df, 'left_index_prox',  ROW_SELECT);
LeftHandQuaternions.Index.Mid       = readQuat(df, 'left_index_mid',   ROW_SELECT);

LeftHandQuaternions.Middle.Proximal = readQuat(df, 'left_middle_prox', ROW_SELECT);
LeftHandQuaternions.Middle.Mid      = readQuat(df, 'left_middle_mid',  ROW_SELECT);

LeftHandQuaternions.Ring.Proximal   = readQuat(df, 'left_ring_prox',   ROW_SELECT);
LeftHandQuaternions.Ring.Mid        = readQuat(df, 'left_ring_mid',    ROW_SELECT);

LeftHandQuaternions.Pinky.Proximal  = readQuat(df, 'left_pinky_prox',  ROW_SELECT);
LeftHandQuaternions.Pinky.Mid       = readQuat(df, 'left_pinky_mid',   ROW_SELECT);

%% Cell 6 — Pretty-print all quaternions
printQuaternions(LeftHandQuaternions, '');

%% Cell 7 — Quick unpack of wrist quaternion
q = LeftHandQuaternions.Wrist;
fprintf('Wrist Quaternion: x=%.3f, y=%.3f, z=%.3f, w=%.3f\n', q(1), q(2), q(3), q(4));

%% Cell 8 — Quaternion primitives are defined as local functions at bottom
%  q_normalise, q_conjugate, q_multiply, q_rotate_vector, q_relative,
%  twist_about_x, transform_vector_swing_only, transform_vector_twist_only

%% Cell 9 — Rotation matrices (note: uses LeftHand instead of LeftHandQuaternions
%  in the original Python — kept the same here for fidelity. Swap names if needed.)
LeftHand = LeftHandQuaternions;   % alias so the next block compiles unchanged

LeftHand_RotMats = struct();
LeftHand_RotMats.Palm = quat_to_rotmat(LeftHand.Palm);

fingers = {'Thumb','Index','Middle','Ring','Pinky'};
for k = 1:numel(fingers)
    f = fingers{k};
    R_palm = quat_to_rotmat(LeftHand.Palm);
    R_prox = quat_to_rotmat(LeftHand.(f).Proximal);
    R_mid  = quat_to_rotmat(LeftHand.(f).Mid);

    LeftHand_RotMats.(f).Palm2Proximal  = R_prox * inv(R_palm) * Rz_cw_90;
    LeftHand_RotMats.(f).Proximal2Mid   = R_mid  * inv(R_prox) * Rz_cw_90;
end

%% Cell 10 — Link lengths (segment vectors in their local frame)
LeftHand_Links = struct();
LeftHand_Links.Palm = [0, 0, 0];

LeftHand_Links.Thumb.Metacarpal = [-25,   0, 0.10];
LeftHand_Links.Thumb.Proximal   = [  0, -40, 0.00];
LeftHand_Links.Thumb.Mid        = [  0, -30, 0.00];

LeftHand_Links.Index.Metacarpal = [-25, -80, 0.12];
LeftHand_Links.Index.Proximal   = [  0, -40, 0.00];
LeftHand_Links.Index.Mid        = [  0, -30, 0.00];

LeftHand_Links.Middle.Metacarpal = [  0, -80, 0.00];
LeftHand_Links.Middle.Proximal   = [  0, -40, 0.00];
LeftHand_Links.Middle.Mid        = [  0, -30, 0.00];

LeftHand_Links.Ring.Metacarpal  = [-20, -80, 0.00];
LeftHand_Links.Ring.Proximal    = [  0, -40, 0.00];
LeftHand_Links.Ring.Mid         = [  0, -30, 0.00];

LeftHand_Links.Pinky.Metacarpal = [-40, -80, 0.00];
LeftHand_Links.Pinky.Proximal   = [  0, -40, 0.00];
LeftHand_Links.Pinky.Mid        = [  0, -30, 0.00];

%% Cell 11 — Transform each link into the palm frame
LeftHand_Vectors = struct();

for k = 1:numel(fingers)
    f = fingers{k};

    % Metacarpal: rotate by palm quaternion only
    LeftHand_Vectors.(f).Metacarpal = q_rotate_vector( ...
        LeftHandQuaternions.Palm, LeftHand_Links.(f).Metacarpal);

    % Proximal: twist-only transform between palm and proximal sensors
    LeftHand_Vectors.(f).Proximal = transform_vector_twist_only( ...
        LeftHand_Links.(f).Proximal, ...
        LeftHandQuaternions.(f).Proximal, ...
        LeftHandQuaternions.Palm);

    % Mid: twist-only transform between proximal and mid sensors
    LeftHand_Vectors.(f).Mid = transform_vector_twist_only( ...
        LeftHand_Links.(f).Mid, ...
        LeftHandQuaternions.(f).Mid, ...
        LeftHandQuaternions.(f).Proximal);
end

%% Cell 12 — Chain vectors into joint positions and plot
palm_origin = LeftHand_Links.Palm(:).';   % row vector

LeftHand_JointPositions = struct();
segments = {'Metacarpal', 'Proximal', 'Mid'};

for k = 1:numel(fingers)
    f = fingers{k};
    current_pos = palm_origin;
    for s = 1:numel(segments)
        seg = segments{s};
        current_pos = current_pos + LeftHand_Vectors.(f).(seg);
        LeftHand_JointPositions.(f).(seg) = current_pos;
    end
end

% Plot
fig = figure('Position', [100, 100, 1200, 900]);
ax  = axes(fig);
hold(ax, 'on');
grid(ax, 'on');

% Palm origin
scatter3(ax, palm_origin(1), palm_origin(2), palm_origin(3), ...
         60, 'k', 'filled');
text(ax, palm_origin(1), palm_origin(2), palm_origin(3), '  Palm', ...
     'Color', 'k', 'FontSize', 10);

finger_colors = struct( ...
    'Thumb',  [1.00, 0.50, 0.05], ...   % tab:orange
    'Index',  [0.12, 0.47, 0.71], ...   % tab:blue
    'Middle', [0.17, 0.63, 0.17], ...   % tab:green
    'Ring',   [0.84, 0.15, 0.16], ...   % tab:red
    'Pinky',  [0.58, 0.40, 0.74] );     % tab:purple

for k = 1:numel(fingers)
    f = fingers{k};
    c = finger_colors.(f);

    pts = [ palm_origin; ...
            LeftHand_JointPositions.(f).Metacarpal; ...
            LeftHand_JointPositions.(f).Proximal; ...
            LeftHand_JointPositions.(f).Mid ];

    plot3(ax, pts(:,1), pts(:,2), pts(:,3), '-o', ...
          'Color', c, 'MarkerFaceColor', c, ...
          'LineWidth', 1.5, 'DisplayName', f);

    text(ax, pts(end,1), pts(end,2), pts(end,3), ...
         sprintf('  %s tip', f), 'Color', c, 'FontSize', 8);

    % Draw segment vectors as arrows
    start_pos = palm_origin;
    for s = 1:numel(segments)
        seg = segments{s};
        vec = LeftHand_Vectors.(f).(seg);
        quiver3(ax, start_pos(1), start_pos(2), start_pos(3), ...
                vec(1), vec(2), vec(3), 0, ...
                'Color', c, 'LineWidth', 1, 'MaxHeadSize', 0.3, ...
                'HandleVisibility', 'off');
        start_pos = start_pos + vec;
    end
end

xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
title(ax, 'Left Hand Finger Joint Chains');
legend(ax, 'show', 'Location', 'best');
axis(ax, 'equal');
view(ax, 3);
hold(ax, 'off');


% ============================================================
%                       LOCAL FUNCTIONS
% ============================================================

function R = quat_to_rotmat(q)
    % q in XYZW order
    q = q(:).' / norm(q);
    x = q(1); y = q(2); z = q(3); w = q(4);
    R = [ 1 - 2*(y^2 + z^2),  2*(x*y - z*w),       2*(x*z + y*w);
          2*(x*y + z*w),      1 - 2*(x^2 + z^2),   2*(y*z - x*w);
          2*(x*z - y*w),      2*(y*z + x*w),       1 - 2*(x^2 + y^2) ];
end

function T = T_translate(t)
    T = eye(4);
    T(1:3, 4) = t(:);
end

function T = T_rotate(R)
    T = eye(4);
    T(1:3, 1:3) = R;
end

% ---- Quaternion primitives (XYZW) ----

function q = q_normalise(q)
    q = q / norm(q);
end

function q = q_conjugate(q)
    % Flip vector part, keep scalar
    q = [-q(1), -q(2), -q(3), q(4)];
end

function q = q_multiply(q1, q2)
    % Hamilton product q1 ⊗ q2 (XYZW)
    x1 = q1(1); y1 = q1(2); z1 = q1(3); w1 = q1(4);
    x2 = q2(1); y2 = q2(2); z2 = q2(3); w2 = q2(4);
    q = [ w1*x2 + x1*w2 + y1*z2 - z1*y2, ...   % x
          w1*y2 - x1*z2 + y1*w2 + z1*x2, ...   % y
          w1*z2 + x1*y2 - y1*x2 + z1*w2, ...   % z
          w1*w2 - x1*x2 - y1*y2 - z1*z2 ];     % w
end

function v_out = q_rotate_vector(q, v)
    % Sandwich product: rotate 3-vector v by quaternion q
    v_quat = [v(1), v(2), v(3), 0.0];
    q_conj = q_conjugate(q);
    rotated = q_multiply(q_multiply(q, v_quat), q_conj);
    v_out = rotated(1:3);
end

function q_rel = q_relative(q_from, q_to)
    % Rotation from q_from to q_to: q_from^-1 ⊗ q_to
    q_rel = q_multiply(q_conjugate(q_from), q_to);
end

% ---- Swing-twist decomposition (XYZW) ----

function q_t = twist_about_x(q)
    % Keep only the X-axis twist component, renormalise
    x = q(1); w = q(4);
    denom = sqrt(w*w + x*x);
    if denom < 1e-9
        q_t = [0, 0, 0, 1];   % identity in XYZW
    else
        q_t = [x/denom, 0, 0, w/denom];
    end
end

function v_out = transform_vector_swing_only(v_forearm, q_forearm, q_palm)
    q_f_twist = twist_about_x(q_forearm);
    q_p_twist = twist_about_x(q_palm);
    q_f_swing = q_multiply(q_forearm, q_conjugate(q_f_twist));
    q_p_swing = q_multiply(q_palm,    q_conjugate(q_p_twist));
    q_rel_swing = q_relative(q_p_swing, q_f_swing);
    v_out = q_rotate_vector(q_rel_swing, v_forearm);
end

function v_out = transform_vector_twist_only(v_forearm, q_forearm, q_palm)
    q_f_twist = twist_about_x(q_forearm);
    q_p_twist = twist_about_x(q_palm);
    q_rel_twist = q_relative(q_p_twist, q_f_twist);
    v_out = q_rotate_vector(q_rel_twist, v_forearm);
end

% ---- CSV / printing helpers ----

function q = readQuat(df, prefix, row)
    % Read a single row of XYZW columns named <prefix>_quat_x/y/z/w
    q = [ df.([prefix '_quat_x'])(row), ...
          df.([prefix '_quat_y'])(row), ...
          df.([prefix '_quat_z'])(row), ...
          df.([prefix '_quat_w'])(row) ];
end

function printQuaternions(s, prefix)
    fn = fieldnames(s);
    for i = 1:numel(fn)
        key = fn{i};
        val = s.(key);
        if isstruct(val)
            fprintf('%s%s:\n', prefix, key);
            printQuaternions(val, [prefix '  ']);
        else
            fprintf('%s%s = [%.3f, %.3f, %.3f, %.3f]\n', ...
                    prefix, key, val(1), val(2), val(3), val(4));
        end
    end
end
