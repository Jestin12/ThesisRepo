%% Full hand reconstruction from glove CSV
% MATLAB version of the uploaded Jupyter notebook.
% Loads one CSV row, extracts quaternions, computes relative rotations,
% builds simple forward kinematics, and plots the reconstructed hand.
%
% The uploaded notebook reads underscore-style quaternion columns such as
% left_palm_prox_quat_w and left_index_prox_quat_w, then forms relative
% palm->proximal and proximal->mid rotations for each finger. [file:118]

clear; clc; close all;

%% User settings
csvPath   = '/home/jestin/ThesisRepo/ML/TwoHand_L_Fist_R_Fist_filtered_butterworth_lp/glove_data_L_Fist_R_Fist_5s_3_2026-04-19_19-31-18_bw_lp_5.0hz.csv';

handSide  = 'left';    % 'left' or 'right'
rowSelect = 1;         % MATLAB uses 1-based indexing
applyAxisFix = true;

if ~isfile(csvPath)
    error('CSV file not found: %s', csvPath);
end

T = readtable(csvPath);
fprintf('Loaded: %s\n', csvPath);
fprintf('Shape: %d rows x %d columns\n', height(T), width(T));

if rowSelect < 1 || rowSelect > height(T)
    error('rowSelect must be between 1 and %d', height(T));
end

%% Fixed 90 deg clockwise rotation about +Z
Rz_cw_90 = [0  1  0;
           -1  0  0;
            0  0  1];

if applyAxisFix
    R_align = Rz_cw_90;
else
    R_align = eye(3);
end

%% Load quaternions from the selected row
Hand = loadHandQuaternions(T, rowSelect, handSide);
disp(Hand.sources)

%% Convert to rotation matrices and compute relative rotations
Rot = struct();
Rot.Palm = quatToRotm(Hand.Palm) * R_align;

fingerNames = {'Thumb','Index','Middle','Ring','Pinky'};
for i = 1:numel(fingerNames)
    finger = fingerNames{i};
    if ~isfield(Hand, finger)
        continue;
    end

    R_prox = quatToRotm(Hand.(finger).Proximal) * R_align;
    R_mid  = quatToRotm(Hand.(finger).Mid) * R_align;

    Rot.(finger).Global.Proximal = R_prox;
    Rot.(finger).Global.Mid      = R_mid;
    Rot.(finger).Relative.Palm2Proximal = relativeRotation(Rot.Palm, R_prox);
    Rot.(finger).Relative.Proximal2Mid  = relativeRotation(R_prox, R_mid);
end

%% Simple hand geometry (edit with your measured dimensions)
Links = struct();
Links.Palm = [0; 0; 0];

Links.Thumb.Metacarpal = [-25;   0;  0];
Links.Thumb.Proximal   = [  0; -40;  0];
Links.Thumb.Mid        = [  0; -30;  0];
Links.Thumb.Distal     = [  0; -20;  0];

Links.Index.Metacarpal = [-25; -80;  0];
Links.Index.Proximal   = [  0; -40;  0];
Links.Index.Mid        = [  0; -30;  0];
Links.Index.Distal     = [  0; -20;  0];

Links.Middle.Metacarpal = [  0; -80;  0];
Links.Middle.Proximal   = [  0; -45;  0];
Links.Middle.Mid        = [  0; -35;  0];
Links.Middle.Distal     = [  0; -22;  0];

Links.Ring.Metacarpal = [ 20; -78;  0];
Links.Ring.Proximal   = [  0; -42;  0];
Links.Ring.Mid        = [  0; -30;  0];
Links.Ring.Distal     = [  0; -20;  0];

Links.Pinky.Metacarpal = [ 40; -72;  0];
Links.Pinky.Proximal   = [  0; -35;  0];
Links.Pinky.Mid        = [  0; -24;  0];
Links.Pinky.Distal     = [  0; -18;  0];

%% Build FK transforms
Tf = buildHandTransforms(Rot, Links);

%% Plot
figure('Color','w','Position',[100 100 1100 850]);
ax = axes; hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title(sprintf('%s hand reconstruction, row %d', capitalize(handSide), rowSelect));

plotFrame(ax, eye(4), 12, 'Palm');
colors = lines(numel(fingerNames));

for i = 1:numel(fingerNames)
    finger = fingerNames{i};
    if ~isfield(Tf, finger)
        continue;
    end

    p0 = extractPoint(eye(4));
    p1 = extractPoint(Tf.(finger).Knuckle);
    p2 = extractPoint(Tf.(finger).Proximal);
    p3 = extractPoint(Tf.(finger).Mid);
    p4 = extractPoint(Tf.(finger).Tip);

    pts = [p0 p1 p2 p3 p4];
    plot3(pts(1,:), pts(2,:), pts(3,:), '-o', 'LineWidth', 2, 'Color', colors(i,:));

    plotFrame(ax, Tf.(finger).Proximal, 8, [finger '-P']);
    plotFrame(ax, Tf.(finger).Mid, 8, [finger '-M']);
end

legend(fingerNames, 'Location', 'bestoutside');
rotate3d on;

%% ---------- Local functions ----------

function q = normalizeQuat(q)
q = double(q(:));
if numel(q) ~= 4
    error('Quaternion must have 4 elements.');
end
n = norm(q);
if n == 0
    error('Zero quaternion cannot be normalized.');
end
q = q / n;
end

function R = quatToRotm(q)
% Expects scalar-first [w; x; y; z]
q = normalizeQuat(q);
w = q(1); x = q(2); y = q(3); z = q(4);
R = [1 - 2*(y^2 + z^2),   2*(x*y - z*w),     2*(x*z + y*w);
     2*(x*y + z*w),       1 - 2*(x^2 + z^2), 2*(y*z - x*w);
     2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x^2 + y^2)];
end

function Tm = Ttranslate(t)
Tm = eye(4);
Tm(1:3,4) = double(t(:));
end

function Tm = Trotate(R)
Tm = eye(4);
Tm(1:3,1:3) = double(R);
end

function Rrel = relativeRotation(Rparent, Rchild)
Rrel = Rparent' * Rchild;
end

function Hand = loadHandQuaternions(T, rowIdx, handSide)
side = lower(string(handSide));
Hand = struct();
Sources = struct();

[palmQ, palmSrc] = extractQuaternion(T, rowIdx, {side + "_wrist", side + "_palm_prox", side + "_palm_mid"}, true);
Hand.Palm = palmQ;
Sources.Palm = palmSrc;

fingers = {'thumb','index','middle','ring','pinky'};
for i = 1:numel(fingers)
    finger = fingers{i};
    fingerCap = capitalize(finger);

    [proxQ, proxSrc] = extractQuaternion(T, rowIdx, {side + "_" + finger + "_prox"}, false);
    [midQ,  midSrc ] = extractQuaternion(T, rowIdx, {side + "_" + finger + "_mid"}, false);

    if isempty(proxQ) && isempty(midQ)
        continue;
    end
    if isempty(proxQ)
        proxQ = midQ;
        proxSrc = midSrc + " (fallback for prox)";
    end
    if isempty(midQ)
        midQ = proxQ;
        midSrc = proxSrc + " (fallback for mid)";
    end

    Hand.(fingerCap).Proximal = proxQ;
    Hand.(fingerCap).Mid = midQ;
    Sources.(fingerCap).Proximal = proxSrc;
    Sources.(fingerCap).Mid = midSrc;
end

Hand.sources = Sources;
end

function [q, usedPrefix] = extractQuaternion(T, rowIdx, prefixes, required)
q = [];
usedPrefix = "";
vars = string(T.Properties.VariableNames);
for i = 1:numel(prefixes)
    prefix = string(prefixes{i});
    cols = [prefix + "_quat_w", prefix + "_quat_x", prefix + "_quat_y", prefix + "_quat_z"];
    if all(ismember(cols, vars))
        q = [T{rowIdx, cols(1)}; T{rowIdx, cols(2)}; T{rowIdx, cols(3)}; T{rowIdx, cols(4)}];
        usedPrefix = prefix;
        return;
    end
end
if required
    error('No quaternion columns found for prefixes: %s', strjoin(string(prefixes), ', '));
end
end

function Tf = buildHandTransforms(Rot, Links)
Tf = struct();
Tf.Palm = eye(4);
fingers = {'Thumb','Index','Middle','Ring','Pinky'};
for i = 1:numel(fingers)
    finger = fingers{i};
    if ~isfield(Rot, finger)
        continue;
    end

    T_palm_to_knuckle = Ttranslate(Links.(finger).Metacarpal);
    T_knuckle_to_prox = Trotate(Rot.(finger).Relative.Palm2Proximal);
    T_prox_link = Ttranslate(Links.(finger).Proximal);
    T_prox_to_mid = Trotate(Rot.(finger).Relative.Proximal2Mid);
    T_mid_link = Ttranslate(Links.(finger).Mid);
    T_dist_link = Ttranslate(Links.(finger).Distal);

    T_prox = Tf.Palm * T_palm_to_knuckle * T_knuckle_to_prox;
    T_mid  = T_prox * T_prox_link * T_prox_to_mid;
    T_tip  = T_mid * T_mid_link * T_dist_link;

    Tf.(finger).Knuckle  = Tf.Palm * T_palm_to_knuckle;
    Tf.(finger).Proximal = T_prox;
    Tf.(finger).Mid      = T_mid;
    Tf.(finger).Tip      = T_tip;
end
end

function plotFrame(ax, Tm, axisLen, labelText)
origin = Tm(1:3,4);
R = Tm(1:3,1:3);
ex = R * [axisLen;0;0];
ey = R * [0;axisLen;0];
ez = R * [0;0;axisLen];
quiver3(ax, origin(1), origin(2), origin(3), ex(1), ex(2), ex(3), 0, 'r', 'LineWidth', 1.2);
quiver3(ax, origin(1), origin(2), origin(3), ey(1), ey(2), ey(3), 0, 'g', 'LineWidth', 1.2);
quiver3(ax, origin(1), origin(2), origin(3), ez(1), ez(2), ez(3), 0, 'b', 'LineWidth', 1.2);
text(origin(1), origin(2), origin(3), [' ' char(labelText)], 'Parent', ax, 'FontSize', 9);
end

function p = extractPoint(Tm)
p = Tm(1:3,4);
end

function s = capitalize(txt)
txt = char(txt);
s = string([upper(txt(1)) txt(2:end)]);
end
