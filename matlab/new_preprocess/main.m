clear, clc
addpath(genpath("/home/brucebot/workspace/matlab_utils"));
addpath("/home/brucebot/workspace/lc-calibration/L1_relaxation")
loadLibraries(2);


% Files
mocap_path = "./";
datasets = 1;



%% Datasets
if datasets == 1
    mocap_name = "CCW1";
    mocap_files = load(mocap_path + mocap_name + ".mat");
    
    distance(1,:) = [318,1834];
    distance(2,:) = [2792,5156];
    distance(3,:) = [5779,6880];
    distance(4,:) = [7422,8925];
    distance(5,:) = [9625,11170];
    distance(6,:) = [11850,13270];
    estimation_indices = 1:6;
    
elseif datasets == 2
    mocap_name = "straight_front1";
    mocap_files = load(mocap_path + mocap_name + ".mat");

    
    distance(1,:) = [44, 2188];
    distance(2,:) = [2900, 3252];
    distance(3,:) = [4053, 4681];
    distance(4,:) = [5026, 5498];
    distance(5,:) = [6935, 8542];
    distance(6,:) = [8977, 9519];
    estimation_indices = 2:5;
end

lt_estimate_path = mocap_path + ...
                    "predictions/mocap_prediction/" + ...
                    lower(mocap_name) + "-";
% apriltag estimatation path
at_estimate_path = mocap_path + ...
                   "predictions/apriltag_prediction/" + ...
                   lower(mocap_name) + "-";

%% Frame assignment
for k = 1:mocap_files.(mocap_name).RigidBodies.Bodies
    if strcmp(mocap_files.(mocap_name).RigidBodies.Name(k),'lidar')
        lidar_pos_raw = ...
            mocap_files.(mocap_name).RigidBodies.Positions(k,:,:);
        lidar_rot_raw = ...
            mocap_files.(mocap_name).RigidBodies.Rotations(k,:,:);
    else
        tag_pos_raw = ...
            mocap_files.(mocap_name).RigidBodies.Positions(k,:,:);
        tag_rot_raw = ...
            mocap_files.(mocap_name).RigidBodies.Rotations(k,:,:);
    end
end


%% Remove invalid data points
n = 1;
invalid_num = 0;
total_frames = mocap_files.(mocap_name).Frames;
for m = 1: total_frames
    lidar_invalid = any(isnan(lidar_pos_raw(:,:,m))) || ...
                    any(isnan(lidar_rot_raw(:,:,m)));
    tag_invalid = any(isnan(tag_pos_raw(:,:,m))) || ...
                  any(isnan(tag_rot_raw(:,:,m)));
              
    if (lidar_invalid || tag_invalid)
        lidar_pos_raw(:,:,m) = zeros(1,3);
        lidar_rot_raw(:,:,m) = zeros(1,9);
        tag_pos_raw(:,:,m) = zeros(1,3);
        tag_rot_raw(:,:,m) = zeros(1,9);
        invalid_num = invalid_num + 1;
    else
        lidar_pos(:,:,n) = lidar_pos_raw(:,:,m);
        lidar_rot(:,:,n) = lidar_rot_raw(:,:,m);
        tag_pos(:,:,n) = tag_pos_raw(:,:,m);
        tag_rot(:,:,n) = tag_rot_raw(:,:,m);
        n = n + 1;
    end
end

valid_frames = n - 1;
lidart = 1:valid_frames;
y = squeeze(lidar_pos(1, 1,:));
figure(1);
scatter(lidart,y,'filled');
p = 1:valid_frames;
q = squeeze(lidar_pos(1, 2,:));
figure(2);
scatter(p,q,'filled');
p = 1:valid_frames;
q = squeeze(lidar_pos(1, 3,:));
figure(3);
scatter(p,q,'filled');



%% Frame conversion
H_L1L2 = eye(4);
H_T1T2 = eye(4);
H_L1L2(1:3, 1:3) =  rotz(-90) * roty(180);
H_T1T2(1:3, 1:3) =  rotz(-90) * roty(180);


%% Mocap data
num_mocap_data = size(distance, 1);
M_rpy_L2T2 = zeros(num_mocap_data, 3);
M_trans_L2T2 = zeros(num_mocap_data, 3);


for i = 1:num_mocap_data
    % LiDAR 1
    L1_trans_mean = mean(lidar_pos(1, :, distance(i,1):distance(i,2)), 3);
    L1_rot_mean = mean(lidar_rot(1, :, distance(i,1):distance(i,2)), 3);
    
    % Target 1
    T1_trans_mean = mean(tag_pos(1, :, distance(i,1):distance(i,2)), 3);
    T1_rot_mean = mean(tag_rot(1, :, distance(i,1):distance(i,2)), 3);
    
    % Convert rotm from 1x9 to 3x3
    L1_rotm = reshape(L1_rot_mean, 3, 3);
    T1_rotm = reshape(T1_rot_mean, 3, 3);
    
    % Construct H
    H_W1T1 = createSE3(T1_rotm, T1_trans_mean');
    H_W1L1 = createSE3(L1_rotm, L1_trans_mean');
    
    
%     H_L2T2 = (H_WL1 * H_L1L2) \ (H_WT1 * H_T1T2);
    H_L2T2 = (H_T1T2 * H_W1T1) / (H_L1L2 * H_W1L1);
    
    M_rpy_L2T2(i, :) = rad2deg(rotm2eul(H_L2T2(1:3,1:3), 'XYZ'));
    M_trans_L2T2(i, :) = H_L2T2(1:3,4) ./ 1000;
end

%% Estimated data
L_trans_L2T2 = zeros(length(estimation_indices), 3);
L_rpy_L2T2 = zeros(length(estimation_indices), 3);
A_trans_C2T2 = zeros(length(estimation_indices), 3);
A_rpy_C2T2 = zeros(length(estimation_indices), 3);
for i = estimation_indices
    lt_current_path = lt_estimate_path + num2str(i) + "/";
    lt_estimate_file = dir(lt_current_path + "*.txt");
    lidart = dlmread(lt_current_path + lt_estimate_file.name, ',', 1, 0);
    L_trans_L2T2(i,:) = mean(lidart(:, 4:6), 1);
    L_rpy_L2T2(i,:) = rad2deg(mean(lidart(:, 7:9),1));
    
    at_current_path = at_estimate_path + num2str(i) + "/";
    at_estimate_file = dir(at_current_path + "*april.txt");
    apriltag = dlmread(at_current_path + at_estimate_file.name, ',', 1, 0);
    A_trans_C2T2(i,:) = mean(apriltag(:,3:5),1);
    A_rpy_C2T2(i,:) = rad2deg(mean(apriltag(:,6:8),1));
end
M_rpy_L2T2
M_trans_L2T2
L_rpy_L2T2
L_trans_L2T2


%% Results
Distance_meter = M_trans_L2T2(estimation_indices,  1);
d_x_meter = L_trans_L2T2(estimation_indices,  1) - ...
            M_trans_L2T2(estimation_indices, 1);
d_y_meter = L_trans_L2T2(estimation_indices,  2) - ...
            M_trans_L2T2(estimation_indices, 2);
d_z_meter = L_trans_L2T2(estimation_indices,  3) - ...
            M_trans_L2T2(estimation_indices, 3);
d_r_degree = L_rpy_L2T2(estimation_indices,  1) - ...
             M_trans_L2T2(estimation_indices, 1);
d_p_degree = L_rpy_L2T2(estimation_indices,  2) - ...
             M_trans_L2T2(estimation_indices, 2);
d_h_degree = L_rpy_L2T2(estimation_indices,  3) - ...
             M_trans_L2T2(estimation_indices, 3);


%% Tables
T = table(Distance_meter, ...
          d_x_meter, d_y_meter, d_z_meter, ...
          d_r_degree,d_p_degree,d_h_degree)


%% Plottings
figure(1);
plot(M_trans_L2T2(estimation_indices, 1), d_x_meter);

figure(2);
plot(M_trans_L2T2(estimation_indices, 1), d_y_meter);

figure(3);
plot(M_trans_L2T2(estimation_indices, 1), d_z_meter);

figure(4);
plot(M_trans_L2T2(estimation_indices, 1), d_r_degree);

figure(5);
plot(M_trans_L2T2(estimation_indices, 1), d_p_degree);

figure(6);
plot(M_trans_L2T2(estimation_indices, 1), d_h_degree);


%%
num_data = 3;

% LiDAR 1
L1_trans_mean = mean(lidar_pos(1,:,distance(num_data,1):distance(num_data,2)),3)./1000;
L1_rot_mean = mean(lidar_rot(1,:,distance(num_data,1):distance(num_data,2)),3);

% Target 1
T1_trans_mean = mean(tag_pos(1,:,distance(num_data,1):distance(num_data,2)),3)./1000;
T1_rot_mean = mean(tag_rot(1,:,distance(num_data,1):distance(num_data,2)),3);

% Convert rotm from 1x9 to 3x3
L1_rotm = reshape(L1_rot_mean, 3, 3);
T1_rotm = reshape(T1_rot_mean, 3, 3);

% Construct H
H_W1T1 = createSE3(T1_rotm, T1_trans_mean);
H_W1L1 = createSE3(L1_rotm, L1_trans_mean);

% change basis
H_L1T1 = H_W1L1 \ H_W1T1;
H_W3W1 = eye(4);
H_W3W1(1:3, 1:3) = roty(180) * rotz(-90);

H_W1L2 = (H_W1L1 * H_L1L2);
H_W1T2 = (H_W1T1 * H_T1T2);
H_L2T2 = H_W1L2 \ H_W1T2;



M_rpy_L2T2(i, :) = rad2deg(rotm2eul(H_L2T2(1:3,1:3), 'XYZ'));
M_trans_L2T2(i, :) = H_L2T2(1:3,4) ./ 1000;

    
%% Testing
[axes_h, fig_h] = createFigHandleWithNumber(5, 100,"frames", 1, 1);

% Frame
W1 = eye(4);  % Frame of LiDAR Device 


W2 = eye(4);
W2(1:3, 1:3) = rotz(-90);
W2(1:3, 4) = 1;

W3 = eye(4);
W3(1:3, 1:3) = rotx(180) * rotz(-90);
W3(1:3, 4) = 2;


cur_fig = axes_h(1);
plotColoredOriginAxisWithText(cur_fig, "W1", W1, 0.5)
plotColoredOriginAxisWithText(cur_fig, "W2", W2, 0.5)
plotColoredOriginAxisWithText(cur_fig, "W3", W3, 0.5)

showCurrentPlot(cur_fig, "Frame system", [-40 30], 1)



%%

% Frame
W3 = eye(4);  % Frame of LiDAR Device 
W1 = eye(4);
W1(1:3, 1:3) = rotx(180) * rotz(-90);
% W1(1:3, 1:3) = roty(180);
W1(1:3, 4) = 1;


cur_fig = axes_h(1);
plotColoredOriginAxisWithText(cur_fig, "W3", W3, 0.5)
plotColoredOriginAxisWithText(cur_fig, "W1", W1, 0.5)
showCurrentPlot(cur_fig, "Frame system", [-40 30], 1)


cur_fig = axes_h(2);
plotColoredOriginAxisWithText(cur_fig, "Origin", eye(4), 0.5)
plotColoredOriginAxisWithText(cur_fig, "Mocap", W1, 0.5)
plotColoredOriginAxisWithText(cur_fig, "LiDAR", H_W1L1 * W1, 0.5)
plotColoredOriginAxisWithText(cur_fig, "Tag", H_W1T1 * W1, 0.5)
showCurrentPlot(cur_fig, "Mcap Frames", [-40 30], 1)



cur_fig = axes_h(3);
plotColoredOriginAxisWithText(cur_fig, "Origin", W1, 0.5)
plotColoredOriginAxisWithText(cur_fig, "LiDAR", H_W1L2 * W1, 0.5)
plotColoredOriginAxisWithText(cur_fig, "Tag", H_W1T2 * W1, 0.5)
showCurrentPlot(cur_fig, "LiDAR Frames", [-40 30], 1)



% reloadCurrentPlot(cur_fig)
% set(cur_fig, 'ZDir','reverse')
% set(cur_fig, 'ZDir','normal')

disp("Done")