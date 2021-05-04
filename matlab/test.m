clear, clc
addpath(genpath("/home/brucebot/workspace/matlab_utils"));
addpath("/home/brucebot/workspace/lc-calibration/L1_relaxation")
loadLibraries(2);


%% Datasets
mocap_name = "CCW1";
mocap_files = load("CCW1.mat");

distance(1,:) = [318,1834];
distance(2,:) = [2792,5156];
distance(3,:) = [5779,6880];
distance(4,:) = [7422,8925];
distance(5,:) = [9625,11170];
distance(6,:) = [11850,13270];
estimation_indices = 3;


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
R_L1L2 =  rotx(180) * roty(0) * rotz(-92);
R_T1T2 =  rotx(200) * roty(-7)* rotz(-85);
O_L2_in_L1 = [0.12 0.0 0.12]';
O_T2_in_T1 = [0, 0, 0]';
H_L1L2 = constructHByRotationTranslation(R_L1L2, -R_L1L2 * O_L2_in_L1);
H_T1T2 = constructHByRotationTranslation(R_T1T2, -R_T1T2 * O_T2_in_T1);

%% Mocap data
% LiDAR1
L1_trans_mean = lidar_pos(1, :, distance(estimation_indices,1)) ./ 1000;
L1_rot_mean = lidar_rot(1, :, distance(estimation_indices,1));


% Target1
T1_trans_mean = tag_pos(1, :, distance(estimation_indices,1)) ./ 1000;
T1_rot_mean = tag_rot(1, :, distance(estimation_indices,1));


% Convert rotm from 1x9 to 3x3
L1_rotm = reshape(L1_rot_mean, 3, 3);
T1_rotm = reshape(T1_rot_mean, 3, 3);

% Construct H
H_W1T1 = constructHByRotationTranslation(T1_rotm', -T1_rotm' * T1_trans_mean');
H_W1L1 = constructHByRotationTranslation(L1_rotm', -L1_rotm' * L1_trans_mean');

   

%%%%%%%%%%%%%%
M_H_L2T2 = (H_T1T2 * H_W1T1) / (H_L1L2 * H_W1L1);
M_R_L2T2 = M_H_L2T2(1:3, 1:3);
M_O_T2_in_L2 = -M_R_L2T2 \ M_H_L2T2(1:3, 4);



%% Estimated data
lidart = dlmread("tag_size1.220000pose.txt", ',', 1, 0);
L_trans_L2T2_raw = mean(lidart(:, 4:6), 1);
L_rotm_L2T2_raw = reshape(mean(lidart(1, 7:end), 1), [], 3)';
L_rpy_L2T2_raw = rad2deg(rotm2eul(L_rotm_L2T2_raw(1:3,1:3), 'XYZ'));

L_H_L2T2_raw = constructHByRotationTranslation(L_rotm_L2T2_raw, L_trans_L2T2_raw);
L_H_L2T2 = inv(L_H_L2T2_raw);

%%%%%%%%%%%%%%
L_R_L2T2 = L_H_L2T2(1:3, 1:3);
L_O_T2_in_L2 = -L_R_L2T2 \ L_H_L2T2(1:3, 4);


%% Restuls
M_cur_rpy = rad2deg(rotm2eul(M_H_L2T2(1:3,1:3), 'XYZ'));
M_cur_trans = M_O_T2_in_L2; % -M_H_L2T2(1:3,1:3)' * M_H_L2T2(1:3,4); 
L_cur_rpy = rad2deg(rotm2eul(L_H_L2T2(1:3,1:3), 'XYZ'));
L_cur_trans = L_O_T2_in_L2; %L_H_L2T2(1:3, 4);


disp("Translation error:")
M_cur_trans - L_cur_trans

disp("RPY error:")
M_cur_rpy - L_cur_rpy + [90-45, 0 180]


