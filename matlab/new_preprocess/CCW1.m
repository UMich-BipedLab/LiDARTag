clear
file = load('CCW1.mat');
total_frames = file.CCW1.Frames;
for k = 1:file.CCW1.RigidBodies.Bodies
    if strcmp(file.CCW1.RigidBodies.Name(k),'lidar')
        lidar_pos_raw = file.CCW1.RigidBodies.Positions(k,:,:);
        lidar_rot_raw = file.CCW1.RigidBodies.Rotations(k,:,:);
    else
        tag_pos_raw = file.CCW1.RigidBodies.Positions(k,:,:);
        tag_rot_raw = file.CCW1.RigidBodies.Rotations(k,:,:);
    end
end

n = 1;
invalid_num = 0;
for m = 1: total_frames
    lidar_invalid = any(isnan(lidar_pos_raw(:,:,m))) || any(isnan(lidar_rot_raw(:,:,m)));
    tag_invalid = any(isnan(tag_pos_raw(:,:,m))) || any(isnan(tag_rot_raw(:,:,m)));
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
% x = 1:valid_frames;
% y = squeeze(lidar_pos(1,1,:));
% figure(1);
% scatter(x,y,'filled');
% p = 1:valid_frames;
% q = squeeze(lidar_pos(1,2,:));
% figure(2);
% scatter(p,q,'filled');
% p = 1:valid_frames;
% q = squeeze(lidar_pos(1,3,:));
% figure(3);
% scatter(p,q,'filled');

mocap_to_lidar_change_of_basis = [0 1 0 0; 1 0 0 0; 0 0 -1 0; 0 0 0 1];
new_basis = mocap_to_lidar_change_of_basis;
%CCW1
distance(1,:) = [318,1834];
distance(2,:) = [2792,5156];
distance(3,:) = [5779,6880];
distance(4,:) = [7422,8925];
distance(5,:) = [9625,11170];
distance(6,:) = [11850,13270];
for i = 1:6
    lidar_pos_mean = mean(lidar_pos(1,:,distance(i,1):distance(i,2)),3);
    lidar_rot_mean = mean(lidar_rot(1,:,distance(i,1):distance(i,2)),3);
    tag_pos_mean = mean(tag_pos(1,:,distance(i,1):distance(i,2)),3);
    tag_rot_mean = mean(tag_rot(1,:,distance(i,1):distance(i,2)),3);
    lidar_rotm = reshape(lidar_rot_mean,3,3);
    tag_rotm = reshape(tag_rot_mean,3,3);
    world_H_tag = createSE3(tag_rotm, tag_pos_mean');
    world_H_lidar = createSE3(lidar_rotm, lidar_pos_mean');
    lidar_H_tag =  world_H_lidar \ world_H_tag;
    lidar_H_tag_t = new_basis * lidar_H_tag * inv(new_basis);
    angles = rotm2eul(lidar_H_tag_t(1:3,1:3), 'XYZ');
    
%     angles = fliplr(angles)';
%     angles    = flipud(new_basis*angles)';
    tag_rotm_lidar(i,:) = angles*180/pi;
    tag_pos_lidar(i,:) =lidar_H_tag_t(1:3,4)/1000;
end
tag_rotm_lidar
tag_pos_lidar
for i = 1:6
    file_name = strcat('ccw1-',int2str(i),'.txt');
    x = dlmread(file_name,',',1,0);
    pose_prediction_L(i,:) = mean(x(:,4:6),1);
    rot_prediction_L(i,:) = mean(x(:,7:9),1)*180/pi;
end
pose_prediction_L
rot_prediction_L
figure(1);
delta_x_meter = pose_prediction_L(:,1) - tag_pos_lidar(:,1);
plot(tag_pos_lidar(:,1),delta_x_meter);
figure(2);
delta_y_meter = pose_prediction_L(:,2) - tag_pos_lidar(:,2);
plot(tag_pos_lidar(:,1),delta_y_meter);
figure(3);
delta_z_meter = pose_prediction_L(:,3) - tag_pos_lidar(:,3);
plot(tag_pos_lidar(:,1),delta_z_meter);
figure(4);
delta_r_degree = rot_prediction_L(:,1) - tag_rotm_lidar(:,1);
plot(tag_pos_lidar(:,1),delta_r_degree);
figure(5);
delta_p_degree = rot_prediction_L(:,2) - tag_rotm_lidar(:,2);
plot(tag_pos_lidar(:,1),delta_p_degree);
figure(6);
delta_yaw_degree = rot_prediction_L(:,3) - tag_rotm_lidar(:,3);
plot(tag_pos_lidar(:,1),delta_yaw_degree);
Distance_meter = tag_pos_lidar(:,1);
T = table(Distance_meter,delta_x_meter,delta_y_meter,delta_z_meter,delta_r_degree,delta_p_degree,delta_yaw_degree)