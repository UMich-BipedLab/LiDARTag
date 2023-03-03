clear 
file = load('big-3.mat');
total_frames = file.big_3.Frames;
for k = 1: file.big_3.RigidBodies.Bodies
    if strcmp(file.big_3.RigidBodies.Name(k), 'lidar')
        lidar_pos = file.big_3.RigidBodies.Positions(k,:,:);
        lidar_rot = file.big_3.RigidBodies.Rotations(k,:,:);
    else
        tag_pos = file.big_3.RigidBodies.Positions(k,:,:);
        tag_rot = file.big_3.RigidBodies.Rotations(k,:,:);
    end
end
invalid_num = 0;
for m = 1: total_frames
    lidar_invalid = any(isnan(lidar_pos(:,:,m))) || any(isnan(lidar_rot(:,:,m)));
    tag_invalid = any(isnan(tag_pos(:,:,m))) || any(isnan(tag_rot(:,:,m)));
    if (lidar_invalid || tag_invalid)
        lidar_pos(:,:,m) = zeros(1,3);
        lidar_rot(:,:,m) = zeros(1,9);
        tag_pos(:,:,m) = zeros(1,3);
        tag_rot(:,:,m) = zeros(1,9);
        invalid_num = invalid_num + 1;
    end
end
lidar_pos_mean = sum(lidar_pos,3)/(total_frames - invalid_num);
lidar_rot_mean = sum(lidar_rot,3)/(total_frames - invalid_num);
tag_pos_mean = sum(tag_pos,3)/(total_frames - invalid_num);
tag_rot_mean = sum(tag_rot,3)/(total_frames - invalid_num);
lidar_rotm = reshape(lidar_rot_mean,3,3);
tag_rotm = reshape(tag_rot_mean,3,3);
world_H_tag = createSE3(tag_rotm, tag_pos_mean');
world_H_lidar = createSE3(lidar_rotm, lidar_pos_mean');
lidar_H_tag =  world_H_lidar \ world_H_tag;
angles = rotm2eul(lidar_H_tag(1:3,1:3), 'ZYX');
mocap_to_lidar_change_of_basis = [0 -1 0; 1 0 0; 0 0 1];
new_basis = mocap_to_lidar_change_of_basis;
angles = fliplr(angles)';
angles    = flipud(new_basis*angles)';
tag_rotm_lidar = angles;
tag_pos_lidar = new_basis*lidar_H_tag(1:3,4);
figure(1)
vector(1,:) = tag_rotm_lidar;
vector(2,:) = tag_pos_lidar/1000;
linespec='-'

quiver3(zeros(2,1),zeros(2,1),zeros(2,1),vector(:,1), vector(:,2),vector(:,3),0,linespec)
