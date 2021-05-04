function SingleTextData = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,i)
    SingleTextData.change_coordinate = change_coordinate;
    SingleTextData.tag_size = tag_size;
    lidar_path = root_path + "gt_lidar/";
    lt_estimate_path = root_path + "lidartag_estimates/";
    lidar_files = dir(lidar_path + prefix + "*.txt");
    lidar_file = lidar_files(i).name;
    SingleTextData.name = ...
        lidar_file(...
        1:strfind(lidar_file, '.') - 1);
    lidar_data = ...
        mean(dlmread(lidar_path + lidar_files(i).name, ',', 1, 0));
    SingleTextData.lidar_corners = ...
        [lidar_data(2:4)', lidar_data(5:7)', ...
         lidar_data(8:10)', lidar_data(11:13)'];    
    out_t = computePoseFromLiDARToMocapMarkers(...
        SingleTextData.lidar_corners, mocap_corners, tag_size, corner_order);
    
    for fn = fieldnames(out_t)'
       SingleTextData.(fn{1}) = out_t.(fn{1});
    end
  
    
    name = prefix + num2str(i);
    lt_current_path = lt_estimate_path + name + "/";
    pose_file = dir(lt_current_path + "*1.2*pose.txt");
    
    % pose data
    pose_raw_data = dlmread(lt_current_path + pose_file.name, ',', 1, 0);
    SingleTextData.pose_raw_data = pose_raw_data;
    SingleTextData.iter = pose_raw_data(:, 1);
    SingleTextData.id = pose_raw_data(:, 2);
    SingleTextData.rot_num = pose_raw_data(:, 3);
    [SingleTextData.rotm, SingleTextData.est_translation] = ...
        computeMeanOfTranslationNRotation(...
        pose_raw_data(:, 7:end), ...
        pose_raw_data(:, 4:6));
    SingleTextData.est_L_H_LT = eye(4);
    SingleTextData.est_L_H_LT(1:3, 1:3) = SingleTextData.rotm;
    SingleTextData.est_L_H_LT(1:3, 4) = SingleTextData.est_translation;
    SingleTextData.est_rpy = ...
        rad2deg(rotm2eul(SingleTextData.est_L_H_LT(1:3, 1:3), 'XYZ')); 
    
    % General Analysis
    SingleTextData = analyzeLiDARTagPackage(SingleTextData, lt_current_path);
end