function out_t = ...
    computePoseFromLiDARToMocapMarkers(L_X, M_Y, tag_size, corner_order)
    % X: Target vertices estiamted from the lidar frame 
    % Y: Mocap makers measured from the mocap frame
    % Assume X and Y are the same but measured from different sensors, 
    % Here, the sensors are mocap and lidar

    % Transform data from 3xn to nx3
    L_X = L_X'; % n x 3
    M_Y = M_Y'; % n x 3
    
    % Construct template at the lidar origin
    template = [0, 0, 0, 0;
                tag_size/2, -tag_size/2, -tag_size/2 ,tag_size/2;
                tag_size/2, tag_size/2, -tag_size/2, -tag_size/2];
    template = template(:, corner_order)';
            

    
    % Estimate H from Y to X, which results in Y measured in LiDAR frame
    [cost_ML, L_Y, ~] = procrustes(L_X, M_Y, 'scaling', 0, 'reflection', 0);
    
    
    % Esimate the H from the template at the lidar origin to the markers at
    % LiDAR frame (L_marker)
    [cost_LT, ~, transform] = procrustes(L_Y, template, 'scaling', 0, 'reflection', 0);
    
    
    % L_template_to_target and L_Y should be close to each other
    H_ML = eye(4);
    H_ML(1:3, 1:3) = transform.T';
    H_ML(1:3, 4) = transform.c(1, :)';
    L_template_to_target = H_ML * convertToHomogeneousCoord(template');

    out_t.cost_ML = cost_ML;
    out_t.cost_LT = cost_LT;
    out_t.L_markers = L_Y;
    out_t.L_template_to_target = L_template_to_target;
    out_t.L_H_LT = H_ML;
    out_t.translation = H_ML(1:3, 4)';
    out_t.rpy = rad2deg(rotm2eul(H_ML(1:3, 1:3), "XYZ"));
    
end
