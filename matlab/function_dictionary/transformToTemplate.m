function out_t = transformToTemplate(opt, opts, X)

    [~, centroid, U] = computePlaneReturnR(X(1:3,:));
    if abs(det(U) -1) > 1e-5
        rotm_init = U(:, [3 2 1])';
    else
        rotm_init = U(:, [3 1 2])';
    end

    out_t.target_size = opts.target_size;
    opt.UseCentroid = 0;
    opt.rpy_init = rad2deg(rotm2eul(rotm_init, "XYZ"));
    % opt.rpy_init = rad2deg(rotm2eul(rotm_init));
    opt.T_init = -rotm_init*centroid(1:3);
    opt.simulation_data = 0; % don't change, used in findSuitableVertices_v2.m
    opt.method = "Constraint Customize Lie Group";  

    % clean up data
    [~, ~, clean_up_indices, ~] = removeLiDARTargetOutliers(X, out_t.target_size, opt);
    out_t.X_clean = X(:, clean_up_indices);

    % optimize pose and vertices
    % opt.T_init = -rotm_init*centroid(1:3) + [-0.1 -0.5 -0.5]';

    opt = optimizeCost(opt, out_t.X_clean, out_t.target_size, opts.box_width);
    out_t.cost = opt.opt_total_cost;
    out_t.computation_time = opt.computation_time;
    [~, X_h] = checkHomogeneousCoord(out_t.X_clean(1:3, :), 1);
    pionts_ideal_frame = opt.H_opt * X_h; % 
    out_t.pionts_ideal_frame = [pionts_ideal_frame(1:3, :); out_t.X_clean(4, :)];
    out_t.projected_vertices = opt.H_opt\converToHomogeneousCoord(opt.ideal_frame);


    out_t.H_init = [rotm_init, opt.T_init; 0 0 0 1];   
    out_t.pionts_initial_guess = out_t.H_init * X_h; % 
    out_t.pionts_initial_guess = [out_t.pionts_initial_guess(1:3, :); out_t.X_clean(4, :)];
end