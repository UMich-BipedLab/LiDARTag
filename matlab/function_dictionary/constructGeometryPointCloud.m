function geometry_pc = constructGeometryPointCloud(point_cloud, separation)
    if ~exist('separation', 'var')
        separation = 0.1;
    end
    average_intensity = mean(point_cloud(4,:));
    offset_intensity = point_cloud(4,:) - average_intensity;
    
    pos_ind = offset_intensity>=0;
    pos_intensity = offset_intensity(pos_ind);
    pos_threshold = median(pos_intensity);
    pos_intensity = pos_intensity/abs(pos_threshold);
    
    neg_ind = offset_intensity<0;
    neg_intensity = offset_intensity(neg_ind);
    neg_threshold = median(neg_intensity);
    neg_intensity = neg_intensity/abs(neg_threshold);
    
    offset_intensity(pos_ind) = separation * pos_intensity;
    offset_intensity(neg_ind) = separation * neg_intensity;
%     offset_intensity = offset_intensity/threshold;
    geometry_pc = point_cloud;
    geometry_pc(1,:) = offset_intensity;
end