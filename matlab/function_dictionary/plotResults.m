function plotResults(opts, image_array, ideal_object_list, ...
                     X, X_clean, pionts_ideal_frame, projected_vertices,...
                     geometry_img, geometry_pc)
    [axes_h, ~] = createFigHandleWithNumber(5, 1, "overlayAprilTag", 1, 1);
    cur_axes = axes_h(1);
    AprilTag_map = genColorMap(image_array(4,:), 10, 1);
    scatter3(cur_axes, image_array(1,:), image_array(2,:), image_array(3,:), [], AprilTag_map, 'fill')
    plotObjectsList(cur_axes, opts, [], ideal_object_list)
    plotOriginalAxisWithText(cur_axes, "LiDAR")
    LiDARTag_map = genColorMap(X(4,:), 10, 1);
    scatter3(cur_axes, X(1,:), X(2,:), X(3,:), [], LiDARTag_map, 'fill')
    viewCurrentPlot(cur_axes, "Ideal LiDARTag with AprilTag", [-60 20], 1)
    reloadCurrentPlot(cur_axes, 1)

    cur_axes = axes_h(2);
    scatter3(cur_axes, X(1,:), X(2,:), X(3,:), [], 'fill', 'go')
    clean_LiDARTag_map = genColorMap(X_clean(4, :), 10, 1);
    scatter3(cur_axes, X_clean(1,:), X_clean(2,:), X_clean(3,:), [], clean_LiDARTag_map, 'fill')
    scatter3(cur_axes, pionts_ideal_frame(1,:), pionts_ideal_frame(2,:), pionts_ideal_frame(3,:), [], clean_LiDARTag_map, 'fill')
    scatter3(cur_axes, projected_vertices(1,:), projected_vertices(2,:), projected_vertices(3,:), 'fill', 'ro')
    plotObjectsList(cur_axes, opts, [], ideal_object_list)
    viewCurrentPlot(cur_axes, "Cleaned LiDARTag", [-70 10], 1)



    cur_axes = axes_h(3);
    scatter3(cur_axes, geometry_img(1,:), geometry_img(2,:), geometry_img(3,:), [], AprilTag_map, 'fill')
    plotObjectsList(cur_axes, opts, [], ideal_object_list)
    plotOriginalAxisWithText(cur_axes, "LiDAR")
    LiDARTag_map = genColorMap(X(4,:), 10, 1);
    scatter3(cur_axes, X(1,:), X(2,:), X(3,:), [], LiDARTag_map, 'fill')
    viewCurrentPlot(cur_axes, "Geometry AprilTag", [-60 20], 1)
    reloadCurrentPlot(cur_axes, 1)

    cur_axes = axes_h(4);
    scatter3(cur_axes, X(1,:), X(2,:), X(3,:), [], 'fill', 'go')
    scatter3(cur_axes, X_clean(1,:), X_clean(2,:), X_clean(3,:), [], clean_LiDARTag_map, 'fill')
    scatter3(cur_axes, geometry_pc(1,:), geometry_pc(2,:), geometry_pc(3,:), [], clean_LiDARTag_map, 'fill')
    scatter3(cur_axes, projected_vertices(1,:), projected_vertices(2,:), projected_vertices(3,:), 'fill', 'ro')
    plotObjectsList(cur_axes, opts, [], ideal_object_list)
    viewCurrentPlot(cur_axes, "Geometry LiDARTag", [-70 10], 1)
end