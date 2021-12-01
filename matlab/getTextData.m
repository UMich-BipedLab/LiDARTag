function TextData = getTextData(opts)
    root_path = opts.data_path;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct11-2020/front1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct11-2020/";
    prefix = "front"; 
    mocap_corners = [-463.5, -1.397e3, -2.139e3, -1.21e3;
                        1.04e4,  1.05e4,  1.023e4,  1.011e4;
                     -893.48,   -1.629e3, -740.55, -5.4412] ./ 1000;
    change_coordinate = [0, 0, 180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    
    TextData(1) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct11-2020/front2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct11-2020/";
    prefix = "front"; 
    mocap_corners = [-463.5, -1.397e3, -2.139e3, -1.21e3;
                        1.04e4,  1.05e4,  1.023e4,  1.011e4;
                     -893.48,   -1.629e3, -740.55, -5.4412] ./ 1000;
    change_coordinate = [0, 0, 180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    TextData(2) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,2);
    

   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct11-2020/front3
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct11-2020/";
    prefix = "front"; 
    mocap_corners = [-463.5, -1.397e3, -2.139e3, -1.21e3;
                        1.04e4,  1.05e4,  1.023e4,  1.011e4;
                     -893.48,   -1.629e3, -740.55, -5.4412] ./ 1000;
    change_coordinate = [0, 0, 180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    TextData(3) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,3);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct11-2020/front4
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct11-2020/";
    prefix = "front"; 
    mocap_corners = [-463.5, -1.397e3, -2.139e3, -1.21e3;
                        1.04e4,  1.05e4,  1.023e4,  1.011e4;
                     -893.48,   -1.629e3, -740.55, -5.4412] ./ 1000;
    change_coordinate = [0, 0, 180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    TextData(4) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,4);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct11-2020/front5
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct11-2020/";
    prefix = "front"; 
    mocap_corners = [-463.5, -1.397e3, -2.139e3, -1.21e3;
                        1.04e4,  1.05e4,  1.023e4,  1.011e4;
                     -893.48,   -1.629e3, -740.55, -5.4412] ./ 1000;
    change_coordinate = [0, 0, 180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    TextData(5) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,5);


    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct7-2020/front6
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct07-2020/";
    prefix = "front"; 
    mocap_corners = [-1231.2, -678.92, -1486.8, -2039.1;
                        7823,  7674.8,  8384.2,  8532.5;
                     -1.1026,   -1043, -1562.1, -520.13] ./ 1000;
    change_coordinate = [0, 0, 180];
    corner_order = [1, 2, 3, 4];
    tag_size = 1.22;
    TextData(6) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,6);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct7-2020/front7
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct07-2020/";
    prefix = "front"; 
    mocap_corners = [-1231.2, -678.92, -1486.8, -2039.1;
                        7823,  7674.8,  8384.2,  8532.5;
                     -1.1026,   -1043, -1562.1, -520.13] ./ 1000;
    change_coordinate = [0, 0, 180];
    corner_order = [1, 2, 3, 4];
    tag_size = 1.22;
    TextData(7) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,7);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct13-2020/ccw1-1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct13-2020/";
    prefix = "ccw1-"; 
    mocap_corners = [
            -1048.59669393742,-340.305804296641,625.935265764107,-88.8040158428149
             8880.42788298431,8660.87423516959,8709.80064463733,8930.80568283424
            -1033.10454793262,-101.79535034276,-815.690194430646,-1745.58145924785] ./ 1000;
    change_coordinate = [0, 0, -180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    TextData(8) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,1);



    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct13-2020/ccw1-2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct13-2020/";
    prefix = "ccw1-"; 
    mocap_corners = [
            -1048.59669393742,-340.305804296641,625.935265764107,-88.8040158428149
             8880.42788298431,8660.87423516959,8709.80064463733,8930.80568283424
            -1033.10454793262,-101.79535034276,-815.690194430646,-1745.58145924785] ./ 1000;
    change_coordinate = [0, 0, -180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    TextData(9) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,2);

 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct13-2020/ccw1-3
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct13-2020/";
    prefix = "ccw1-"; 
    mocap_corners = [
            -1048.59669393742,-340.305804296641,625.935265764107,-88.8040158428149
             8880.42788298431,8660.87423516959,8709.80064463733,8930.80568283424
            -1033.10454793262,-101.79535034276,-815.690194430646,-1745.58145924785] ./ 1000;
    change_coordinate = [0, 0, -180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    TextData(10) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,3);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct13-2020/ccw1-4
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct13-2020/";
    prefix = "ccw1-"; 
    mocap_corners = [
            -1048.59669393742,-340.305804296641,625.935265764107,-88.8040158428149
             8880.42788298431,8660.87423516959,8709.80064463733,8930.80568283424
            -1033.10454793262,-101.79535034276,-815.690194430646,-1745.58145924785] ./ 1000;
    change_coordinate = [0, 0, -180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    TextData(11) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct13-2020/ccw1-5
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct13-2020/";
    prefix = "ccw1-"; 
    mocap_corners = [
            -1048.59669393742,-340.305804296641,625.935265764107,-88.8040158428149
             8880.42788298431,8660.87423516959,8709.80064463733,8930.80568283424
            -1033.10454793262,-101.79535034276,-815.690194430646,-1745.58145924785] ./ 1000;
    change_coordinate = [0, 0, -180];
    corner_order = [2, 3, 4, 1];
    tag_size = 1.22;
    TextData(12) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,5);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct07-2020/ccw1-6
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     event_name = "Oct07-2020/";
    prefix = "ccw1-"; 
    mocap_corners = [-1231.2, -678.92, -1486.8, -2039.1;
                        7823,  7674.8,  8384.2,  8532.5;
                     -1.1026,   -1043, -1562.1, -520.13] ./ 1000;
    change_coordinate = [0, 0, -180];
    corner_order = [1, 2, 3, 4];
    tag_size = 1.22;
    TextData(13) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,6);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Oct07-2020/ccw1-7
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    event_name = "Oct07-2020/";
    prefix = "ccw1-"; 
    mocap_corners = [-1231.2, -678.92, -1486.8, -2039.1;
                        7823,  7674.8,  8384.2,  8532.5;
                     -1.1026,   -1043, -1562.1, -520.13] ./ 1000;
    change_coordinate = [0, 0, -180];
    corner_order = [1, 2, 3, 4];
    tag_size = 1.22;
    TextData(14) = getSingleTextData(root_path, prefix, tag_size, change_coordinate, corner_order, mocap_corners,7);
end
    
