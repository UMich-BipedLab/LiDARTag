function [out_t, opts, opt] = loadData(opts, dataset)
    path = "./test_mats/";
    if dataset==1
        name = "velodyne_points-lab3-closer-big--2019-09-06-08-38.mat";
        opts.target_size = 0.8051;
        opts.rotation = -90;
%         opts.rotation = 0;
        opts.v_flip = 0;
    elseif dataset==2
        name = "velodyne_points-lab4-closer-big--2019-09-06-13-49.mat";
        opts.target_size = 0.8051;
        opts.rotation = -90;
        opts.v_flip = 1;
    elseif dataset==3
        name = "velodyne_points-lab5-closer-bag--2019-09-06-14-27.mat";
        opts.target_size = 0.8051;
        opts.rotation = 180;
        opts.v_flip = 1;
    elseif dataset==4
        name = "velodyne_points-lab6-closer-big--2019-09-06-15-09.mat";
        opts.target_size = 0.8051;
        opts.rotation = 0;
        opts.v_flip = 0;
    elseif dataset==5
        name = "velodyne_points-lab7-closer-big--2019-09-06-15-14.mat";
        opts.target_size = 0.8051;
        opts.rotation = 0;
        opts.v_flip = 0;
    elseif dataset==6
        name = "velodyne_points-lab8-closer-big--2019-09-06-15-28.mat";
        opts.target_size = 0.8051;
        opts.rotation = 0;
        opts.v_flip = 0;
    elseif dataset==7
        name = "velodyne_points-lab3-closer-small--2019-09-06-08-35.mat";
        opts.target_size = 0.158;
        opts.target_size = 0.16;
    end
    pc = loadPointCloud(path, name);
    out_t.X = getPayloadWithIntensity(pc, 1, 1);
    out_t.X(4, :) = out_t.X(4, :)./255;
    
    std_noise = estimatePlanarNoise(out_t.X);
    opts.box_width = 2*std_noise(3);

    % ideal frame
    opts.polygon = 4;
    opts.rpy = [0, 0, 0];
    opts.xyz = [0, 0, 0];
    [ideal_object_list, ~] = createDynamicScene(opts);
    opt.ideal_frame = convertXYZstructToXYZmatrix(ideal_object_list.object_vertices);
    out_t.ideal_object_list = ideal_object_list;
end