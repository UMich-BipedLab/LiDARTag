function out_t = loadAprilTagPointCloud(opts, tag_num)
    if ~isfield(opts, 'file_type')
         opts.file_type = ".csv";
    end
    
    if ~isfield(opts, 'num_tag')
        opts.num_tag = 30;
    end
    if ~isfield(opts, 'rotation')
        opts.rotation = -90;
    end
    if ~isfield(opts, 'v_flip')
        opts.v_flip = 0;
    end
    img_path = opts.img_path;
    img_ary = num2str([0:opts.num_tag-1].',opts.img_prefix);
%     name_image_array = 


    cur_img = img_ary(tag_num ,:);
    img_name = string(cur_img(1:strfind(cur_img,'.')-1)) + "_planer" + opts.file_type;
    img_geo_name = string(cur_img(1:strfind(cur_img,'.')-1)) + "_geometry" + opts.file_type;
    
    % image point cloud
    if isfile(opts.load_path + img_name)
        if opts.file_type == ".csv"
            image_array = load(opts.load_path + img_name);
        else
            load(opts.load_path + img_name);
        end
        
        displayMessages(opts, img_name + " loaded", 3);
    else
        img_dict = imread(img_path + cur_img); % Load LiDARTag
        if opts.v_flip
            img_dict = flip(img_dict, 2);           % horizontal flip
        end
        img_dict = imrotate(img_dict, opts.rotation);
        img_dict = rgb2gray(img_dict)./255;
        [img_dict] = imresize(img_dict, 3, 'nearest');
        scale_ratio = size(img_dict)/opts.target_size;

        image_array = zeros(4, size(img_dict, 1)*size(img_dict, 2));
        counter = 1;
        for i = 1:size(img_dict, 1)
            for j = 1:size(img_dict, 2)
                current_intensity = img_dict(i, j);
                point =  [0; double(i/scale_ratio(1)); double(j/scale_ratio(2)); double(current_intensity)];
                image_array(:, counter) = point;
                counter = counter + 1;
            end
        end
        image_array_mean = mean(image_array(1:3,:), 2);
        image_array(1:3, :) = image_array(1:3, :) - image_array_mean;
        
        if opts.file_type == ".csv"
            writematrix(image_array, opts.save_path + img_name, 'Delimiter', ',')
        else
            save(opts.save_path + img_name, 'image_array');
        end
        displayMessages(opts, img_name + " saved", 3);
    end
    
    if isfile(opts.load_path + img_geo_name)
        if opts.file_type == ".csv"
            geometry_img = load(opts.load_path + img_geo_name);
        else
            load(opts.load_path + img_geo_name);
        end
        displayMessages(opts, img_geo_name + " loaded", 3);
    else
        geometry_img = constructGeometryPointCloud(image_array, opts.ell);
        
        if opts.file_type == ".csv"
            writematrix(geometry_img, opts.save_path + img_geo_name, 'Delimiter', ',')
        else
            save(opts.save_path + geometry_img, 'geometry_img');
        end
        displayMessages(opts, img_geo_name + " saved", 3);
    end
    out_t.image_name = cur_img;
    out_t.image_array = image_array;
    out_t.geometry_img = geometry_img;
end