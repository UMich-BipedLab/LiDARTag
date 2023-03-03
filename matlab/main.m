clear, clc
addpath(genpath("/home/brucebot/workspace/matlab_utils"));
addpath("/home/brucebot/workspace/lc-calibration/L1_relaxation")
loadLibraries(2);

analyze_package = 1; % analyze each step of the pipeline
event = 2; % which date
dataset = 2; % 1: front 2: ccw
tag_size = 1.22;



%% Which date and which datasets
switch event
    case 1
        event_name = "Oct01-2020/";
        dataset = dataset; 
         % dataset: 1
    case 2
        event_name = "Oct07-2020/";
        dataset = dataset; 
        % dataset: 1 or 2
    case 3
        event_name = "Oct11-2020/";
        % dataset: 3 or 4
        dataset = 2 + dataset; 
    case 4
        event_name = "Oct13-2020/";
        % dataset: 5 or 6
        dataset = 4 + dataset; 
end


%% Load mocap corners and assign correspondances
switch dataset
    case 1
        prefix = "ccw1-";
        mocap_corners = [-1231.2, -678.92, -1486.8, -2039.1;
                        7823,  7674.8,  8384.2,  8532.5;
                     -1.1026,   -1043, -1562.1, -520.13] ./ 1000;
        % top, right, bottom, left
        change_coordinate = [0, 0, 180];
        corner_order = [1, 2, 3, 4];
        
    case 2
        prefix = "ccw1-";
        mocap_corners = [-1231.2, -678.92, -1486.8, -2039.1;
                        7823,  7674.8,  8384.2,  8532.5;
                     -1.1026,   -1043, -1562.1, -520.13] ./ 1000;
        change_coordinate = [0, 0, -180];
        % top, right, bottom, left
        corner_order = [1, 2, 3, 4];
    case 3
        prefix = "front"; 
        % right, bottom, left, top
        mocap_corners = [-463.5, -1.397e3, -2.139e3, -1.21e3;
                        1.04e4,  1.05e4,  1.023e4,  1.011e4;
                     -893.48,   -1.629e3, -740.55, -5.4412] ./ 1000;
        change_coordinate = [0, 0, 180];
        corner_order = [2, 3, 4, 1];
    case 4
        prefix = "ccw1-";
        corner_order = [2, 3, 4, 1];
        mocap_corners = [-444.32, -1.21e3, -1.97e3, -1.2e3;
                    1.01e4, 1.07e4, 1.078e4, 1.021e4;
                    -920.093, -1.62e3, -708.135, -1.0498] ./ 1000;
    case 5
        prefix = "front"; 
        % right, bottom, left, top
        mocap_corners = [-1207.00473282087,-162.398350868965,407.243820707562,-645.857194240357
                          8849.07949318208,8649.68879356377,8788.08379357059,8982.60086074787
                          -664.681198185466,-134.780774365747,-1182.68393961901,-1712.23067661678] ./ 1000;
        change_coordinate = [0, 0, 180];
        corner_order = [2, 3, 4, 1];
%         corner_order = [1,2,3,4];
    case 6
        prefix = "ccw1-";
        corner_order = [2, 3, 4, 1];
        % right, bottom, left, top
        mocap_corners = [
            -1048.59669393742,-340.305804296641,625.935265764107,-88.8040158428149
             8880.42788298431,8660.87423516959,8709.80064463733,8930.80568283424
            -1033.10454793262,-101.79535034276,-815.690194430646,-1745.58145924785] ./ 1000;
        change_coordinate = [0, 0, -180];
end


%% Path
root_path = "./paper_data_pre/";
lidar_path = root_path + "gt_lidar/" + event_name;
lt_estimate_path = root_path + "lidartag_estimates/" + event_name;
% camera_path = root_path + "gt_camera/" + event_name;
lt_estimate_folers = dir(lt_estimate_path + prefix + "*");
num_estimates = size(lt_estimate_folers, 1);



%% Load data from txt files (ground truth data)
lidar_files = dir(lidar_path + prefix + "*.txt");
num_lidar_files = length(lidar_files);
% camera_files = dir(camera_path + prefix + "*.txt");
% num_camera_files = length(camera_files);
% num_data = min(num_lidar_files, num_camera_files);
num_data = num_lidar_files;

ground_truth(num_data) = struct();
for i = 1:num_data
    % find idar file in camera files
    ground_truth(i).lidar_file = lidar_files(i).name;
%     index = find(strcmp({camera_files.name}, ...
%                         ground_truth(i).lidar_file)==1);
    
    ground_truth(i).name = ...
        ground_truth(i).lidar_file(...
        1:strfind(ground_truth(i).lidar_file, '.') - 1);
    
    % load lidar data
    lidar_data = ...
        mean(dlmread(lidar_path + lidar_files(i).name, ',', 1, 0));
    ground_truth(i).lidar_corners = ...
        [lidar_data(2:4)', lidar_data(5:7)', ...
         lidar_data(8:10)', lidar_data(11:13)'];
    
%     % load camera data
%     ground_truth(i).camera_file = camera_files(index).name;
%     camera_data = ...
%         mean(dlmread(camera_path + ...
%         ground_truth(i).camera_file, ',', 1, 0));
%     ground_truth(i).camera_corners = ...
%         [camera_data(2:4)', camera_data(5:7)', ...
%          camera_data(8:10)', camera_data(11:13)'];
%      
%     % assert if names are not the same 
%     assert(...
%         strcmp(ground_truth(i).camera_file, ...
%         ground_truth(i).lidar_file), "files mismatch")
    
    
    out_t = computePoseFromLiDARToMocapMarkers(...
        ground_truth(i).lidar_corners, mocap_corners, tag_size, corner_order);
    
    for fn = fieldnames(out_t)'
       ground_truth(i).(fn{1}) = out_t.(fn{1});
    end
end


%% Load estimated LiDARTag pose/ID information
lidartag(length(num_estimates)) = struct('name', [], 'pose_file', [],...
    'pose_raw_data', [], 'iter', [], ...
    'id', [], 'rot_num', [], 'rotm', [], 'translation', [], ...
    'L_H_LT', [], 'rpy', [], ...
    'num_data', [], 'computation_hz', [], 'computation_mean', [],...
    'decoding_hz', [], 'decoding_mean', [], ...
    'timing_hz', [], 'timing_mean', [], 'clusters', []);
for i = 1:num_estimates
    lidartag(i).name = prefix + num2str(i);
    lt_current_path = lt_estimate_path + lidartag(i).name + "/";
    lidartag(i).pose_file = dir(lt_current_path + "*1.2*pose.txt");
    
    if isempty(lidartag(i).pose_file)
        continue
    end
    
    % pose data
    lidartag(i).pose_raw_data = dlmread(...
        lt_current_path + lidartag(i).pose_file.name, ',', 1, 0);
    
    lidartag(i).iter = lidartag(i).pose_raw_data(:, 1);
    lidartag(i).id = lidartag(i).pose_raw_data(:, 2);
    lidartag(i).rot_num = lidartag(i).pose_raw_data(:, 3);
    [lidartag(i).rotm, lidartag(i).translation] = ...
        computeMeanOfTranslationNRotation(...
        lidartag(i).pose_raw_data(:, 7:end), ...
        lidartag(i).pose_raw_data(:, 4:6));
    lidartag(i).L_H_LT = eye(4);
    lidartag(i).L_H_LT(1:3, 1:3) = lidartag(i).rotm;
    lidartag(i).L_H_LT(1:3, 4) = lidartag(i).translation;
    lidartag(i).rpy = ...
        rad2deg(rotm2eul(lidartag(i).L_H_LT(1:3, 1:3), 'XYZ'));  
    
    if analyze_package
        lidartag(i) = analyzeLiDARTagPackage(lidartag(i), lt_current_path);
    else
        lidartag(i).num_data = size(lidartag(i).pose_raw_data, 1);
    end
end


%% Compare with ground truth
gt_ID = 0;
gt_rot_num = 3;
results(num_data) = struct('name', [], 'distance', [], 'num_scans', [], ...
    'ID_ratio', [], 'translation', [], 'geodesic', []);
for i = 1:num_estimates
    if (isempty(lidartag(i).pose_raw_data))
        continue;
    end
    
    index = find(strcmp({ground_truth.name}, lidartag(i).name)==1);
    if isempty(index)
        continue;
    end
    
    results(i).name = lidartag(i).name;
    results(i).distance = norm(ground_truth(index).translation);
    results(i).num_scans = lidartag(i).num_data;
    results(i).ID = length(find(abs((lidartag(i).id - gt_ID))));
    results(i).rot_num = ...
        length(find(abs((lidartag(i).rot_num - gt_rot_num))));
    
    results(i).ID_ratio = results(i).ID / lidartag(i).num_data;
    results(i).rot_num_ratio = ...
        results(i).rot_num / lidartag(i).num_data;

    i;
    % translation is in mm
    results(i).translation = ...
        (ground_truth(index).translation - lidartag(i).translation) * 1000;
    results(i).translation = ...
        norm((ground_truth(index).translation - lidartag(i).translation) * 1000);

    results(i).rpy = ...
        ground_truth(index).rpy - lidartag(i).rpy + change_coordinate;
    results(i).geodesic = rad2deg(norm(Log_SO3(eul2rotm(deg2rad(results(i).rpy), "XYZ"))));
    results(i).dH = ground_truth(index).L_H_LT / lidartag(i).L_H_LT;
end

disp("===============================================================")
disp("=========== Results of Decoding and Pose Estimation ===========")
disp("===============================================================")
results(all(cell2mat(arrayfun( @(x) structfun( @isempty, x), ...
        results, 'UniformOutput', false)), 1)) = [];
struct2table(results)


%% Package general analysis
if analyze_package
    disp("===============================================================")
    disp("================= Results of General Analysis =================")
    disp("===============================================================")
    for i = 1:num_estimates
        dataset_num = i;
        if isempty(lidartag(dataset_num).num_data)
            warning("No results for %s", lidartag(dataset_num).name)
        else
            disp("-------------------------------------------------------")
            fprintf("Each of below average over %i scans ", ...
                lidartag(dataset_num).num_data)
            fprintf("from %s dataset: \n", lidartag(dataset_num).name)
            disp("-------------------------------------------------------")
            struct2table(lidartag(dataset_num).computation_hz)
            
            if isempty(lidartag(dataset_num).timing_hz)
                 warning("No timing_hz for %s dataset", ...
                     lidartag(dataset_num).name)
            else
                struct2table(lidartag(dataset_num).timing_hz)
            end
            
            struct2table(lidartag(dataset_num).clusters)

            if isempty(lidartag(dataset_num).decoding_hz)
                 warning("No decoding_hz for %s dataset", ...
                     lidartag(dataset_num).name)
            else
                struct2table(lidartag(dataset_num).decoding_hz)
            end
        end
    end

    %% summary
    disp("===============================================================")
    disp("============= Summary Results of General Analysis =============")
    disp("===============================================================")

    summary_t = summarizeGeneralAnalysis(lidartag, "hz");
    disp("-----------------------------------------------------------")
    fprintf("Average Hz over all dadtasets with %i scans \n", ...
        summary_t.total_scans)
    disp("-----------------------------------------------------------")

    t1 = struct2table(summary_t.computation_hz)
    t2 = struct2table(summary_t.timing_hz)
    t3 = struct2table(summary_t.clusters)
    
    if isfield(summary_t, 'decoding_hz')
        t4 = struct2table(summary_t.decoding_hz)
    end


    disp("-----------------------------------------------------------")
    fprintf("Average ms over all dadtasets with %i scans \n", ...
        summary_t.total_scans)
    disp("-----------------------------------------------------------")
    summary_t = summarizeGeneralAnalysis(lidartag, "mean");
    t1 = struct2table(summary_t.computation_mean)
    t2 = struct2table(summary_t.timing_mean)
    t3 = struct2table(summary_t.clusters)
    if isfield(summary_t, 'decoding_mean')
        t4 = struct2table(summary_t.decoding_mean)
    end
end
% t5 = array2table(table2array(struct2table(summary_t.computation_hz)).');
% t5.Properties.RowNames = t1.Properties.VariableNames;
% t5.Properties.VariableNames = "Hz";
% t5





%% Plottings
% X = ground_truth(1).lidar_corners'; % n x 3
% Y = mocap_corners'; % n x 3
% [~, mocap_at_lidar, ~] = procrustes(X, Y, 'scaling', 0, 'reflection', 0);
% 
% template = [0, 0, 0, 0;
%             tag_size/2, -tag_size/2, -tag_size/2 ,tag_size/2;
%             tag_size/2, tag_size/2, -tag_size/2, -tag_size/2]';
% [d, ~, transform] = ...
% procrustes(mocap_at_lidar, template, 'scaling', 0, 'reflection', 0);
% H_ML = eye(4);
% H_ML(1:3, 1:3) = transform.T';
% H_ML(1:3, 4) = transform.c(1, :)';
% mocap_lidar = H_ML * convertToHomogeneousCoord(template');
% 
% 
% 
% [axes_h, fig_h] = createFigHandleWithNumber(2, 1, "pose", 1, 1);
% cur_axes = axes_h(1);
% h1 = scatter3(cur_axes, X(:, 1), X(:, 2), X(:, 3), 'ko');
% h2 = scatter3(cur_axes, Y(:, 1), Y(:, 2), Y(:, 3), 'b.'); 
% h3 = scatter3(cur_axes, ...
%               mocap_at_lidar(:, 1), ...
%               mocap_at_lidar(:, 2), ...
%               mocap_at_lidar(:, 3), 'rx');
% legend([h1, h2, h3], ...
%       ["L_lidar_corners", "M_mocap_corners", "L_mocap_corners"])
% plotColoredOriginAxisWithText(cur_axes, "LiDAR", eye(4), 0.5)
% showCurrentPlot(cur_axes, "Mocap to LiDAR", [-50, 30])
% 
% 
% cur_axes = axes_h(2);
% scatter3(cur_axes, ...
%          template(:, 1), template(:, 2), template(:, 3), ...
%          'fill', 'ko')
% scatter3(cur_axes, X(:, 1), X(:, 2), X(:, 3), 'bo')    
% scatter3(cur_axes, ...
% mocap_lidar(1, :), mocap_lidar(2, :), mocap_lidar(3, :), 'r*')
% % scatter3(...
% cur_axes, mocap_lidar(:, 1), mocap_lidar(:, 2), mocap_lidar(:, 3), 'r*')
% plotColoredOriginAxisWithText(cur_axes, "LiDAR", eye(4), 0.5)
% showCurrentPlot(cur_axes, "LiDAR", [-70, 10])

















