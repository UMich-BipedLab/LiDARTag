clear, clc
% addpath(genpath("E:/2020summer_research/matlab/matlab_utils-master"));
% addpath("/home/brucebot/workspace/lc-calibration/L1_relaxation")
loadLibraries(2);
opts.data_path = "./paper_data/";



analyze_package = 1; % analyze each step of the pipeline
% event = 4; % which date
dataset = 2; % 1: front 2: ccw
tag_size = 1.22;
TextData = getTextData(opts);


    
%% Compare with ground truth
gt_ID = 0;
gt_rot_num = 3;
num_estimates = 7;
results(num_estimates) = struct('name', [], 'distance', [], 'num_scans', [], ...
    'ID_ratio', [], 'translation', [], 'geodesic', []);
for i = (1 + num_estimates * (dataset - 1)) : ...
        (num_estimates + num_estimates * (dataset-1))
    if (isempty(TextData(i).pose_raw_data))
        continue;
    end
    
    
    results(i).name = TextData(i).name;
    results(i).distance = norm(TextData(i).translation);
    results(i).num_scans = size(TextData(i).pose_raw_data, 1);
    results(i).ID = length(find(abs((TextData(i).id - gt_ID))));
    results(i).rot_num = ...
        length(find(abs((TextData(i).rot_num - gt_rot_num))));
    
    results(i).ID_ratio = results(i).ID / results(i).num_scans;
    results(i).rot_num_ratio = ...
        results(i).rot_num / results(i).num_scans;

    i;
    % translation is in mm
    results(i).translation = ...
        (TextData(i).translation - TextData(i).est_translation) * 1000;
    results(i).translation = ...
        norm((TextData(i).translation - TextData(i).est_translation) * 1000);

    results(i).rpy = ...
        TextData(i).rpy - TextData(i).est_rpy + TextData(i).change_coordinate;
    results(i).geodesic = rad2deg(norm(Log_SO3(eul2rotm(deg2rad(results(i).rpy), "XYZ"))));
    results(i).dH = TextData(i).L_H_LT / TextData(i).est_L_H_LT;
end

disp("===============================================================")
disp("=========== Results of Decoding and Pose Estimation ===========")
disp("===============================================================")
results(all(cell2mat(arrayfun( @(x) structfun( @isempty, x), ...
        results, 'UniformOutput', false)), 1)) = [];
struct2table(results)

disp("===============================================================")
disp("=========== Summary of Decoding and Pose Estimation ===========")
disp("===============================================================")
disp("-- mean:")
fprintf("---- Number of scans: %.3f\n", mean([results.num_scans]))
fprintf("---- Translation error [mm]: %.3f\n", mean([results.translation]))
fprintf("---- Rotation error [deg]: %.3f\n", mean([results.geodesic]))
fprintf("---- ID ratio [%]: %.3f\n", sum([results.ID])/sum([results.num_scans]) * 100)

disp("-- median:")
fprintf("---- Number of scans: %.3f\n", median([results.num_scans]))
fprintf("---- Translation error [mm]: %.3f\n", median([results.translation]))
fprintf("---- Rotation error [deg]: %.3f\n", median([results.geodesic]))

disp("-- std:")
fprintf("---- Number of scans: %.3f\n", std([results.num_scans]))
fprintf("---- Translation error [mm]: %.3f\n", std([results.translation]))
fprintf("---- Rotation error [deg]: %.3f\n", std([results.geodesic]))




%% Package general analysis
if analyze_package
    %% summary
    disp("===============================================================")
    disp("============= Summary Results of General Analysis =============")
    disp("===============================================================")

    summary_t_hz = summarizeGeneralAnalysis(TextData, "hz");
    summary_t_mean = summarizeGeneralAnalysis(TextData, "mean");
    disp("-----------------------------------------------------------")
    fprintf("Average over all dadtasets with %i scans \n", ...
        summary_t_hz.total_scans)
    disp("-----------------------------------------------------------")

    disp("--- Computation time [Hz]-----")
    struct2table(summary_t_hz.computation_hz)
    
    disp("--- Computation time of each step [Hz]-----")
    note = "Note: Due to calling the timing function hundreds of times " + ...
        "to time each function in the pipeline, " + ...
        "the total computation time is slower than the above. \n";
    fprintf(note)
    struct2table(summary_t_hz.timing_hz)
    
    disp("--- Computation time of each step [ms]-----")
    struct2table(summary_t_mean.timing_mean)
    
    disp("--- Cluster removal in each step -----")
    struct2table(summary_t_hz.clusters)
    
    disp("----- Speed of double sum -----")
    struct2table(summary_t_mean.decoding_mean)

end


%% False positives
for i = 1:3
    data_num = i;


    switch data_num
        case 1 
            dataset = "cartographer/horizontal_lidar";
        case 2 
            dataset = "cartographer/vertical_lidar";
        case 3
            dataset = "H3D";
    end
    path_folder = opts.data_path + "public_datasets/" + dataset + "/stats.txt";
    delimiterIn = ',';
    headerlinesIn = 1;

    disp("===============================================================")
    fprintf("====== False Positives for %s dataset ======\n", ...
        dataset)
    disp("===============================================================")
    data.pose_raw_data = dlmread(path_folder, ',', 1, 0);
    data.num_scan = size(data.pose_raw_data, 1);
    false_positives_index = find(data.pose_raw_data(:, size(data.pose_raw_data, 2)));
    data.pose_raw_data(false_positives_index, :);
    data.false_positives = length(false_positives_index);
    data.false_positives_ratio = length(false_positives_index)/data.num_scan * 100 
end


%% Indoor
disp("Compuration time in a cluttered laboratory")
indoor_path = opts.data_path + "indoor/";
indoor = analyzeLiDARTagPackage([], indoor_path);
disp("--- Computation time [Hz]-----")
struct2table(indoor.computation_hz)