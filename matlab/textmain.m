clear, clc
% addpath(genpath("E:/2020summer_research/matlab/matlab_utils-master"));
% addpath("/home/brucebot/workspace/lc-calibration/L1_relaxation")
loadLibraries(2);

analyze_package = 1; % analyze each step of the pipeline
% event = 4; % which date
dataset = 2; % 1: front 2: ccw
tag_size = 1.22;
TextData = getTextData();


    
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
fprintf("---- ID ratio: %.3f\n", sum([results.ID])/sum([results.num_scans]) * 100)
fprintf("---- Rotation Number ratio: %.3f\n", sum([results.rot_num])/sum([results.num_scans]) * 100)
disp("-- median:")
fprintf("---- Number of scans: %.3f\n", median([results.num_scans]))
fprintf("---- Translation error [mm]: %.3f\n", median([results.translation]))
fprintf("---- Rotation error [deg]: %.3f\n", median([results.geodesic]))
fprintf("---- ID ratio: %.3f\n", sum([results.ID])/sum([results.num_scans]) * 100)
fprintf("---- Rotation Number ratio: %.3f\n", sum([results.rot_num])/sum([results.num_scans]) * 100)
disp("-- std:")
fprintf("---- Number of scans: %.3f\n", std([results.num_scans]))
fprintf("---- Translation error [mm]: %.3f\n", std([results.translation]))
fprintf("---- Rotation error [deg]: %.3f\n", std([results.geodesic]))
fprintf("---- ID ratio: %.3f\n", sum([results.ID])/sum([results.num_scans]) * 100)
fprintf("---- Rotation Number ratio: %.3f\n", sum([results.rot_num])/sum([results.num_scans]) * 100)



%% Package general analysis
if analyze_package
    disp("===============================================================")
    disp("================= Results of General Analysis =================")
    disp("===============================================================")
    for i = (1 + num_estimates * (dataset - 1)) : ...
        (num_estimates + num_estimates * (dataset-1))
        dataset_num = i;
        if isempty(TextData(dataset_num).num_data)
            warning("No results for %s", TextData(dataset_num).name)
        else
            disp("-------------------------------------------------------")
            fprintf("Each of below average over %i scans ", ...
                TextData(dataset_num).num_data)
            fprintf("from %s dataset: \n", TextData(dataset_num).name)
            disp("-------------------------------------------------------")
            struct2table(TextData(dataset_num).computation_hz)
            struct2table(TextData(dataset_num).timing_hz)
            struct2table(TextData(dataset_num).clusters)

            if ~isfield(TextData(dataset_num), ...
                'decoding_hz') || ...
                isempty(TextData(dataset_num).decoding_hz)
                 warning("No decoding_hz for %s dataset", ...
                     TextData(dataset_num).name)
            else
                struct2table(TextData(dataset_num).decoding_hz)
            end
        end
    end

    %% summary
    disp("===============================================================")
    disp("============= Summary Results of General Analysis =============")
    disp("===============================================================")

    summary_t = summarizeGeneralAnalysis(TextData, "hz");
    disp("-----------------------------------------------------------")
    fprintf("Average Hz over all dadtasets with %i scans \n", ...
        summary_t.total_scans)
    disp("-----------------------------------------------------------")

    t1 = struct2table(summary_t.computation_hz)
    t2 = struct2table(summary_t.timing_hz)
    t3 = struct2table(summary_t.clusters)
    
    if isfield('decoding_hz', summary_t)
        t4 = struct2table(summary_t.decoding_hz)
    end


%     disp("-----------------------------------------------------------")
%     fprintf("Average ms over all dadtasets with %i scans \n", ...
%         summary_t.total_scans)
%     disp("-----------------------------------------------------------")
%     summary_t = summarizeGeneralAnalysis(TextData, "mean");
%     t1 = struct2table(summary_t.computation_mean)
%     t2 = struct2table(summary_t.timing_mean)
%     t3 = struct2table(summary_t.clusters)
%     if isfield('decoding_mean', summary_t)
%         t4 = struct2table(summary_t.decoding_mean)
%     end
end