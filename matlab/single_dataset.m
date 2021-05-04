clc, clear
data_num = 4;
path = "./paper_data_pre/lidartag_estimates/";



switch data_num
    case 1 
        dataset = "straight-ccw1-3";
    case 2 
        dataset = "lab3";
    case 3
        dataset = "ccw1-3";
    case 4
        dataset = "new-lab2";
    case 5
        path = "./paper_data/public_datasets/";
        dataset = "cartographer/horizontal_lidar";
    case 6
        path = "./paper_data/public_datasets/";
        dataset = "cartographer/vertical_lidar";
    case 7
        path = "./paper_data/public_datasets/";
        dataset = "H3D";
    case 8
        dataset = "lab-small-middle";
    case 9
        dataset = "Oct01-2020/ccw1-5";
%         dataset = "Oct07-2020/ccw1-8";
end
path_folder = path + dataset + "/";
delimiterIn = ',';
headerlinesIn = 1;


lidartag = analyzeLiDARTagPackage([], path_folder);

disp("===============================================================")
disp("============= Summary Results of General Analysis =============")
disp("===============================================================")
% summary_t = summarizeGeneralAnalysis(lidartag);

% disp("-----------------------------------------------------------")
% fprintf("Average over all dadtasets with %i scans \n", ...
%     summary_t.total_scans)
% disp("-----------------------------------------------------------")

% if ~isempty(lidartag.timing_hz)
%     t1 = struct2table(lidartag.timing_hz)
% end
% 

% if ~isempty(lidartag.computation_mean)
%     
%     t4 = struct2table(lidartag.computation_mean)
% end

if ~isempty(lidartag.computation_hz)
    disp("---------- Computation time in hz ----------")
    struct2table(lidartag.computation_hz)
end


if ~isempty(lidartag.timing_mean)
    disp("---------- Timine Each step in ms ----------")
    struct2table(lidartag.timing_mean)
end


if ~isempty(lidartag.clusters)
    disp("---------- Remaining Clusters ----------")
    struct2table(lidartag.clusters)
end

if ~isempty(lidartag.decoding_mean)
    disp("---------- Speed of Decoding ----------")
    struct2table(lidartag.decoding_mean)
end