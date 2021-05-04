clc, clear

path = "./paper_data/public_datasets/";
data_num = 3;


switch data_num
    case 1 
        dataset = "cartographer/horizontal_lidar";
    case 2 
        dataset = "cartographer/vertical_lidar";
    case 3
        dataset = "H3D";
end
path_folder = path + dataset + "/stats.txt";
delimiterIn = ',';
headerlinesIn = 1;



disp("===============================================================")
fprintf("=============== False Positives for %s dataset ===============\n", ...
    dataset)
disp("===============================================================")
data.pose_raw_data = dlmread(path_folder, ',', 1, 0);
data.num_scan = size(data.pose_raw_data, 1);
false_positives_index = find(data.pose_raw_data(:, size(data.pose_raw_data, 2)));
data.pose_raw_data(false_positives_index, :);
data.false_positives = length(false_positives_index);
data.false_positives_ratio = length(false_positives_index)/data.num_scan * 100 