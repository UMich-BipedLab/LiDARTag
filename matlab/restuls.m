clc, clear
data_num = 3;
path = "./paper_data_pre/lidartag_estimates/";

switch data_num
    case 1 
        dataset = "straight-ccw1-3";
    case 2 
        dataset = "lab3";
    case 3
        dataset = "ccw1-3";
end
path_folder = path + dataset + "/";
delimiterIn = ',';
headerlinesIn = 1;


%% Computation time
name = "timing_computation_only.txt";
filename = path_folder +name;
data = importdata(filename, delimiterIn, headerlinesIn);

for k = 1:size(data.data, 2)
%    timing.(genvarname(data.colheaders{1, k})) = data.data(:, k);
   computation_mean.(genvarname(string(data.colheaders{1, k}) + "_mean")) = mean(data.data(:, k));
   computation_hz.(genvarname(string(data.colheaders{1, k}) + "_Hz")) = 1000/mean(data.data(:, k));
end
% struct2table(computation_mean)


%% Decoding
name = "decoding_analysis.txt";
filename = path_folder + name;
data = importdata(filename, delimiterIn, headerlinesIn);

for k = 1:size(data.data, 2)
   decoding_hz.(genvarname(string(data.colheaders{1, k}) + "_Hz")) = 1e3/mean(data.data(:, k));
end

%% Timings
% name = "computation_time.txt";
name = "timing_all.txt";
filename = path_folder +name;
data = importdata(filename, delimiterIn, headerlinesIn);

for k = 1:size(data.data, 2)
%    timing.(genvarname(data.colheaders{1, k})) = data.data(:, k);
   timing_mean.(genvarname(string(data.colheaders{1, k}) + "_mean")) = mean(data.data(:, k));
   timing_hz.(genvarname(string(data.colheaders{1, k}) + "_Hz")) = 1000/mean(data.data(:, k));
end
% struct2table(timing_mean)



%% Clusters
% name = "computation_time.txt";
name = "stats.txt";
filename = path_folder + name;
data = importdata(filename, delimiterIn, headerlinesIn);

for k = 1:size(data.data, 2)
%    records.(genvarname(data.colheaders{1, k})) = data.data(:, k);
   clusters.(genvarname(string(data.colheaders{1, k}) + "_mean")) = mean(data.data(:, k));
end


fprintf("Each of below sums over %i scans from %s dataset: \n", size(data.data, 1), dataset)
struct2table(computation_hz)
struct2table(decoding_hz)
struct2table(timing_hz)
struct2table(clusters)



disp("All done")