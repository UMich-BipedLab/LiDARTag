function out_t = analyzeLiDARTagPackage(input_t, path_folder)
    delimiterIn = ',';
    headerlinesIn = 1;
    
    if isempty(input_t)
        index = strfind(path_folder, '/');
        char_name = char(path_folder);
        input_t.name = char_name(index(end-1)+1:index(end)-1);
    end
    out_t = input_t;
    out_t.num_data = [];
    out_t.computation_mean = [];
    out_t.timing_mean = [];
    out_t.computation_hz = [];
    out_t.timing_hz = [];
    out_t.clusters = [];
    
    %% Computation time
    name = "timing_computation_only.txt";
    filename = path_folder + name;

    data = importdata(filename, delimiterIn, headerlinesIn);

    % start from 2 to skip the first column
    for k = 2:size(data.data, 2)
    %    timing.(genvarname(data.colheaders{1, k})) = data.data(:, k);
       computation_mean.(genvarname(string(data.colheaders{1, k}) + ...
           "_mean")) = mean(data.data(:, k));
       computation_hz.(genvarname(string(data.colheaders{1, k}) + ...
           "_Hz")) = 1000/mean(data.data(:, k));
    end
    out_t.computation_mean = computation_mean;

    % struct2table(computation_mean)


    %% Timings
    % name = "computation_time.txt";
    name = "timing_all.txt";
    filename = path_folder + name;
    if ~isfile(filename)
%         warning("No such file: %s", filename)
        timing_mean = [];
        timing_hz = [];
    else
        data = importdata(filename, delimiterIn, headerlinesIn);

        for k = 2:size(data.data, 2)
        %    timing.(genvarname(data.colheaders{1, k})) = data.data(:, k);
           timing_mean.(genvarname(string(data.colheaders{1, k}) + ...
               "_mean")) = mean(data.data(:, k));
           timing_hz.(genvarname(string(data.colheaders{1, k}) + ...
               "_Hz")) = 1000/mean(data.data(:, k));
        end
    end

    % struct2table(timing_mean)



    %% Clusters
    % name = "computation_time.txt";
    name = "stats.txt";
    filename = path_folder + name;
    if ~isfile(filename)
        error("No such file: %s", filename)
    end
    data = importdata(filename, delimiterIn, headerlinesIn);

    for k = 2:size(data.data, 2)
    %    records.(genvarname(data.colheaders{1, k})) = data.data(:, k);
       clusters.(genvarname(string(data.colheaders{1, k}) + ...
           "_mean")) = mean(data.data(:, k));
    end
    
    
    %% preparing for output_t
    out_t.num_data = size(data.data, 1);
    out_t.computation_mean = computation_mean;
    out_t.computation_hz = computation_hz;
    
    out_t.timing_mean = timing_mean;
    out_t.timing_hz = timing_hz;
    
    out_t.decoding_mean = [];
    out_t.decoding_hz = [];
       
    out_t.clusters = clusters;
    
    %% Decoding
    name = "decoding_analysis.txt";
    filename = path_folder + name;
    if isfile(filename)
        data = importdata(filename, delimiterIn, headerlinesIn);
        for k = 2:size(data.data, 2)
           decoding_hz.(genvarname(string(data.colheaders{1, k}) + ...
               "_Hz")) = 1e3/mean(data.data(:, k));
           decoding_mean.(genvarname(string(data.colheaders{1, k}) + ...
               "_mean")) = mean(data.data(:, k));
        end  
        out_t.decoding_hz = decoding_hz;
        out_t.decoding_mean = decoding_mean;
    end

end