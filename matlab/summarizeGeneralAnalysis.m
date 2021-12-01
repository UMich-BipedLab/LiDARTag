function summary_t = summarizeGeneralAnalysis(lidartag, suffix)
    counter_general = 0;
    counter_decoding = 0;
    summary_t = struct('total_scans', 0);
    for i = 1:size(lidartag, 2)
        dataset_num = i;
        if isempty(lidartag(dataset_num).num_data)
            warning("No results for %s", lidartag(dataset_num).name)
        else
            summary_t.total_scans = ...
                summary_t.total_scans + lidartag(dataset_num).num_data;
            if ~exist('sum_computation_mat', 'var')
                sum_computation_mat = zeros(numel(fieldnames(...
                    lidartag(dataset_num).("computation_" + suffix))), 1);
            end
            
            if ~exist('sum_timing_mat', 'var')
                sum_timing_mat = zeros(numel(fieldnames(...
                    lidartag(dataset_num).("timing_" + suffix))), 1);
            end
            
            if ~exist('sum_clusters_mat', 'var')
                sum_clusters_mat = zeros(numel(fieldnames(...
                    lidartag(dataset_num).clusters)), 1);
            end
            

            sum_computation_mat = ...
                    sum_computation_mat + ...
                    cell2mat(struct2cell(...
                    lidartag(dataset_num).("computation_" + suffix)));
            if ~isfield('computation_hz', summary_t)
                summary_t.("computation_" + suffix) = ...
                    lidartag(dataset_num).("computation_" + suffix);
            end

            sum_timing_mat = ...
                    sum_timing_mat + ...
                    cell2mat(struct2cell(...
                    lidartag(dataset_num).("timing_" + suffix)));  
            if ~isfield('timing_hz', summary_t)
                summary_t.("timing_" + suffix) = ...
                    lidartag(dataset_num).("timing_" + suffix);
            end
            
            sum_clusters_mat = ...
                    sum_clusters_mat + ...
                    cell2mat(struct2cell(...
                    lidartag(dataset_num).clusters)); 
            if ~isfield('clusters', summary_t)
                summary_t.clusters = ...
                    lidartag(dataset_num).clusters;
            end
            
            counter_general = counter_general + 1;
            
            if isempty(lidartag(dataset_num).("decoding_" + suffix))
%                  warning("No decoding_hz for %s dataset", ...
%                      lidartag(dataset_num).name)
            else
                if ~exist('sum_decoding_mat', 'var')
                    sum_decoding_mat = zeros(numel(fieldnames(...
                        lidartag(dataset_num).("decoding_" + suffix))), 1);
                end
                sum_decoding_mat = ...
                    sum_decoding_mat + ...
                    cell2mat(struct2cell(...
                    lidartag(dataset_num).("decoding_" + suffix))); 
                
                if ~isfield('decoding_hz', summary_t)
                    summary_t.("decoding_" + suffix) = ...
                        lidartag(dataset_num).("decoding_" + suffix);
                end
                counter_decoding = counter_decoding + 1;
            end
        end
    end
    ave_computation = sum_computation_mat ./ counter_general;
    ave_timing = sum_timing_mat ./ counter_general;
    ave_clusters = sum_clusters_mat ./ counter_general;
    
    if exist('sum_decoding_mat', 'var')
        ave_decoding = sum_decoding_mat ./ counter_decoding;
    end
    

    
    computation_field_names = fieldnames(summary_t.("computation_" + suffix));
    assert(length(ave_computation)==length(computation_field_names));
    for i = 1:length(ave_computation)
        summary_t.("computation_" + suffix).(string(computation_field_names(i))) = ...
            ave_computation(i);
    end
    
    timing_field_names = fieldnames(summary_t.("timing_" + suffix));
    assert(length(ave_timing)==length(timing_field_names));
    for i = 1:length(ave_timing)
        summary_t.("timing_" + suffix).(string(timing_field_names(i))) = ...
            ave_timing(i);
    end
    
    cluster_field_names = fieldnames(summary_t.clusters);
    assert(length(ave_clusters)==length(cluster_field_names));
    for i = 1:length(ave_clusters)
        summary_t.clusters.(string(cluster_field_names(i))) = ...
            ave_clusters(i);
    end
    
    if isfield('decoding_hz', summary_t)
        decoding_field_names = fieldnames(summary_t.("decoding_" + suffix));
        assert(length(ave_decoding)==length(decoding_field_names));
        for i = 1:length(ave_decoding)
            summary_t.("decoding_" + suffix).(string(decoding_field_names(i))) = ...
                ave_decoding(i);
        end
    end
end















