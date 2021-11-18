%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

clc, clear

% Tag Family Setting
tag = loadAprilTagFamily();

addpath(genpath("/home/brucebot/workspace/matlab_utils"));
addpath("/home/brucebot/workspace/lc-calibration/L1_relaxation")
loadLibraries(2);


poolobj = gcp('nocreate');
% delete(poolobj)
if isempty(poolobj)
    parpool('max-threading', 4);
end
opts.verbose.output_level = 1;
opts.num_tag = 5; % max is 30
opts.img_path  = "tag16h5/"; % tag_img/
opts.img_prefix = "tag16_06_%05d.png";


%% Load lidartag data
dataset = 1;
[out_t, opts, opt] = loadData(opts, dataset);
lidartag_data_t = transformToTemplate(opt, opts, out_t.X);

target_size = lidartag_data_t.target_size;
X_clean = lidartag_data_t.X_clean;
pionts_ideal_frame = lidartag_data_t.pionts_ideal_frame;
projected_vertices = lidartag_data_t.projected_vertices;


opts.save_path = "./data/" + opts.img_path + "/" + num2str(opts.target_size) + "/";
opts.load_path = "./data/" + opts.img_path + "/" + num2str(opts.target_size) + "/";
checkDirectory(opts.save_path);

%% AprilTag   
clc
score_table_t(opts.num_tag) = struct();
opts.ell = lidartag_data_t.target_size/tag.num_bit/2;
parforProgress(opts.num_tag);
parfor img = 1:opts.num_tag
    parforProgress;

    aptiltag_data_t = loadAprilTagPointCloud(opts, img);
    inner_product_t = computeGeometryAndNormalInnerProduct(opts, lidartag_data_t, aptiltag_data_t);

    score_table_t(img).image_name = aptiltag_data_t.image_name;
    score_table_t(img).image_array = aptiltag_data_t.image_array;
    score_table_t(img).geometry_img = aptiltag_data_t.geometry_img;
    score_table_t(img).geometry_pc = inner_product_t.geometry_pc;
    score_table_t(img).inner_product = inner_product_t.inner_product;
    score_table_t(img).geometry_inner_product = inner_product_t.geometry_inner_product;
end
parforProgress(0);


inner_product_vec = [score_table_t(:).inner_product]
fprintf("\n\n")
geo_inner_product_vec = [score_table_t(:).geometry_inner_product]



fprintf("inner_product\n")
[value, indx] = max(inner_product_vec)

fprintf("geometry_inner_product\n")
[geo_value, geo_indx] = max(geo_inner_product_vec)


fprintf("image name (normal): %s\n", score_table_t(indx).image_name)
fprintf("image name (geom): %s\n", score_table_t(geo_indx).image_name)
%% Plottings
tag_num = geo_indx;
image_name = score_table_t(tag_num).image_name;
image_array = score_table_t(tag_num).image_array;
geometry_img = score_table_t(tag_num).geometry_img;
geometry_pc = score_table_t(tag_num).geometry_pc;


% Plottings
plotResults(opts, image_array, out_t.ideal_object_list, out_t.X, ...
            lidartag_data_t.X_clean, lidartag_data_t.pionts_ideal_frame, ...
            lidartag_data_t.projected_vertices,...
            geometry_img, geometry_pc)
