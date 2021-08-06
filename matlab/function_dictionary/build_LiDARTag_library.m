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
addpath(genpath("/home/shigeki/tag_ws/src/matlab_utils"));


poolobj = gcp('nocreate');
% delete(poolobj)
if isempty(poolobj)
    parpool('threads');
end

opts.verbose.output_level = 1;
opts.num_tag = 9; %5, 9, 30 % max is 30 for tag1606
opts.img_path  = "tag16h5/"; % tag_img/
opts.img_prefix = "tag16_05_%05d.png";

%% Tag Family Setting
tag = loadAprilTagFamily();



%% ideal frame
opts.target_size = 1.03;
opts.polygon = 4;
opts.rpy = [0, 0, 0];
opts.xyz = [0, 0, 0];
[ideal_object_list, ~] = createDynamicScene(opts);
LiDARTag.ideal_frame = convertXYZstructToXYZmatrix(ideal_object_list.object_vertices);
LiDARTag.ideal_object_list = ideal_object_list;



opts.save_path = "./data_test/" + opts.img_path + num2str(opts.target_size) + "/";
opts.load_path = "./data_test/" + opts.img_path + num2str(opts.target_size) + "/";
checkDirectory(opts.save_path);


%% AprilTag   
score_table_t(opts.num_tag) = struct();
opts.ell = opts.target_size/tag.num_bit/2;


for img = 1:opts.num_tag
    aptiltag_data_t = loadAprilTagPointCloud(opts, img);
    score_table_t(img).image_name = aptiltag_data_t.image_name;
    score_table_t(img).image_array = aptiltag_data_t.image_array;
    score_table_t(img).geometry_img = aptiltag_data_t.geometry_img;
end


%% Plottings
tag_num = 5;
image_name = score_table_t(tag_num).image_name;
image_array = score_table_t(tag_num).image_array;
geometry_img = score_table_t(tag_num).geometry_img;


% Plottings
[axes_h, ~] = createFigHandleWithNumber(5, 1, "overlayAprilTag", 1, 1);
cur_axes = axes_h(1);
AprilTag_map = genColorMap(image_array(4,:), 10, 1);
scatter3(cur_axes, image_array(1,:), image_array(2,:), image_array(3,:), [], AprilTag_map, 'fill')
plotObjectsList(cur_axes, opts, [], LiDARTag.ideal_object_list)
plotOriginalAxisWithText(cur_axes, "LiDAR")
viewCurrentPlot(cur_axes, "Template with AprilTag", [-60 20], 1)
reloadCurrentPlot(cur_axes, 1)


cur_axes = axes_h(2);
scatter3(cur_axes, geometry_img(1,:), geometry_img(2,:), geometry_img(3,:), [], AprilTag_map, 'fill')
plotObjectsList(cur_axes, opts, [], LiDARTag.ideal_object_list)
plotOriginalAxisWithText(cur_axes, "LiDAR")
viewCurrentPlot(cur_axes, "Template with Geometry AprilTag", [-60 20], 1)
reloadCurrentPlot(cur_axes, 1)
disp("Library built")
