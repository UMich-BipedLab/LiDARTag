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
opts.img_path  = "tag16h6/"; % tag_img/
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
tag_number = 1;
opts.ell = lidartag_data_t.target_size/tag.num_bit/2;
aptiltag_data_t = loadAprilTagPointCloud(opts, tag_number);
inner_product_t = computeGeometryAndNormalInnerProduct(opts, lidartag_data_t, aptiltag_data_t);
fprintf("normal inner product: %3f\n", inner_product_t.inner_product)
fprintf("geometry inner product: %3f\n", inner_product_t.geometry_inner_product)

% Plottings
plotResults(opts, aptiltag_data_t.image_array, out_t.ideal_object_list, out_t.X, ...
            lidartag_data_t.X_clean, lidartag_data_t.pionts_ideal_frame, ...
            lidartag_data_t. projected_vertices,...
            aptiltag_data_t.geometry_img, inner_product_t.geometry_pc)

disp("All done")








% infernce_old = -1;
% for infernce = 0:num_tag
%     for target = 0:num_tag
% %         img_inference = imread(img_ary(target,:)); % Load LiDARTag
% % 
% %         img_dict = imread(img_ary(infernce,:)); % Load LiDARTag
%         img_dict = imread(img_path + sprintf("tag16_05_%05d.png", target)); % Load LiDARTag
%         img_inference = imread(img_path + sprintf("tag16_05_%05d.png", infernce)); % Load LiDARTag
%         fprintf("------\n -- Working on \n --- Target: %s \n --- Inference: %s \n------\n", ...
%                 sprintf("tag16_05_%05d.png", target), sprintf("tag16_05_%05d.png", infernce))
%         score_table.Properties.VariableNames(target+1) = "tag" + num2str(target);
%         score_table.Properties.RowNames(infernce+1) = "tag" + num2str(infernce);
%         
%         % Show Image in 3D
%         if plotting.plot_figure
%             figure(1)
%             gcf = figure(1);
%             set(gcf,'name',  sprintf("tag16_05_%05d.png", target));
%             img_x = -plotting.img_x: plotting.grid_size : plotting.img_x;
%             img_y = -plotting.img_y: plotting.grid_size : plotting.img_y;
%             image(img_x(:), img_y(:), img_dict, 'AlphaData', 0.5); 
%             hold on;
%         end
% 
% 
%         %% Construct Tag Image Gaussian Distribution
%         disp("Constructing tag image Gaussian distribution...")
%         sigma = [tag.grid_size/5 0; 0 tag.grid_size/5];
%         peak = 1;
%         [tag.img.distribution, tag.img.normalized_distribution] = genTagImageDistribution(img_dict, tag.grid_size, plotting, peak, sigma);
%         % max(tag.img.normalized_distribution, [], 'all')
%         % min(tag.img.normalized_distribution, [], 'all')
%         tag.img.distribution = tag.img.distribution + plotting.height;
%         tag.img.normalized_distribution = tag.img.normalized_distribution + plotting.height;
% 
%         % surf(plotting.plot_x, plotting.plot_y, tag.img.distribution, 'AlphaData', 0.1,'FaceAlpha', 0.5)
%         if plotting.plot_figure
%             surf(plotting.plot_x, plotting.plot_y, tag.img.normalized_distribution, 'AlphaData', 0.1,'FaceAlpha', 0.5)
% 
% 
%             % Figure Setting
%             % view(-30,40)
%             view(0,45)
%             view(90,70)
%             view(180,-90)
%             view(25,30)
%             % caxis([min(y(:))-0.5*range(y(:)),max(y(:))])
%             % axis([-3 3 -3 3 0 0.4])
%             % xlabel('x')
%             % ylabel('y')
%             % zlabel('Probability Density')
%             axis equal
%             fig = gcf;
%             set(fig, 'Color', 'None')
%             hold off
%         end
% 
% 
%         %% LiDAR points simulation with intensities
%         if infernce_old ~= infernce
%             disp("Constructing LiDARTag frame and xyz points...")
%             tag = genFrameCoordinate(tag); % LiDARTag frame
%             tag = genLiDARTagPoints(tag); % LiDARTag xyz-points
% 
%             % LiDAR Intensity
%             disp("Constructing intensity of LiDARTag points...")
%             vec1 = [0 -1 0];
%             vec2 = [0 0 -1];
%             corner = [0 -2 -2];
%             use_binary = 0;
%             tag.sim.noisy.points = genPointIntensityBasedOnImageTag(corner, vec1, vec2, img_inference, peak, tag.num_bit, tag.grid_size, tag.sim.noisy.points, use_binary);
% 
%             % Show results
%             if plotting.plot_figure
%                 disp('Drawing constructed results...')
%                 figure(2)
%                 gcf = figure(2);
%                 set(gcf,'name',  sprintf("tag16_05_%05d.png", infernce));
%                 ptCloud = pointCloud([tag.sim.noisy.points.x; tag.sim.noisy.points.y; tag.sim.noisy.points.z]', 'Intensity', tag.sim.noisy.points.normalized_intensity');
%                 pcshow(ptCloud)
%                 hold off
% 
%                 figure(3)
%                 gcf = figure(3);
%                 set(gcf,'name',  sprintf("tag16_05_%05d.png", infernce));
%                 for i = 1:size(tag.sim.noisy.points.x, 2)
%                     scatter3(tag.sim.noisy.points.x(i), tag.sim.noisy.points.y(i), tag.sim.noisy.points.z(i), 'or', 'MarkerFaceColor', 'k', 'MarkerFaceAlpha', tag.sim.noisy.points.plotting.normalized_intensity(i))
%                     hold on
%                 end
%                 scatter3(tag.sim.noise_free.points.x, tag.sim.noise_free.points.y, tag.sim.noise_free.points.z, '.b')
%                 scatter3(tag.scatter3.x, tag.scatter3.y, tag.scatter3.z, 'ok')
%                 plot3(reshape(tag.plot3.x,2,[]),reshape(tag.plot3.y,2,[]), reshape(tag.plot3.z,2,[]), 'k')
%                 axis equal
%                 xlabel('x')
%                 ylabel('y')
%                 zlabel('z')
%                 hold off
%             end
% 
%             %% Create LiDAR point distributions
%             disp('Constructing LiDARTag point distributions...')
%             tag.sim.noisy.points = genLiDARPointsDistribution(plotting, peak, sigma, tag.sim.noisy.points);
% 
%             % Show results
%             if plotting.plot_figure
%                 disp('Drawing constructed results...')
%                 figure(4)
%                 gcf = figure(4);
%                 set(gcf,'name',  sprintf("tag16_05_%05d.png", infernce));
%                 image(img_x(:), img_y(:), img_inference, 'AlphaData', 0.5); 
%                 hold on
%                 surf(plotting.plot_x, plotting.plot_y, tag.sim.noisy.points.normalized_distribution, 'AlphaData', 0.1,'FaceAlpha', 0.5)
%                 axis equal
%                 xlabel('x')
%                 ylabel('y')
%                 zlabel('z')
%                 hold off
% 
%                 figure(5)
%                 gcf = figure(5);
%                 set(gcf,'name',  sprintf("tag16_05_%05d.png", infernce));
%                 for i = 1:size(tag.sim.noisy.points.x, 2)
%                     scatter3(tag.sim.noisy.points.x(i), tag.sim.noisy.points.y(i), tag.sim.noisy.points.z(i), 'or', 'MarkerFaceColor', 'r', 'MarkerFaceAlpha', tag.sim.noisy.points.plotting.normalized_intensity(i))
%                     hold on
%                 end
%                 scatter3(tag.sim.noise_free.points.x, tag.sim.noise_free.points.y, tag.sim.noise_free.points.z, '.b')
%                 scatter3(tag.scatter3.x, tag.scatter3.y, tag.scatter3.z, 'ok')
%                 plot3(reshape(tag.plot3.x,2,[]),reshape(tag.plot3.y,2,[]), reshape(tag.plot3.z,2,[]), 'k')
%                 surf(tag.sim.noisy.points.normalized_distribution, plotting.plot_x, plotting.plot_y, 'AlphaData', 0.1,'FaceAlpha', 0.5)
%                 axis equal
%                 xlabel('x')
%                 ylabel('y')
%                 zlabel('z')
%                 hold off
%                 disp('Done plotting')
%                 drawnow
%             end
%             infernce_old = infernce;
%         end
%         %% Compare similarity
%         % load examgrades
%         % test1 = grades(:,1);
%         % x = (test1-75)/10;
%         % h = kstest(x)
%         % cdfplot(x)
%         % hold on
%         % x_values = linspace(min(x),max(x));
%         % plot(x_values,normcdf(x_values,0,1),'r-')
%         % legend('Empirical CDF','Standard Normal CDF','Location','best')
%         %%
%         disp("Comparing distribution ...")
%         fprintf(" --- Target: %s \n --- Inference: %s \n", ...
%                 sprintf("tag16_05_%05d.png", target), sprintf("tag16_05_%05d.png", infernce))
%         intensity_matrix = intensityToMatrix(tag.sim.noisy.points.x, ...
%                                              tag.sim.noisy.points.y, ...
%                                              tag.sim.noisy.points.z, ...
%                                              tag.sim.noisy.points.intensity);
% 
%         intensity_matrix2 = intensityToMatrix(plotting.plot_x, plotting.plot_y, ...
%                                               zeros(1, size(tag.sim.noisy.points.z, 2)), ...
%                                               tag.img.normalized_distribution);
%         score = computeDistributionCost(intensity_matrix, intensity_matrix2);
%         score_table(target+1, infernce+1) = {score};
%         if plotting.pause 
% %             tag.img.normalized_distribution
%             disp('Pausing ...')
%             pause
%         end
%     end
% end
% %%
% disp("--- Results")
% score_table
% [~,max_tag] = max(table2array(score_table, [], 2), [], 1);
% max_tag = max_tag -1