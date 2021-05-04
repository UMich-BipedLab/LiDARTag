clear, clc
addpath(genpath("/home/brucebot/workspace/matlab_utils"));
addpath("/home/brucebot/workspace/lc-calibration/L1_relaxation")
loadLibraries(2);

root_path = "./paper_data/";


angles = rad2deg(readmatrix(root_path + "angles_ours.txt"));

%%
figure(3)
cla(3)
hold on 
scatter(angles(1,:), angles(1,:), '.')
scatter(angles(2,:), angles(2,:), '*')
scatter(angles(3,:), angles(3,:), 'o')
scatter(angles(4,:), angles(4,:), 'v')
% for i = 1:size(angles, 1)
%     scatter(angles(i,:), angles(i,:), '.')
% end