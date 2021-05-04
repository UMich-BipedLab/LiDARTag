function [r_mean, t_mean] = ...
    computeMeanOfTranslationNRotation(rot_v, trans_v)
    % rot_v: n x 9
    % trans_v: n x 3
    % rot_v is in the oder of r11, r12, r13, r21, r22, r23, r31, r32, r33 
    
    t_mean = mean(trans_v, 1);
    num_rotation = size(rot_v, 1);
    
    ave = zeros(3, 1);
    for i = 1:num_rotation
        rotm = reshape(rot_v(i, :), [], 3)';
        ave = ave + Log_SO3(rotm);
    end
    ave = ave / num_rotation;
    r_mean = Exp_SO3(ave);
end