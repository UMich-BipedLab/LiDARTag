clc
addpath(genpath("/home/brucebot/workspace/matlab_utils"));
loadLibraries(2);


lie = 1;

if lie == 1
    r = sym('r%d%d',[1 3]);
    R = Exp_SO3(r);
    % rpy = sym('rpy%d%d',[1 3]);
    % R = rotx(rpy(1)) * roty(rpy(2)) * rotz(rpy(3));
    T = sym('t%d%d', [3 1]);
    H = [R T; 0 0 0 1];

    % 3D pionts
    num_points = 2;
    X = sym('X%d', [1, num_points]);
    Y = sym('Y%d', [1, num_points]);
    Z = sym('Z%d', [1, num_points]);
    points = [X;Y;Z;ones(1, num_points)];
    points_trans = simplify(H * points);

    %
    H_vec = [r, transpose(T)];


    d_px = simplify(jacobian(points_trans(1), H_vec));
    d_py = simplify(jacobian(points_trans(2), H_vec));
    d_pz = simplify(jacobian(points_trans(3), H_vec));

    disp("saving d_px")
    matlabFunction(d_px,'File','d_px_lie');

    disp("saving d_py")
    matlabFunction(d_py,'File','d_py_lie');

    disp("saving d_pz")
    matlabFunction(d_pz,'File','d_pz_lie');
elseif lie == 0
    rpy = sym('rpy%d%d',[1 3]);
    R = rotx_sym(rpy(1)) * roty_sym(rpy(2)) * rotz_sym(rpy(3));
    T = sym('t%d%d', [3 1]);
    H = [R T; 0 0 0 1];

    % 3D pionts
    num_points = 1;
    X = sym('X%d', [1, num_points]);
    Y = sym('Y%d', [1, num_points]);
    Z = sym('Z%d', [1, num_points]);
    points = [X;Y;Z;ones(1, num_points)];
%     test = H * points;
    points_trans = simplify(H * points);
    
    H_vec = [rpy, transpose(T)];

    d_px_euler = simplify(jacobian(points_trans(1, :), H_vec));
    d_py_euler = simplify(jacobian(points_trans(2, :), H_vec));
    d_pz_euler = simplify(jacobian(points_trans(3, :), H_vec));

%     disp("saving d_px")
%     matlabFunction(d_px_euler,'File','d_px_euler');
% 
%     disp("saving d_py")
%     matlabFunction(d_py_euler,'File','d_py_euler');
% 
%     disp("saving d_pz")
%     matlabFunction(d_pz_euler,'File','d_pz_euler');
end


disp("Done")




