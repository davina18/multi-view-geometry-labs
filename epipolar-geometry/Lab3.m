% This a empty script to help you move faster on you lab work.
clear all;
close all;
clc;


%% Step 1
% Camera 1
au1 = 100; av1 = 120; uo1 = 128; vo1 = 128;
imageSize = [256 256];

%% Step 2
% Camera 2
au2 = 90; av2 = 110; uo2 = 128; vo2 = 128; 
ax = 0.1; by = pi/4; cz = 0.2; % XYZ EULER 
tx = -1000; ty = 190; tz = 230; 

%% STEP 3
% Compute intrinsic matrices and projection matrices

K1 = [au1 0 uo1; 0 av1 vo1; 0 0 1]; % Intrisics matrix for camera 1
wR1c = eye(3);   % rotation of camera 1, from the camera to the world coordinate frame
wt1c = [0 0 0]'; % translation of camera 1, from the camera to the world coordinate frame

% Note: ************** You have to add your own code from here onward ************
K2 = [au2 0 uo2; 0 av2 vo2; 0 0 1]; 
rot_x = [1 0 0; 0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)];
rot_y = [cos(by) 0 sin(by); 0 1 0; -sin(by) 0 cos(by)];
rot_z = [cos(cz) -sin(cz) 0; sin(cz) cos(cz) 0; 0 0 1];
wR2c = rot_x * rot_y * rot_z
wt2c = [tx ty tz]';


P1 = K1 * [wR1c' -wR1c' * wt1c];

P2 = K2 * [wR2c' -wR2c' * wt2c];


%% STEP 4
% Attention: This is an invented matrix just to have some input for the drawing
% functions. You have to compute it properly 
%F = [3e-05 7e-05 -0.006;2e-05 -2.e-05 0.01;-0.009 -0.01 1];
brack_tx = [0 -tz ty; tz 0 -tx; -ty tx 0];
F = inv(K2)'  * wR2c' * brack_tx * inv(K1);

F = F./F(3,3);
fprintf('Step 4:\n\tAnalytically obtained F:\n');
disp(F);

%% STEP 5
V(:,1) = [100;-400;2000];
V(:,2) = [300;-400;3000];
V(:,3) = [500;-400;4000];
V(:,4) = [700;-400;2000];
V(:,5) = [900;-400;3000];
V(:,6) = [100;-40;4000];
V(:,7) = [300;-40;2000];
V(:,8) = [500;-40;3000];
V(:,9) = [700;-40;4000];
V(:,10) = [900;-40;2000];
V(:,11) = [100;40;3000];
V(:,12) = [300;40;4000];
V(:,13) = [500;40;2000];
V(:,14) = [700;40;3000];
V(:,15) = [900;40;4000];
V(:,16) = [100;400;2000];
V(:,17) = [300;400;3000];
V(:,18) = [500;400;4000];
V(:,19) = [700;400;2000];
V(:,20) = [900;400;3000];

%% STEP 6
% Projection on image planes
cam1_p2d = mvg_projectPointToImagePlane(V,P1);
cam2_p2d = mvg_projectPointToImagePlane(V,P2);

%% STEP 7
% example of the plotting functions 
% Draw 2D projections on image planes
cam1_fig = mvg_show_projected_points(cam1_p2d(1:2,:),imageSize,'Projected points on image plane 1');
cam2_fig = mvg_show_projected_points(cam2_p2d(1:2,:),imageSize,'Projected points on image plane 2');

% Draw epipolar lines
[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(cam1_p2d,cam2_p2d,F);
[cam1_fig,cam2_fig] = mvg_show_epipolar_lines(cam1_fig, cam2_fig, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');

% Draw epipoles
%ep_1 = [-300 200 1];    % These are invented values just for illustrating. You have to compute them
%ep_2 = [200 50 1];      % These are invented values just for illustrating. You have to compute them
[U,S,V] = svd(F);
ep_1 = V(:, end); % Last column of V
ep_1 = ep_1 / ep_1(3); % Normalize to make cartesian
ep_2 = U(:, end); % Last column of U
ep_2 = ep_2 / ep_2(3); % Normalise to make cartesian
[~,~] = mvg_show_epipoles(cam1_fig, cam2_fig, ep_1, ep_2);

return;

[U,S,V] = svd(F);
S(3,3) = 0;
F_rank2 = U*S*V';

null(F_rank2)