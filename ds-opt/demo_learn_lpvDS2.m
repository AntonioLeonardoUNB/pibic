%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo Script for GMM-based LPV-DS Learning introduced in paper:          %
%  'A Physically-Consistent Bayesian Non-Parametric Mixture Model for     %
%   Dynamical System Learning.'; N. Figueroa and A. Billard; CoRL 2018    %
% With this script you can load 2D toy trajectories or even real-world    %
% trajectories acquired via kinesthetic taching and test the different    %
% GMM fitting approaches.                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2018 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Nadia Figueroa                                                 % 
% email:   nadia.figueroafernandez@epfl.ch                                %
% website: http://lasa.epfl.ch                                            %
%                                                                         %
% This work was supported by the EU project Cogimon H2020-ICT-23-2014.    %
%                                                                         %
% Permission is granted to copy, distribute, and/or modify this program   %
% under the terms of the GNU General Public License, version 2 or any     %
% later version published by the Free Software Foundation.                %
%                                                                         %
% This program is distributed in the hope that it will be useful, but     %
% WITHOUT ANY WARRANTY; without even the implied warranty of              %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General%
% Public License for more details                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 1 - OPTION 1 (DATA LOADING): Load CORL-paper Datasets %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc
%%%%%%%%%%%%%%%%%%%%%%%%% Select a Dataset %%%%%%%%%%%%%%%%%%%%%%%%%%
% 1:  Messy Snake Dataset   (2D) *
% 2:  L-shape Dataset       (2D) *
% 3:  A-shape Dataset       (2D) * 
% 4:  S-shape Dataset       (2D) * 
% 5:  Dual-behavior Dataset (2D) *
% 6:  Via-point Dataset     (3D) * 9  trajectories recorded at 100Hz
% 7:  Sink Dataset          (3D) * 11 trajectories recorded at 100Hz
% 8:  CShape bottom         (3D) * 16 trajectories recorded at 100Hz
% 9:  CShape top            (3D) --12 trajectories recorded at 100Hz
% 10: CShape all            (3D) -- x trajectories recorded at 100Hz
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pkg_dir         = pwd;
chosen_dataset  = 17; 
sub_sample      = 1; % '>2' for real 3D Datasets, '1' for 2D toy datasets
nb_trajectories = 1; % Only for real 3D data
[Data, Data_sh, att, x0_all, data, dt] = load_dataset_DS(pkg_dir, chosen_dataset, sub_sample, nb_trajectories);

vel_samples = 10; vel_size = 0.5; 
[h_data, h_att, h_vel] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);
hold on; 

M           = size(Data,1)/2;    
Xi_ref      = Data(1:M,:);
Xi_dot_ref  = Data(M+1:end,:);   
axis_limits = axis;

est_options = [];
est_options.type             = 0;   % GMM Estimation Algorithm Type
est_options.maxK             = 10;  % Maximum Gaussians for Type 1
est_options.fixed_K          = [];  % Fix K and estimate with EM for Type 1

est_options.samplerIter      = 50;  % Maximum Sampler Iterations
                                    % For type 0: 20-50 iter is sufficient
                                    % For type 2: >100 iter are needed
                                    
est_options.do_plots         = 0;   % Plot Estimation Statistics

nb_data = length(Data);
sub_sample = 1;
if nb_data > 500
    sub_sample = 2;
elseif nb_data > 1000
        sub_sample = 3;
end
est_options.sub_sample       = sub_sample;       

est_options.estimate_l       = 1;   % '0/1' Estimate the lengthscale, if set to 1
est_options.l_sensitivity    = 2;   % lengthscale sensitivity [1-10->>100]
                                    % Default value is set to '2' as in the
                                    % paper, for very messy, close to
                                    % self-intersecting trajectories, we
                                    % recommend a higher value
est_options.length_scale     = [];  % if estimate_l=0 you can define your own
                                    % l, when setting l=0 only
                                    % directionality is taken into account

[Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options);

[idx] = knnsearch(Mu', att', 'k', size(Mu,2));
Priors = Priors(:,idx);
Mu     = Mu(:,idx);
Sigma  = Sigma(:,:,idx);

Sigma(:,:,1) = 1.*max(diag(Sigma(:,:,1)))*eye(M);
Mu(:,1) = att;

clear ds_gmm; ds_gmm.Mu = Mu; ds_gmm.Sigma = Sigma; ds_gmm.Priors = Priors; 

adjusts_C  = 1;
if adjusts_C  == 1 
    if M == 2
        tot_dilation_factor = 1; rel_dilation_fact = 0.25;
    elseif M == 3
        tot_dilation_factor = 1; rel_dilation_fact = 0.75;        
    end
    Sigma_ = adjust_Covariances(ds_gmm.Priors, ds_gmm.Sigma, tot_dilation_factor, rel_dilation_fact);
    ds_gmm.Sigma = Sigma_;
end   

[~, est_labels] =  my_gmm_cluster(Xi_ref, ds_gmm.Priors, ds_gmm.Mu, ds_gmm.Sigma, 'hard', []);

lyap_constr = 2;      % 0:'convex':     A' + A < 0 (Proposed in paper)
                      % 2:'non-convex': A'P + PA < -Q given P (Proposed in paper)                                 
init_cvx    = 1;      % 0/1: initialize non-cvx problem with cvx 
symm_constr = 0;      % This forces all A's to be symmetric (good for simple reaching motions)

if lyap_constr == 0 || lyap_constr == 1
    P_opt = eye(M);
else

    [Vxf] = learn_wsaqf(Data_sh);
    P_opt = Vxf.P;
end
  
if lyap_constr == 1
    [A_k, b_k, ~] = optimize_lpv_ds_from_data(Data_sh, zeros(M,1), lyap_constr, ds_gmm, P_opt, init_cvx);
    ds_lpv = @(x) lpv_ds(x-repmat(att,[1 size(x,2)]), ds_gmm, A_k, b_k);
else
    [A_k, b_k, ~] = optimize_lpv_ds_from_data(Data, att, lyap_constr, ds_gmm, P_opt, init_cvx, symm_constr);
    ds_lpv = @(x) lpv_ds(x, ds_gmm, A_k, b_k);
end


