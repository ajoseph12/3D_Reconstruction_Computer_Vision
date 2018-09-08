% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1924.625078079663808 ; 1939.010491609418978 ];

%-- Principal point:
cc = [ 837.048200240172719 ; 849.579804723215034 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.088963110669255 ; 0.270036104061024 ; 0.020267160734389 ; -0.002922116451425 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 31.816944209029568 ; 32.298160173522504 ];

%-- Principal point uncertainty:
cc_error = [ 21.429836664676028 ; 16.486276767454410 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.027226016192494 ; 0.185185415029392 ; 0.002201048699957 ; 0.002822080712658 ; 0.000000000000000 ];

%-- Image size:
nx = 1624;
ny = 1224;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 25;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.138044e+00 ; -2.171663e+00 ; -2.648634e-01 ];
Tc_1  = [ -5.019322e-02 ; -4.996191e-01 ; 3.735256e+00 ];
omc_error_1 = [ 3.155209e-02 ; 2.976219e-02 ; 6.355448e-02 ];
Tc_error_1  = [ 4.202089e-02 ; 3.204153e-02 ; 6.413091e-02 ];

%-- Image #2:
omc_2 = [ 1.994641e+00 ; 2.071042e+00 ; 5.509839e-01 ];
Tc_2  = [ -1.277556e-01 ; -2.778446e-01 ; 2.130222e+00 ];
omc_error_2 = [ 8.576481e-03 ; 9.017694e-03 ; 1.660183e-02 ];
Tc_error_2  = [ 2.403720e-02 ; 1.819909e-02 ; 3.623079e-02 ];

%-- Image #3:
omc_3 = [ -1.949934e+00 ; -2.099830e+00 ; 8.992467e-02 ];
Tc_3  = [ 3.835071e-01 ; -2.535803e-01 ; 2.107065e+00 ];
omc_error_3 = [ 2.041761e-02 ; 2.891078e-02 ; 4.139263e-02 ];
Tc_error_3  = [ 2.379691e-02 ; 1.813556e-02 ; 3.389358e-02 ];

%-- Image #4:
omc_4 = [ 2.057763e+00 ; 2.136983e+00 ; 5.370460e-01 ];
Tc_4  = [ -2.688666e-02 ; -2.202180e-01 ; 1.847731e+00 ];
omc_error_4 = [ 8.575580e-03 ; 8.023651e-03 ; 1.668182e-02 ];
Tc_error_4  = [ 2.075091e-02 ; 1.575356e-02 ; 3.104011e-02 ];

%-- Image #5:
omc_5 = [ -2.139926e+00 ; -2.153066e+00 ; -2.773830e-01 ];
Tc_5  = [ 8.869278e-02 ; -1.736231e-01 ; 1.808179e+00 ];
omc_error_5 = [ 8.450713e-03 ; 1.026200e-02 ; 2.035886e-02 ];
Tc_error_5  = [ 2.017763e-02 ; 1.541358e-02 ; 2.974849e-02 ];

%-- Image #6:
omc_6 = [ -2.094305e+00 ; -2.115277e+00 ; -2.230504e-01 ];
Tc_6  = [ 1.323766e-01 ; -1.549911e-01 ; 1.706375e+00 ];
omc_error_6 = [ 7.887013e-03 ; 1.015291e-02 ; 1.986852e-02 ];
Tc_error_6  = [ 1.903154e-02 ; 1.455945e-02 ; 2.785244e-02 ];

%-- Image #7:
omc_7 = [ -1.840676e+00 ; -1.904143e+00 ; -4.812053e-01 ];
Tc_7  = [ 1.126652e-01 ; -1.695219e-01 ; 1.642610e+00 ];
omc_error_7 = [ 5.210490e-03 ; 7.926347e-03 ; 1.599684e-02 ];
Tc_error_7  = [ 1.844188e-02 ; 1.401621e-02 ; 2.623782e-02 ];

%-- Image #8:
omc_8 = [ 2.024142e+00 ; 2.181553e+00 ; -2.467287e-01 ];
Tc_8  = [ -3.409681e-03 ; -2.138259e-01 ; 1.620190e+00 ];
omc_error_8 = [ 7.755867e-03 ; 1.040792e-02 ; 1.997581e-02 ];
Tc_error_8  = [ 1.821175e-02 ; 1.374047e-02 ; 2.547846e-02 ];

%-- Image #9:
omc_9 = [ -2.044343e+00 ; -2.051417e+00 ; -1.174482e-01 ];
Tc_9  = [ 7.538950e-02 ; -1.438506e-01 ; 1.421071e+00 ];
omc_error_9 = [ 7.367102e-03 ; 9.478548e-03 ; 1.909069e-02 ];
Tc_error_9  = [ 1.589828e-02 ; 1.211978e-02 ; 2.256040e-02 ];

%-- Image #10:
omc_10 = [ -2.192505e+00 ; -2.203347e+00 ; -3.762016e-01 ];
Tc_10  = [ -1.457100e-01 ; -1.006623e-01 ; 1.332272e+00 ];
omc_error_10 = [ 6.790950e-03 ; 7.995939e-03 ; 1.726265e-02 ];
Tc_error_10  = [ 1.499536e-02 ; 1.137315e-02 ; 2.221947e-02 ];

%-- Image #11:
omc_11 = [ 2.143248e+00 ; 2.130719e+00 ; 5.396098e-01 ];
Tc_11  = [ -1.753230e-01 ; -6.348615e-02 ; 1.115459e+00 ];
omc_error_11 = [ 7.506199e-03 ; 5.286264e-03 ; 1.542618e-02 ];
Tc_error_11  = [ 1.263166e-02 ; 9.541128e-03 ; 1.897204e-02 ];

%-- Image #12:
omc_12 = [ -2.065270e+00 ; -2.120481e+00 ; -8.760554e-02 ];
Tc_12  = [ -3.618503e-02 ; -4.736017e-01 ; 3.585836e+00 ];
omc_error_12 = [ 3.764502e-02 ; 3.911740e-02 ; 8.205823e-02 ];
Tc_error_12  = [ 4.033380e-02 ; 3.064120e-02 ; 5.931132e-02 ];

%-- Image #13:
omc_13 = [ -2.071004e+00 ; -2.113413e+00 ; -6.314367e-01 ];
Tc_13  = [ -1.441106e-01 ; -5.292644e-02 ; 1.100181e+00 ];
omc_error_13 = [ 4.920853e-03 ; 7.351799e-03 ; 1.587616e-02 ];
Tc_error_13  = [ 1.235899e-02 ; 9.391761e-03 ; 1.884988e-02 ];

%-- Image #14:
omc_14 = [ 2.133120e+00 ; 2.114931e+00 ; 3.808333e-02 ];
Tc_14  = [ -7.425466e-04 ; -6.825145e-02 ; 1.126447e+00 ];
omc_error_14 = [ 7.781107e-03 ; 7.375080e-03 ; 1.686333e-02 ];
Tc_error_14  = [ 1.254761e-02 ; 9.550466e-03 ; 1.790839e-02 ];

%-- Image #15:
omc_15 = [ -1.979873e+00 ; -2.013830e+00 ; -4.924022e-01 ];
Tc_15  = [ -5.279399e-02 ; -2.223808e-02 ; 9.557942e-01 ];
omc_error_15 = [ 4.774911e-03 ; 7.258460e-03 ; 1.608941e-02 ];
Tc_error_15  = [ 1.066197e-02 ; 8.124050e-03 ; 1.575016e-02 ];

%-- Image #16:
omc_16 = [ 2.178603e+00 ; 2.196930e+00 ; 2.579851e-01 ];
Tc_16  = [ -8.018292e-02 ; -9.669783e-02 ; 9.350113e-01 ];
omc_error_16 = [ 7.387723e-03 ; 6.171166e-03 ; 1.538963e-02 ];
Tc_error_16  = [ 1.048822e-02 ; 7.962421e-03 ; 1.531590e-02 ];

%-- Image #17:
omc_17 = [ 2.047092e+00 ; 1.944400e+00 ; 1.875295e-01 ];
Tc_17  = [ -2.309388e-01 ; -9.476013e-02 ; 1.245369e+00 ];
omc_error_17 = [ 7.824968e-03 ; 8.641855e-03 ; 1.527684e-02 ];
Tc_error_17  = [ 1.400976e-02 ; 1.065780e-02 ; 2.089026e-02 ];

%-- Image #18:
omc_18 = [ 2.134040e+00 ; 2.233129e+00 ; -3.247584e-01 ];
Tc_18  = [ 1.047026e-01 ; -1.340672e-01 ; 1.551352e+00 ];
omc_error_18 = [ 9.946518e-03 ; 1.034556e-02 ; 2.321807e-02 ];
Tc_error_18  = [ 1.740589e-02 ; 1.314620e-02 ; 2.370478e-02 ];

%-- Image #19:
omc_19 = [ -1.875577e+00 ; -2.015615e+00 ; -2.981789e-01 ];
Tc_19  = [ 1.118206e-01 ; -4.860457e-01 ; 3.396804e+00 ];
omc_error_19 = [ 1.133946e-02 ; 1.383612e-02 ; 2.692069e-02 ];
Tc_error_19  = [ 3.828909e-02 ; 2.906396e-02 ; 5.527742e-02 ];

%-- Image #20:
omc_20 = [ -1.914119e+00 ; -2.150995e+00 ; 1.219140e-01 ];
Tc_20  = [ 2.463656e-01 ; -4.864429e-01 ; 3.132959e+00 ];
omc_error_20 = [ 2.278707e-02 ; 2.691091e-02 ; 5.339107e-02 ];
Tc_error_20  = [ 3.546779e-02 ; 2.674000e-02 ; 5.029008e-02 ];

%-- Image #21:
omc_21 = [ -1.895392e+00 ; -2.075067e+00 ; -5.386913e-01 ];
Tc_21  = [ -3.560779e-02 ; -4.594331e-01 ; 3.045302e+00 ];
omc_error_21 = [ 9.646788e-03 ; 1.132181e-02 ; 2.210923e-02 ];
Tc_error_21  = [ 3.433228e-02 ; 2.612307e-02 ; 5.057395e-02 ];

%-- Image #22:
omc_22 = [ 2.053452e+00 ; 2.183957e+00 ; -2.380980e-01 ];
Tc_22  = [ -6.996432e-02 ; -4.030383e-01 ; 2.862728e+00 ];
omc_error_22 = [ 1.029071e-02 ; 1.419701e-02 ; 2.615963e-02 ];
Tc_error_22  = [ 3.221508e-02 ; 2.440757e-02 ; 4.654675e-02 ];

%-- Image #23:
omc_23 = [ -1.896251e+00 ; -2.032048e+00 ; 7.812090e-02 ];
Tc_23  = [ 2.379478e-01 ; -3.876594e-01 ; 2.889394e+00 ];
omc_error_23 = [ 1.373730e-02 ; 1.677673e-02 ; 3.392817e-02 ];
Tc_error_23  = [ 3.268286e-02 ; 2.461733e-02 ; 4.574161e-02 ];

%-- Image #24:
omc_24 = [ -1.876110e+00 ; -1.948332e+00 ; 1.437273e-01 ];
Tc_24  = [ 2.451219e-01 ; -3.484761e-01 ; 2.758736e+00 ];
omc_error_24 = [ 1.047446e-02 ; 1.260018e-02 ; 2.616834e-02 ];
Tc_error_24  = [ 3.118291e-02 ; 2.348063e-02 ; 4.340258e-02 ];

%-- Image #25:
omc_25 = [ 2.022468e+00 ; 2.221184e+00 ; -2.666535e-01 ];
Tc_25  = [ 8.172282e-02 ; -3.626301e-01 ; 2.612417e+00 ];
omc_error_25 = [ 9.689121e-03 ; 1.319616e-02 ; 2.471696e-02 ];
Tc_error_25  = [ 2.945367e-02 ; 2.224264e-02 ; 4.164874e-02 ];
