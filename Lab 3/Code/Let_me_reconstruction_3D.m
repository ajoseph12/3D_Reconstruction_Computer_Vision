clear all; close all;

%% Calibration image pair stereo1.jpg and stereo2.jpg
% 3D coordinates of the points

load 'XYZ.mat'
load 'xyLeft.mat';
load 'xyRight.mat';

imLeft=imread('stereo1.jpg');
figure(1);clf;image(imLeft);hold on;

imRight=imread('stereo2.jpg');
figure(2);clf;image(imRight);hold on;

% Do camera calibration
% Camera 1
[ty,tx,tz]=size(imLeft);
N=12; u0 = tx/2; v0 = ty/2;
[K1,RT1]=calibTSAI(xyLeft',XYZ',N,u0,v0); % Calibration matrices
P1Tsai = K1*RT1; % Projection matrix

% Camera 2
[ty,tx,tz]=size(imRight);
N=12; u0 = tx/2; v0 = ty/2;
[K2,RT2]=calibTSAI(xyRight',XYZ',N,u0,v0); % Calibration matrices
P2Tsai = K2*RT2; % Projection matrix

% Reprojection of the points used for calibration and verify

XYZ=XYZ';
XYZ_h = [XYZ;ones(1,N)]
% Camera 1

% obtaining 2D from 3D
% u = M*X
xy_l = [];
for i = 1:N
    temp = [];
    temp_img = [];
    temp = P1Tsai*XYZ_h(:,i);
    temp_img = [temp(1)/temp(3),temp(2)/temp(3)];
    xy_l = [xy_l;temp_img];  
end

% plotting obtained 2D points on image
figure(1);
imshow(imLeft);
hold on;
plot(xy_l(:,1), xy_l(:,2), 'r+');


% Camera 2

% obtaining 2D from 3D
xy_r = [];
for i = 1:N
    temp = [];
    temp_img = [];
    temp = P2Tsai*XYZ_h(:,i);
    temp_img = [temp(1)/temp(3),temp(2)/temp(3)];
    xy_r = [xy_r;temp_img];  
end

% plotting obtained 2D points on image
figure(2);
imshow(imRight);
hold on;
plot(xy_r(:,1), xy_r(:,2), 'r+');


%% Reconstruction 3D

% Step 1. Select points from 2D images for reconstruction (from image 1 and 2)

% Select the points from 2D image for reconstruction (from image 1)
numPts_py = 4; % 4 points to see the pyramid
numPts_cube = 8; % 8 points each cube

% Select points from the object
figure(1);
imshow(imLeft);
[Px1,Py1] = ginput(numPts_py);
Ppyrlpx = [Px1 Py1];
imshow(imLeft);
[Px1,Py1] = ginput(numPts_cube);
Pcubelpx = [Px1 Py1];

% Select the same object points from the other image (from image 2)
figure(2);
imshow(imRight);
[Px2,Py2] = ginput(numPts_py);
Ppyrrpx = [Px2 Py2];
imshow(imRight);
[Px1,Py1] = ginput(numPts_cube);
Pcuberpx = [Px1 Py1];


% Step 2: Convert the coordinates in homogeneous coordinate system and then
% project them into the camera coordinate system by using individual camera 
% matrix K1 and K2

Ppyrlpx = Ppyrlpx';
Ppyrrpx = Ppyrrpx';
Pcubelpx = Pcubelpx';
Pcuberpx = Pcuberpx';

% Convert the points into homogeneous coordinate system
Pcubelpxhom = [Pcubelpx;ones(1,numPts_cube)];
Pcuberpxhom = [Pcuberpx;ones(1,numPts_cube)];
Ppyrlpxhom = [Ppyrlpx;ones(1,numPts_py)];
Ppyrrpxhom = [Ppyrrpx;ones(1,numPts_py)];


% Convert 2D object points into individual camera coordinate system using
% inverse intrinsic camera calibration matrix K1 and K2. You will have the 
% Corresponding 3D points in camera coordinate system

K1_sf = K1(:,1:3);
K2_sf = K2(:,1:3);
cam_pyr_l = inv(K1_sf)*Ppyrlpxhom;
cam_pyr_r = inv(K2_sf)*Ppyrrpxhom;
cam_cube_l =  inv(K1_sf)*Pcubelpxhom;
cam_cube_r = inv(K2_sf)*Pcuberpxhom;


% Step 3: Construct global rotation and translation matrix among this two
% camera. Use the Rotation and Translation matrix of each individual
% camera.
R_l = RT1(1:3,1:3);
R_r = RT2(1:3,1:3);
T_l = RT1(1:3,4);
T_r = RT2(1:3,4);
R = R_r*R_l';
T = T_l - R'*T_r;


% Step 4: Perform 3D reconstruction
% Apply 3D Reconstruction algorithm
% Don't forget to save the final 3D coordinates as homogeneous coordinate
% system.

%camera coordinates
Pcam_pyr =[];
Pcam_cube = [];
%world coordinates
PW_pyr = [];
PW_cube = [];
% Global extrinsic parameters
RT = [R T];
RT = [RT; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D reconstruction of the Pyramid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for cpt=1:numPts_py
    % Construction of the matrice A
    pl=cam_pyr_l(:,cpt);
    pr=cam_pyr_r(:,cpt);
    
    A=[pl  -R'*pr cross(pl,R'*pr)];
    
    abc=inv(A)*T;
    
    a0=abc(1);
    b0=abc(2);
    
    final_Point = [];
    % Calculate the final trianglated point with appropriate equation
    P1 = a0*pl;
    P2 = T + b0*R'*pr;
    P = P1 + 0.5*(P2-P1);
    final_Point = P;
    
    Pcam_pyr=[Pcam_pyr [final_Point;1]]; % save as homogeneous coordinate
    
    clear A;
end

% Conversion of coordinates in the world coordinate system

for cpt=1:numPts_py
    rwP = [];
    % do inverse transform with respect to a camera extrinsic calibration 
    % parameter (rotation and translation) matrix to obtain the position of 
    % the 3D points in the real world coordinate system
    
    % Xc = RT*Xw therefore Xw = inv(RT)*Xc
    rwP = inv(RT)*Pcam_pyr(:,cpt);
    PW_pyr = [PW_pyr rwP];
end

% Draw the 3D object
figure(3)
for cpt=1:numPts_py
    plot3(PW_pyr(1,cpt),PW_pyr(2,cpt),PW_pyr(3,cpt),'g*'); hold on; % Draw point
end

% Draw lines
for cpt=1:numPts_py
    for j=cpt+1:numPts_py
        line([PW_pyr(1,cpt) PW_pyr(1,j)], [PW_pyr(2,cpt) PW_pyr(2,j)], [PW_pyr(3,cpt) PW_pyr(3,j)]); 
    end
end
hold off;
title('3D reconstruction in the real world coordinate system');
grid on;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D reconstruction of the Big Cube
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for cpt=1:numPts_cube
    % Construction de la matrice A
    pl=cam_cube_l(:,cpt);
    pr=cam_cube_r(:,cpt);
    
    A=[pl  -R'*pr cross(pl,R'*pr)];
    
    abc=inv(A)*T;
    
    a0=abc(1);
    b0=abc(2);
    
    final_Point = [];
    % Calculate the final trianglated point with appropriate equation
    P1 = a0*pl;
    P2 = T + b0*R.'*pr;
    P = P1 + 0.5*(P2-P1);
    final_Point = P;
    
    Pcam_cube=[Pcam_cube [final_Point;1]]; % save as homogeneous coordinate
    
    clear A;
end

% Conversion of coordinates in the world coordinate system

for cpt=1:numPts_cube
    rwP = [];
    % do inverse transform with respect to a camera extrinsic calibration 
    % parameter (rotation and translation) matrix to obtain the position of 
    % the 3D points in the real world coordinate system
    % Xc = RT*Xw therefore Xw = inv(RT)*Xc
    RT = [R T];
    RT = [RT; 0 0 0 1];
    rwP = inv(RT)*Pcam_cube(:,cpt);
    PW_cube = [PW_cube rwP];
end

% Draw the 3D object
figure(4)
for cpt=1:numPts_cube
    plot3(PW_cube(1,cpt),PW_cube(2,cpt),PW_cube(3,cpt),'g*'); hold on; % Draw point
end

% Draw lines
for cpt=1:numPts_cube
    for j=cpt+1:numPts_cube
        line([PW_cube(1,cpt) PW_cube(1,j)], [PW_cube(2,cpt) PW_cube(2,j)], [PW_cube(3,cpt) PW_cube(3,j)]); 
    end
end
hold off;
title('3D reconstruction in the real world coordinate system');
grid on;