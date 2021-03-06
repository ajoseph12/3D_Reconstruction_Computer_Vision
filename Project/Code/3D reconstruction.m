
% Focal Length and Principal Points of both the cameras obtained from
% previous tasks.
%%%%%%%%%%%%%%%%%%%%%%%Left Camera Focal Length and Principal Point%%%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
fx_rgb_left = 540.1656; fy_rgb_left =  539.3324; 
cx_rgb_left = 316.0161; cy_rgb_left =  248.6396;  
%%%%%%%%%%%%%%%%%%%%%%%Right Camera Focal Length and Principal Point%%%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
fx_rgb_right = 514.7264; fy_rgb_right =  515.3067; 
cx_rgb_right = 305.4592; cy_rgb_right =  267.1152; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Kinect Camera_parameters for depth camera%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fxd =  5.7616540758591043e+02;
fyd = 5.7375619782082447e+02;
cxd = 3.2442516903961865e+02;
cyd = 2.3584766381177013e+02;

rotation_Matrix =  inv([  9.9998579449446667e-01, 3.4203777687649762e-03, -4.0880099301915437e-03;
    -3.4291385577729263e-03, 9.9999183503355726e-01, -2.1379604698021303e-03;
    4.0806639192662465e-03, 2.1519484514690057e-03,  9.9998935859330040e-01]);
% translation_Matrix
translation_Matrix = -[  2.2142187053089738e-02, -1.4391632009665779e-04, -7.9356552371601212e-03 ]';
%%%%%%%%Extrinsic Parameters of Camera 2( Right Camera) with respect to Camera 1(Left Camera)%%%%%%%%%%%%%%%%%%%%
rot_matrix = [0.710519320731973,-0.181645368929196,-0.679828842292788;0.126427485529458,0.983340249501000,-0.130606449358403;0.692227120076406,0.00684935455986787,0.721647213375648];
rot_matrix = inv(rot_matrix);
translation_vector = [ -865.531236035213 ,110.919357309850,378.475325269407 ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%For Left camera%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
left_rgb = imread('LeftRgbobjects_scene21.png'); %Read the left color image
left_depth = imread('LeftDepthobjects_scene21.png'); %Read the left depth image
depth_size = size(left_depth); 

row_number = depth_size(1);
column_number = depth_size(2);

pixel_3D = zeros(row_number,column_number, 3 );

left_cam = zeros(row_number,column_number, 3);
left_cam_color = zeros(row_number,column_number, 3);

left_nz_shw_pt = zeros(row_number * column_number, 3);
left_nz_shw_pt_color = zeros(row_number * column_number, 3);

nzcount = 0;

for row=1:row_number
    for col=1:column_number
        pixel_3D(row,col,1) = ((col - cxd)/ fxd) * double( left_depth(row, col)) ;
        
        pixel_3D(row,col,2) = ((row - cyd) / fyd) * double(left_depth(row, col)) ;
        pixel_3D(row,col,3) = double( left_depth(row, col));
        
        if pixel_3D(row,col,3) > 0
            pixel_temp = [pixel_3D(row, col, 1), pixel_3D(row, col, 2), pixel_3D(row, col, 3)];
            temp = rotation_Matrix * pixel_temp' + translation_Matrix; % 2
            P2Drgb_x = round(fx_rgb_left * temp(1)/temp(3) + cx_rgb_left);% 3
            P2Drgb_y = round(fy_rgb_left * temp(2)/temp(3) + cy_rgb_left);

            if P2Drgb_x > 0 && P2Drgb_y > 0 && P2Drgb_x < column_number  && P2Drgb_y < row_number
                color = left_rgb(P2Drgb_y, P2Drgb_x, :);
                left_cam(row_number, column_number, :) = temp;
                left_cam_color(row_number, column_number, :) = color; 

                nzcount = nzcount + 1;
                left_nz_shw_pt(nzcount,:) =  temp;
                left_nz_shw_pt_color(nzcount, :) = color;
            end
        end
    end
end

left_nz_shw_pt_t = left_nz_shw_pt(1:nzcount, :);
left_nz_shw_pt_color_t = left_nz_shw_pt_color(1:nzcount, :);


left_nz_shw_pt_t_rot = zeros(nzcount, 3);
for counter=1:nzcount
    left_nz_shw_pt_t_rot(counter, :) = (rot_matrix *  left_nz_shw_pt_t(counter, :)' + translation_vector')';
 end
left_nz_shw_pt_t = left_nz_shw_pt_t_rot;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% For right camera%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

right_rgb = imread('RightRgbobjects_scene21.png'); % Read the color image of Right Camera
right_depth = imread('RightDedpthobjects_scene21.png'); %Read the left image of Right Camera

depth_size = size(right_depth);%

row_number = depth_size(1);
column_number = depth_size(2);

pixel_3D = zeros(row_number,column_number, 3 );

right_cam = zeros(row_number,column_number, 3);
right_cam_color = zeros(row_number,column_number, 3);

right_nz_shw_pt = zeros(row_number * column_number, 3);
right_nz_shw_pt_color = zeros(row_number * column_number, 3);

nzcount = 0;

for row=1:row_number
    for col=1:column_number
        pixel_3D(row,col,1) = ((col - cxd)/ fxd) * double(right_depth(row,col)) ;
        
        pixel_3D(row,col,2) = ((row - cyd)/ fyd) * double(right_depth(row,col)) ;
        pixel_3D(row,col,3) = double( right_depth(row,col));
        if pixel_3D(row,col,3) > 0
            pixel_temp = [pixel_3D(row, col, 1), pixel_3D(row, col, 2), pixel_3D(row, col, 3)];
            temp = rotation_Matrix * pixel_temp' + translation_Matrix;
            P2Drgb_x = round(fx_rgb_right * temp(1)/temp(3) + cx_rgb_right);
            P2Drgb_y = round(fy_rgb_right * temp(2)/temp(3) + cy_rgb_right);

            if P2Drgb_x > 0 && P2Drgb_y > 0 && P2Drgb_x < column_number  && P2Drgb_y < row_number
                color = right_rgb(P2Drgb_y, P2Drgb_x, :);
                right_cam(row_number, column_number, :) = temp;
                right_cam_color(row_number, column_number, :) = color; 

                nzcount = nzcount + 1;
                right_nz_shw_pt(nzcount,:) =  temp;
                right_nz_shw_pt_color(nzcount, :) = color;
            end
        end
    end
end

right_nz_shw_pt_t = right_nz_shw_pt(1:nzcount, :);
right_nz_shw_pt_color_t = right_nz_shw_pt_color(1:nzcount, :);

right_nz_shw_pt_t_rot = right_nz_shw_pt_t;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Combined Results for both the Cameras %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
size_left = size(left_nz_shw_pt_t);
size_left = size_left(1);

size_right = size(right_nz_shw_pt_t);
size_right = size_right(1);
totalsize = size_left + size_right;

csp = zeros(totalsize, 3);
cspc = zeros(totalsize, 3);
for i=1:totalsize
    if i <= size_left
        csp(i, :) = left_nz_shw_pt_t(i, :);
        cspc(i, :) = left_nz_shw_pt_color_t(i, :) / 256.0;
    else
        csp(i, :) = right_nz_shw_pt_t_rot( i - size_left, :);
        cspc(i, :) = right_nz_shw_pt_color_t(i-size_left, :) / 256.0;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%Displaying Results%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Uncommnent the following commands to see the figures generated%%%%%%%%%%%%%%%%%

%pcshow(left_nz_shw_pt_t, left_nz_shw_pt_color_t / 256.0);
%pcshow(left_nz_shw_pt_t);
 
%pcshow(right_nz_shw_pt_t_rot, right_nz_shw_pt_color_t / 256.0);
%pcshow(right_nz_shw_pt_t_rot)

%pcshow(csp, cspc)
%pcshow(csp)

%pcshow([csp(:,1), - csp(:, 2), -csp(:, 3)], cspc)




