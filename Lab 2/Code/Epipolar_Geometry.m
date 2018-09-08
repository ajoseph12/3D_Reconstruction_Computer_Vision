clear all;
close all;

N = 8;
I1 = imread('stereo1.jpg');
I2 = imread('stereo2.jpg');

X1 = ones(2,N); % Empty vector to store the 8 points
X2 = ones(2,N);
F = zeros(3,3);  % Empty matrix to calculate the fundamental matrix

disp('Calculation of the fundamental matrix algorithm by 8 points');
[X1,X2] = clickPoints(I1,I2,N);


% Step 2 : Calculate the fundamental matrix with 8 point algorithm
F = MatF(X1,X2);

% Step 3 : Calculate the Epipoles of two camera
e1 = null(F);
e2 = null(F.');

% Click on an object point in the left image and you should see the
click(I1,I2,F)

% Loading points for "betterPointsX1X2.mat" and recalculating the F
imshow(I1);
hold on;
plot(X1(1,:), X1(2,:), 'r*');

imshow(I2);
hold on;
plot(X2(1,:), X2(2,:), 'r*');

temp = load('betterPointsX1X2.mat')
F = MatF(X1,X2)
click(I1,I2,F)

