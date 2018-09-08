function click(I1,I2,F)


% figure(1);
% subplot(1,2,1)
% imagesc(I1);
% subplot(1,2,2)
% imagesc(I2);

P = ones(2);

% Viewing Images
figure; 
subplot(1,2,1);
imshow(I1);hold on;
subplot(1,2,2); 
imshow(I2);hold on;

for i=1:100
% Entering a point on the image
[xi,yi] = ginput(1); 
P(1) = xi;
P(2) = yi;

plot(P(1), P(2),'g+');

% Plotting of the epipolar lines for the point P
x = 1:size(I1,2);
y = [];
% Replace y values with correct values in order to draw the line
temp = []

x_1 = [P(1) P(2) 1]
% l = (a,b,c)
l_2 = F*x_1.'

for i = 1:size(I1,2)
    temp = []
    % equation of line ax + by + c
    temp = -(l_2(3) + l_2(1)*x(i))/l_2(2)
    y = [y;temp]
    
end 

% Plotting of the lines in the right image
subplot(1,2,2); 
plot(x,y,'b');

end