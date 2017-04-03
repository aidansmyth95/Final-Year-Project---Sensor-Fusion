close all; clc; close all;

%sometimes manual editing of ascii characters at beginningof txt file
%is necessary
filename = 'off-chip yaw\t3.txt';
data = textread(filename, '', 'delimiter', '\t','emptyvalue', NaN);
timeStamp = data(:,1);
yaw = data(:,2);

filename = 'on-chip yaw\t3.txt';
data = textread(filename, '', 'delimiter', '\t','emptyvalue', NaN);
timeStamp = data(:,1);
DMP_yaw = data(:,2);

tout = timeStamp;
figure(1);
plot(tout(1:1000),DMP_yaw(1:1000),'.r'); hold on;
plot(tout(1:1000),yaw(1:1000),'.k');hold on;
ylabel('Angle (degrees)');
xlabel('Time (seconds)');
title('Yaw');
legend('DMP','Kalman')
grid on;

% errCX = immse(comp_X(5,:), DMP_X(5,:));
% errCY = immse(comp_Y(5,:), DMP_Y(5,:));
% errKX = immse(kal_X(5,:), DMP_X(5,:));
% errKY = immse(kal_Y(5,:), DMP_Y(5,:));
% 
% fprintf('\n The complementary roll mean-squared error is %0.4f\n', errCX);
% fprintf('\n The complementary pitch mean-squared error is %0.4f\n', errCY);
% fprintf('\n The kalman roll mean-squared error is %0.4f\n', errKX);
% fprintf('\n The kalman pitch mean-squared error is %0.4f\n', errKY);
% 

