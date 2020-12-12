%% 
clc;
clear all;
close all;

%load('ReferenceStates.mat');
StatesLog1 = load('18-Jan-2018_Prediction_Horizon_10_Control_Horizon_5_Q11_10_Q22_10_R_1');
StatesLog2 = load('18-Jan-2018_Prediction_Horizon_15_Control_Horizon_5_Q11_10_Q22_10_R_1');
StatesLog3 = load('18-Jan-2018_Prediction_Horizon_20_Control_Horizon_5_Q11_10_Q22_10_R_1');
% StatesLog4 = load('06-Jan-2018_WP_Set_1_Prediction_Horizon_16_Control_Horizon_1_With_Disturbance');
%StatesLog5 = load('28-Dec-2017_WP_Set_6_Prediction_Horizon_16_Control_Horizon_1_With_Disturbance');
%StatesLog6 = load('28-Dec-2017_WP_Set_6_Prediction_Horizon_20_Control_Horizon_1_With_Disturbance');

[NumberOfSamples_1,~]=size(StatesLog1.StatesLog);
[NumberOfSamples_2,~]=size(StatesLog2.StatesLog);
[NumberOfSamples_3,~]=size(StatesLog3.StatesLog);
%[NumberOfSamples_4,~]=size(StatesLog4.StatesLog);
%[NumberOfSamples_5,~]=size(StatesLog5.StatesLog);
%[NumberOfSamples_6,~]=size(StatesLog6.StatesLog);

Way_Points = StatesLog1.Way_Points;

Way_Points = [0 0; Way_Points];
x_waypoints = Way_Points(:,1);
y_waypoints = Way_Points(:,2);
n_of_points = length(x_waypoints);

ang = 0:0.01:2*pi; 
xp = 2*cos(ang);
yp = 2*sin(ang);

figure
line(x_waypoints,y_waypoints,'Color','red','LineStyle','--',...
                                                'LineWidth',3);
grid on                                                                                    
hold on
plot(StatesLog1.StatesLog(1:NumberOfSamples_1,2),...
     StatesLog1.StatesLog(1:NumberOfSamples_1,3),...
     'LineWidth',2);
hold on 
plot(StatesLog2.StatesLog(1:NumberOfSamples_2,2),...
     StatesLog2.StatesLog(1:NumberOfSamples_2,3),...
     'LineWidth',2);
hold on 
plot(StatesLog3.StatesLog(1:NumberOfSamples_3,2),...
     StatesLog3.StatesLog(1:NumberOfSamples_3,3),...
     'LineWidth',2);
% hold on 
% plot(StatesLog4.StatesLog(1:NumberOfSamples_4,2),...
%      StatesLog4.StatesLog(1:NumberOfSamples_4,3),...
%      'LineWidth',2);
% hold on 
% plot(StatesLog5.StatesLog(1:NumberOfSamples_5,2),...
%      StatesLog5.StatesLog(1:NumberOfSamples_5,3),...
%      'LineWidth',2);
% hold on 
% plot(StatesLog6.StatesLog(1:NumberOfSamples_6,2),...
%      StatesLog6.StatesLog(1:NumberOfSamples_6,3),...
%      'LineWidth',2); 


xlabel('x position(m)')
ylabel('y position(m)')
lgd1 = legend('Minimum Walk Path', ...
'N_{p} = 15, N_{c} = 2, Q_{11} = 1, Q_{22} = 1, R_{11} = R_{22} = 0.01', ... 
'N_{p} = 15, N_{c} = 2, Q_{11} = 1, Q_{22} = 1, R_{11} = R_{22} = 0.02', ...
'N_{p} = 15, N_{c} = 2, Q_{11} = 1, Q_{22} = 1, R_{11} = R_{22} = 0.04');
          
lgd1.FontSize = 14;
      
for i = 1:length(Way_Points)
    hold on
    plot(Way_Points(i,1)+ xp, Way_Points(i,2)+ yp,'Color', 'g');
end

% figure
% plot(StatesLog1.Normalized_Distance(1:NumberOfSamples_1,1), ...
%      StatesLog1.Normalized_Distance(1:NumberOfSamples_1,2)); 
% hold on
% plot(StatesLog2.Normalized_Distance(1:NumberOfSamples_2,1), ...
%      StatesLog2.Normalized_Distance(1:NumberOfSamples_2,2));
% hold on
% plot(StatesLog3.Normalized_Distance(1:NumberOfSamples_3,1), ...
%      StatesLog3.Normalized_Distance(1:NumberOfSamples_3,2));
% % hold on
% % plot(StatesLog4.Normalized_Distance(1:NumberOfSamples_4,1), ...
% %      StatesLog4.Normalized_Distance(1:NumberOfSamples_4,2)); 
% % hold on
% % plot(StatesLog5.Normalized_Distance(1:NumberOfSamples_5,1), ...
% %       StatesLog5.Normalized_Distance(1:NumberOfSamples_5,2)); 
% grid on 
% xlabel('time(sec)')
% ylabel('Error')
% lgd2 = legend('Output Horizon = 4, Input Horizon = 2', ... 
%               'Output Horizon = 4, Input Horizon = 2', ...
%               'Output Horizon = 4, Input Horizon = 2');
% lgd2.FontSize = 14;



Normalized_Distance_Cum_1 = [0, StatesLog1.Normalized_Distance(1,2)];
Cumulative_Error_1 = StatesLog1.Normalized_Distance(1,2);

Normalized_Distance_Cum_2 = [0, StatesLog2.Normalized_Distance(1,2)];
Cumulative_Error_2 = StatesLog2.Normalized_Distance(1,2);

Normalized_Distance_Cum_3 = [0, StatesLog3.Normalized_Distance(1,2)];
Cumulative_Error_3 = StatesLog3.Normalized_Distance(1,2);

Normalized_Distance_Cum_4 = [];
Normalized_Distance_Cum_5 = [];


Normalized_Distance_Avg_Cum_1 = [];
Normalized_Distance_Avg_Cum_2 = [];
Normalized_Distance_Avg_Cum_3 = [];
Normalized_Distance_Avg_Cum_4 = [];
Normalized_Distance_Avg_Cum_5 = [];

for i = 2:1:NumberOfSamples_1
    
    Cumulative_Error_1 = Cumulative_Error_1 + ...
    0.05*abs((StatesLog1.Normalized_Distance(i,2)+StatesLog1.Normalized_Distance(i-1,2)))/2; 
    Normalized_Distance_Cum_1 = [Normalized_Distance_Cum_1; 
    StatesLog1.Normalized_Distance(i,1) Cumulative_Error_1];
end

  
for i = 2:1:NumberOfSamples_2
    
    Cumulative_Error_2 = Cumulative_Error_2 + ...
    0.05*abs((StatesLog2.Normalized_Distance(i,2)+StatesLog2.Normalized_Distance(i-1,2)))/2; 
    Normalized_Distance_Cum_2 = [Normalized_Distance_Cum_2; 
    StatesLog2.Normalized_Distance(i,1) Cumulative_Error_2];
end

for i = 2:1:NumberOfSamples_3
    
    Cumulative_Error_3 = Cumulative_Error_3 + ...
    0.05*abs((StatesLog3.Normalized_Distance(i,2)+StatesLog3.Normalized_Distance(i-1,2)))/2; 
    Normalized_Distance_Cum_3 = [Normalized_Distance_Cum_3; 
    StatesLog3.Normalized_Distance(i,1) Cumulative_Error_3];
end

% for i =1:1:NumberOfSamples_4
%     
%     Cumulative_Error_4 = Cumulative_Error_4 + StatesLog4.Normalized_Distance(i,2); 
%     Normalized_Distance_Cum_4 = [Normalized_Distance_Cum_4; 
%         StatesLog4.Normalized_Distance(i,1) 0.05*Cumulative_Error_4];
% end

% for i =1:1:NumberOfSamples_5
%     
%     Cumulative_Error_5 = Cumulative_Error_5 + StatesLog5.Normalized_Distance(i,2); 
%     Normalized_Distance_Cum_5 = [Normalized_Distance_Cum_5; 
%         StatesLog5.Normalized_Distance(i,1) 0.05*Cumulative_Error_5];
% end


figure
plot(Normalized_Distance_Cum_1(1:NumberOfSamples_1,1), ...
     Normalized_Distance_Cum_1(1:NumberOfSamples_1,2)); 
hold on
plot(Normalized_Distance_Cum_2(1:NumberOfSamples_2,1), ...
     Normalized_Distance_Cum_2(1:NumberOfSamples_2,2)); 
hold on
plot(Normalized_Distance_Cum_3(1:NumberOfSamples_3,1), ...
     Normalized_Distance_Cum_3(1:NumberOfSamples_3,2)); 
% hold on
% plot(Normalized_Distance_Cum_4(1:NumberOfSamples_4,1), ...
%      Normalized_Distance_Cum_4(1:NumberOfSamples_4,2)); 
% hold on
% plot(Normalized_Distance_Cum_5(1:NumberOfSamples_5,1), ...
%      Normalized_Distance_Cum_5(1:NumberOfSamples_5,2)); 
grid on 
xlabel('time(sec)')
ylabel('Cumulative Error')
lgd3 = legend(...
'N_{p} = 15, N_{c} = 2, Q_{11} = 1, Q_{22} = 1 , R_{11} = R_{22} = 0.01', ... 
'N_{p} = 15, N_{c} = 2, Q_{11} = 1, Q_{22} = 1 , R_{11} = R_{22} = 0.02', ...
'N_{p} = 15, N_{c} = 2, Q_{11} = 1, Q_{22} = 1 , R_{11} = R_{22} = 0.04');
lgd3.FontSize = 14;