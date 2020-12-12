%% 
clc
close all 
global Start_X Start_Y
%load('ReferenceStates.mat');

[NumberOfSamples_PP,~]=size(StatesLog_PP);

%% Plot 1

figure
plot(StatesLog_PP(1:NumberOfSamples_PP,2), ...
     StatesLog_PP(1:NumberOfSamples_PP,3), 'LineWidth', 2, 'Color', 'b');

%%Plot Parking Spot
% hold on
% rectangle('Position',[Obstacle1Position_X - LengthOfObstacle/2,...
%                       Obstacle1Position_Y - WidthOfObstacle/2,...
%                       LengthOfObstacle,WidthOfObstacle], ...
%                       'FaceColor',[1 0 0],'EdgeColor','b','LineWidth',2) 

%%Plot Parking Spot
for j = 1:Number_Of_Obstacles
    hold on
    rectangle('Position',[Array_Of_Obstacles(j,1)-Array_Of_Obstacles(j,3)/2, ...
                          Array_Of_Obstacles(j,2)-Array_Of_Obstacles(j,4)/2, ...
                          Array_Of_Obstacles(j,3),Array_Of_Obstacles(j,4)],...
              'FaceColor',[1 0 0],'EdgeColor','b','LineWidth',2)
end                 
                  
xlabel('x position(m)')
ylabel('y position(m)')
%legend('Simulation', 'Optimal Path','WayPoints')
xlim([-1 20])
ylim([-3 5])
%%


%% Plot 2
figure
subplot(2,1,1)
stairs(StatesLog_PP(1:NumberOfSamples_PP,1),...
       StatesLog_PP(1:NumberOfSamples_PP,4),'LineWidth',2);
legend('v_x in NED vs Time')
xlabel('Time(s)')
ylabel('v_x velocity in NED Frame(m/s)')
subplot(2,1,2)
grid on
stairs(StatesLog_PP(1:NumberOfSamples_PP,1),...
       StatesLog_PP(1:NumberOfSamples_PP,5),'LineWidth',2);
legend('v_y in NED vs Time')
xlabel('Time(s)')
ylabel('v_y velocity in NED Frame(m/s)')
