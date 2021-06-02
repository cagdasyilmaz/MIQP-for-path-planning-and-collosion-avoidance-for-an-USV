%% 
clc
close all 
global Start_X Start_Y
%load('ReferenceStates.mat');

[NumberOfSamples,~]=size(StatesLog);

[NumberOfSamples_PP,~]=size(StatesLog_PP);
% 
% figure
% plot(ReferenceStates(1:NumberOfSamples,2)-OutputStates(1:NumberOfSamples,2));
% title('X-Pos Error (m)');
% figure
% plot(ReferenceStates(1:NumberOfSamples,3)-OutputStates(1:NumberOfSamples,3));
% title('Y-Pos Error (m)');
% figure

% plot(ReferenceStates(1:NumberOfSamples,7)-OutputStates(1:NumberOfSamples,7));
% title('Yaw-Pos Error (m)');
% figure

% plot(ReferenceStates(1:NumberOfSamples,8)-OutputStates(1:NumberOfSamples,8));
% title('X-Spd Error (m)');
% figure

% plot(ReferenceStates(1:NumberOfSamples,9)-OutputStates(1:NumberOfSamples,9));
% title('Y-Spd Error (m)');
% figure

% plot(ReferenceStates(1:NumberOfSamples,13)-OutputStates(1:NumberOfSamples,13));
% title('Yaw-Spd Error (m)');


figure
plot(StatesLog(1:NumberOfSamples,2),StatesLog(1:NumberOfSamples,3),...
                                          'LineWidth',2,'Color', 'b');

%%Plot Parking Spot
%--------------------------------------------------------------------------
% hold on
% rectangle('Position',[0,0,4,1],'FaceColor',[1 0 0],'EdgeColor','b',...
%     'LineWidth',2)
% hold on
% rectangle('Position',[6,0,3,1],'FaceColor',[1 0 0],'EdgeColor','b',...
%     'LineWidth',2)
% plot_x = 3:0.1:8.5;
% plot_y = zeros(length(plot_x));
% hold on 
% plot(plot_x,plot_y,'LineWidth',2,'Color', 'b');
% 
% hold on 
% th = 0:pi/100:pi;
% xunit = 10 * cos(th) + 5;
% yunit = 10 * sin(th) + 1.5;
% plot(xunit, yunit,'LineStyle','--','LineWidth',3,'Color', 'g');
% hold on
% straight_line_x = min(xunit):0.1:max(xunit);
% plot(straight_line_x,1.5*ones(length(straight_line_x)),'LineStyle','--',...
%     'LineWidth',0.5,'Color', 'g')
%--------------------------------------------------------------------------
%%

for i = 1:10:length(StatesLog)
hold on

Back_Right_Corner_T =   ...
             [cos(StatesLog(i,4))  -sin(StatesLog(i,4)); ...
             sin(StatesLog(i,4))  cos(StatesLog(i,4))]* ...
             [-0.5; -0.25];
Back_Right_Corner_X = StatesLog(i,2) + Back_Right_Corner_T(1);
Back_Right_Corner_Y = StatesLog(i,3) + Back_Right_Corner_T(2);
         

Front_Right_Corner_T =   ...
             [cos(StatesLog(i,4))  -sin(StatesLog(i,4)); ...
             sin(StatesLog(i,4))  cos(StatesLog(i,4))]* ...
             [0.5; -0.25];
Front_Right_Corner_X = StatesLog(i,2) + Front_Right_Corner_T(1);
Front_Right_Corner_Y = StatesLog(i,3) + Front_Right_Corner_T(2);

Front_Left_Corner_T =   ...
             [cos(StatesLog(i,4))  -sin(StatesLog(i,4)); ...
             sin(StatesLog(i,4))  cos(StatesLog(i,4))]* ...
             [0.5; 0.25];
Front_Left_Corner_X = StatesLog(i,2) + Front_Left_Corner_T(1);
Front_Left_Corner_Y = StatesLog(i,3) + Front_Left_Corner_T(2);

Back_Left_Corner_T =   ...
             [cos(StatesLog(i,4))  -sin(StatesLog(i,4)); ...
             sin(StatesLog(i,4))  cos(StatesLog(i,4))]* ...
             [-0.5; 0.25];
Back_Left_Corner_X = StatesLog(i,2) + Back_Left_Corner_T(1);
Back_Left_Corner_Y = StatesLog(i,3) + Back_Left_Corner_T(2);

Head_T =   ...
             [cos(StatesLog(i,4))  -sin(StatesLog(i,4)); ...
             sin(StatesLog(i,4))  cos(StatesLog(i,4))]* ...
             [0.7; 0];
Head_X = StatesLog(i,2) + Head_T(1);
Head_Y = StatesLog(i,3) + Head_T(2);

% 
% hold on 
% plot(StatesLog(i,2),StatesLog(i,3),'o');         
hold on
plot([Back_Right_Corner_X  Front_Right_Corner_X  Head_X ... 
     Front_Left_Corner_X Back_Left_Corner_X  Back_Right_Corner_X], ...
     [Back_Right_Corner_Y  Front_Right_Corner_Y  Head_Y ...
      Front_Left_Corner_Y Back_Left_Corner_Y  Back_Right_Corner_Y],...
         'Color','black','LineStyle','-', 'LineWidth',0.5);
end

% Enterance of Park Region -----------------------------------------------%
% strmin = ['Enterance of Park Region'];
% t = text(StatesLog(1,2)-2,StatesLog(1,3)+1,strmin);
% t(1).Color = 'red';
% t(1).FontSize = 14;
% ------------------------------------------------------------------------%

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

hold on 
xlabel('x position(m)')
ylabel('y position(m)')
%legend('Simulation', 'Optimal Path','WayPoints')
xlim([-1 20])
ylim([-3 5])

%% Plot 1

figure
plot(StatesLog_PP(1:NumberOfSamples_PP,2), ...
     StatesLog_PP(1:NumberOfSamples_PP,4), 'LineWidth', 2, 'Color', 'b');

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

figure
subplot(2,1,1)
plot(StatesLog(1:NumberOfSamples,1),180*StatesLog(1:NumberOfSamples,4)/pi,'LineWidth',2,'Color', 'b');
hold on
plot(Desired_Outputs(1:NumberOfSamples,1),180*Desired_Outputs(1:NumberOfSamples,3)/pi,'LineWidth',2,'Color','r')
grid on
xlabel('time(sec)')
ylabel('Yaw angle (deg)')
legend('Yaw angle of the Vehicle','Desired Yaw Angle')
ylim([-180 180])
subplot(2,1,2)
plot(StatesLog(1:NumberOfSamples,1),StatesLog(1:NumberOfSamples,5),'LineWidth',2,'Color', 'b');hold on;
plot(Desired_Outputs(1:NumberOfSamples,1),Desired_Outputs(1:NumberOfSamples,2),'LineWidth',2,'Color','r')
grid on
xlabel('time(sec)')
ylabel('Surge Speed (m/s)')
legend('Surge Speed of the Vehicle','Desired Surge Speed')


figure
subplot(2,1,1)
stairs(StatesLog(1:NumberOfSamples,1),...
       StatesLog(1:NumberOfSamples,5),'LineWidth',2);
legend('v_x in Body vs Time')
xlabel('Time(s)')
ylabel('v_x velocity in Body Frame(m/s)')
subplot(2,1,2)
grid on
stairs(StatesLog(1:NumberOfSamples,1),...
       StatesLog(1:NumberOfSamples,6),'LineWidth',2);
legend('v_y in Body vs Time')
xlabel('Time(s)')
ylabel('v_y velocity in Body Frame(m/s)')

%% Plot 2
figure
subplot(2,1,1)
stairs(StatesLog_PP(1:NumberOfSamples_PP,1),...
       StatesLog_PP(1:NumberOfSamples_PP,3),'LineWidth',2);
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
%%
% 
% figure
% plot(StatesLog(1:NumberOfSamples,1),StatesLog(1:NumberOfSamples,6),'LineWidth',2,'Color', 'b');
% grid on
% xlabel('time(sec)')
% ylabel('Sway Speed (m/s)')
% legend('Sway Speed of the Vehicle')
% 
% figure
% plot(StatesLog(1:NumberOfSamples,1),StatesLog(1:NumberOfSamples,7),'LineWidth',2,'Color', 'b'); hold on;
% grid on
% xlabel('time(sec)')
% ylabel('Yaw rate (rad/s)')
% legend('Yaw rate of the Vehicle')
% ylim([-pi/2 pi/2])

% figure
% stairs(Normalized_Distance(:,1),Normalized_Distance(:,2),'LineWidth',2);
% legend('Distance of Vehicle to Path')
% xlabel('Time(s)')
% ylabel('Distance')
% 
figure
subplot(2,1,1)
stairs(LThrusterForceXLog(:,1),LThrusterForceXLog(:,2),'LineWidth',2);
legend('Left thruster force(N)')
xlabel('Time(s)')
ylabel('Torque(N)')
subplot(2,1,2)
grid on
stairs(RThrusterForceXLog(:,1),RThrusterForceXLog(:,2),'r','LineWidth',2);
xlabel('Time(s)')
ylabel('Torque(N)')
legend('Right thruster force(N)')

figure
stairs(StatesLog_PP(1:NumberOfSamples_PP,1),...
       StatesLog_PP(1:NumberOfSamples_PP,Number_Of_Obstacles*4+8),'LineWidth',2);
legend('Quadrant vs Time')
xlabel('Time(s)')
ylabel('Quadrant')

% 
% 
% figure
% plot(Tot_Forces(1:NumberOfSamples,1),Tot_Forces(1:NumberOfSamples,2),'LineWidth',2,'Color', 'b'); 
% hold on
% plot(Dist_Forces(1:NumberOfSamples,1),Dist_Forces(1:NumberOfSamples,2),'LineWidth',2,'Color','r')
% grid on
% xlabel('time(sec)')
% ylabel('X Force')
% 
% figure
% plot(Tot_Forces(1:NumberOfSamples,1),Tot_Forces(1:NumberOfSamples,3),'LineWidth',2,'Color', 'b'); 
% hold on
% plot(Dist_Forces(1:NumberOfSamples,1),Dist_Forces(1:NumberOfSamples,3),'LineWidth',2,'Color','r')
% grid on
% xlabel('time(sec)')
% ylabel('Y Force')
% 
% figure
% plot(Tot_Forces(1:NumberOfSamples,1),Tot_Forces(1:NumberOfSamples,4),'LineWidth',2,'Color', 'b'); 
% hold on
% plot(Dist_Forces(1:NumberOfSamples,1),Dist_Forces(1:NumberOfSamples,4),'LineWidth',2,'Color','r')
% grid on
% xlabel('time(sec)')
% ylabel('N Force')

