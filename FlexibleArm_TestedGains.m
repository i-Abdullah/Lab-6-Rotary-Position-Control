%% info:

%  flexible arm dynamics can be modeled as the rigid arm
%  (a spring-mass-damper system) plus an additional spring-mass- damper



%% housekeepign:

clear
clc
close all


%% ask for input

% desired vlues
thetad = input('Wanted Angle(Radians): ');
dispd = input('Wanted displacement(meter): ');

SetTime = 1;
%% closed loop transfer constants:

Kg = 33.3 ; % no units
Km = 0.0401 ; %V/(rad/sec);
Rm = 19.2 ; % Ohms

Jhub = 0.0005;
Jload = 0.0015;
J = Jhub+Jload ;
LinkLength = 0.45;
Marm = 0.06 ; %kg
Jarm = 0.004 ; %kgm^2
Mtip = 0.05 ; %kg
JM = 0.01;
fc = 1.8 ; %HZ

JL = Jarm + JM;

Karm = (2*pi*fc)^2 *(JL) ;

% note:
% K1 = Kp?
% K2 = Kpd
% K3 = KD?
% K4 = KDd

% use actual gains applied using data collected for error analysis

trial1 = load('Data/Group18_trail1_flex_TG') ;
trial2 = load('Data/Group18_trail2_flex_TG') ;

K1 = [ mean(trial1(:,8)) mean(trial2(:,8)) ];
K2 = [ mean(trial1(:,9)) mean(trial2(:,9)) ];
K3 = [ mean(trial1(:,10)) mean(trial2(:,10)) ];
K4 = [ mean(trial1(:,11)) mean(trial2(:,11)) ];

% definitions in lab docuement:


p1 = - (Kg^2 * Km^2) / (Jhub*Rm) ;
p2 = (Kg^2 * Km^2 * LinkLength ) / (Jhub*Rm) ;

q1 = Karm / ( LinkLength*Jhub) ;
q2 = - ( (Karm*( Jhub + JL )) / (JL*Jhub)) ;

r1 = (Kg*Km)/(Jhub*Rm) ;
r2 = - (Kg*Km*LinkLength)/(Jhub*Rm);

% l = lambda;


% The angle of the arm (or output shaft) is denoted by, theta_L



% Closed Loop System

index = 1;

for j=1:length(K1);
    
        %
        
l3 = -p1 + K3(j).*r1 + K4(j).*r2 ;
l2 = -q2 + K1(j).*r1 + K2(j).*r2 + K4(j).*(p2*r1 - r2*p1) ;
l1 = p1*q2 - q1*p2 + K3(j).*(q1*r2 - r1*q2) + K2(j).*(p2*r1 - r2*p1) ;
l0 = K1(j).*(q1*r2 - r1*q2);

num_angle = K1(j) .* [ r1 0 (q1*r2 - r1*q2) ] ;
den_angle = [1 l3 l2 l1 l0];

% disp = displacement of the tip.

num_disp = K1(j) .* [ r2 (r1*p2 - p1*r2)  0];
den_disp = [1 l3 l2 l1 l0];

System_angle = tf(num_angle,den_angle);

System_disp = tf(num_disp,den_disp);

[ TipeAngle t_angle ] = step(System_angle); % apply unit step, the response is the same for magnitude
% that's bigger than 1 so you just scale it.

[ TipDisplacement t_disp ] = step(System_disp); 


% repeat the unit input
TipeAngle = (thetad)*(TipeAngle);
TipDisplacement = thetad*TipDisplacement;


Time_angle(index,1:length(t_angle)) = t_angle';
deg(index,1:length(TipeAngle)) = TipeAngle';

Time_disp(index,1:length(t_disp)) = t_disp';
disp(index,1:length(TipDisplacement)) = TipDisplacement';


% The goal is to have less than 5% overshoot, and reach to 5% of
% the steady state value within 0.15 seconds, so the values would
% be stored in the following matrix:
        
% what this's doing that it will define the threshold of settling
% time to be when the value is 5% of the stead state value, now
% you'll have to look manually into when that time is
        
        
% store settling time and maximum over shoot, settling time again is
% already defined to be within 0.05 seconds!

%SettleTime(i,j) = SystemInfo.SettlingTime; % each row represent 1 Kd value, each column Kp
%OvershootValue(i,j) = SystemInfo.Overshoot;  % each row represent 1 Kd value, each column Kp

SystemInfo_angle = stepinfo(System_angle,'SettlingTimeThreshold',0.05);
SettleTime_angle(index,1) = SystemInfo_angle.SettlingTime; % each row represent 1 Kd value, each column Kp
OvershootValue_angle(index,1) = SystemInfo_angle.Overshoot;  % each row represent 1 Kd value, each column Kp

SystemInfo_disp = stepinfo(System_disp,'SettlingTimeThreshold',0.05);
SettleTime_disp(index,1) = SystemInfo_disp.SettlingTime; % each row represent 1 Kd value, each column Kp
OvershootValue_disp(index,1) = SystemInfo_disp.Overshoot;  % each row represent 1 Kd value, each column Kp

index = index + 1;
    
end


%% Experimental data:


% adjust the data
%t1 = trial 1

% get time
trial1_time = trial1(:,1)*10^(-3);
trial2_time = trial2(:,1)*10^(-3);

% zero time based on the first index

trial1_time = trial1_time(:) - trial1_time(1);
trial2_time = trial2_time(:) - trial2_time(1);


% shift all the measured tip angle from -input/2 to +input/2 
% so it goes from 0 to input/2;

% the + thetad/2 is to shift the data up by the input.
% the hub goes from negative input/2 to +input/2

trial1_angle = trial1(:,2) + abs(min(trial1(:,2)));
trial2_angle = trial2(:,2)+ abs(min(trial2(:,2)));

% git tip displacement
trial1_tip = trial1(:,3);
trial2_tip = trial2(:,3);


%% dissect data



% - - - - - - - - - ( Trial 1 ) - - - - - - - - - - - - - -

figure(3)

plot(trial1_time,trial1_angle);
pts = ginput(2);
%info = getrect(figure(3));

% find closest time so you can cut from there

idx = knnsearch([trial1_time trial1_angle],pts);

trial1_time = trial1_time(idx(1):idx(2));
trial1_angle = trial1_angle(idx(1):idx(2)); % measured change in angle
measureDisp_t1 = trial1_tip(idx(1):idx(2)); % measured change in tip displacement
% zero time
trial1_time = trial1_time - trial1_time(1);

% get releated tip angle
trial1_tip = trial1_tip(idx(1):idx(2));


% - - - - - - - - - ( end of tiral 1 ) - - - - - - - - - - - -


% - - - - - - - - - ( Trial 2 ) - - - - - - - - - - - - - -

figure(3)

plot(trial2_time,trial2_angle);
pts = ginput(2);
%info = getrect(figure(3));
% find closest time so you can cut from there

idx = knnsearch([trial2_time trial2_angle],pts);

trial2_time = trial2_time(idx(1):idx(2));
trial2_angle = trial2_angle(idx(1):idx(2));

% zero time
trial2_time = trial2_time - trial2_time(1);

% get releated tip angle
trial2_tip = trial2_tip(idx(1):idx(2));


% - - - - - - - - - ( end of tiral 2 ) - - - - - - - - - - - -


% store expermintal data again to be plotted:

time_exp = { trial1_time trial2_time };
angle_exp = { trial1_angle trial2_angle };
tip_exp = { trial1_tip trial2_tip };


%% plot

figure;

for j = 1
    
for i = 1:length(K1) 
    
        figure(j)
        
        Time_plot_angle = Time_angle(i,:);
        angle_plot = deg(i,:);
        
        % clean the zeros:
        % clean everything but first index
        zero_index = find(Time_plot_angle==0);
        Time_plot_angle(zero_index(2:end)) = [];
        
        angle_plot(zero_index(2:end)) = [];

	        plot(Time_plot_angle,angle_plot,'LineWidth',1.5,...
            'DisplayName',[' K1 = ' num2str(K1(i)) ' K2 = ' num2str(K2(i))...
            ' K3 = ' num2str(K3(i)) ' K4 = ' num2str(K4(i))]);
        hold on
        	plot(time_exp{:,i},angle_exp{:,i},'LineWidth',1.5,...
            'DisplayName',['Experimental data for:' ' K1 = ' num2str(K1(i)) ' K2 = ' num2str(K2(i))...
            ' K3 = ' num2str(K3(i)) ' K4 = ' num2str(K4(i))]);

        hold on;
        
        
     
        
        
end

        ylim([-thetad-thetad*0.5 ,thetad+thetad*0.9]);
        ylim([-0.1 ,0.7]);

        xlim([0,5]);
        title('Flexible arm angle')
        Errorbound = 0.05*thetad*(ones(1,20));
        ErrorTime = linspace(0,6,20);
  
	  plot(ErrorTime,Errorbound+thetad,'*-','Color',[0.7 0.7 0.7],'LineWidth',1,'DisplayName','Upper error bound of 5%')
	  plot(ErrorTime,thetad-Errorbound,'*-','Color',[0.7 0.7 0.7],'LineWidth',1,'DisplayName','Lower error bound of 5%')
	
plot([SetTime , SetTime],[-0.1 0.7],'*-','Color',[0.25, 0.25, 0.25],'DisplayName',['Maximum Allowed settle time' num2str(SetTime) 's' ],'LineWidth',2)


  grid minor
  
  xlabel('Time (s)')
  ylabel(' \theta rad')
  %legend([ 'K1=' num2str(K1(1)) ],[ 'K1=' num2str(K1(2)) ])%,[ 'K1=' num2str(K1(3)) ],[ 'K1=' num2str(K1(4)) ],[ 'K1=' num2str(K1(5)) ],'5% Error bounds');


legend('show','Location','SouthEast')


end

saveas(figure(1),'FlexableAngle.jpg')

% do the plot for tip displacement

for j = 1
    
for i = 1:length(K1) 
    
        figure(j+1)
        
        Time_plot_disp = Time_disp(i,:);
        disp_plot = disp(i,:);
        
        % clean the zeros:
        % clean everything but first index
        zero_index = find(Time_plot_disp==0);
        Time_plot_disp(zero_index(2:end)) = [];
        
        disp_plot(zero_index(2:end)) = [];
        
	        plot(Time_plot_disp,disp_plot,'LineWidth',2,...
            'DisplayName',[' K1 = ' num2str(K1(i)) ' K2 = ' num2str(K2(i))...
            ' K3 = ' num2str(K3(i)) ' K4 = ' num2str(K4(i))]);
        hold on
        	plot(time_exp{:,i},tip_exp{:,i},'LineWidth',0.8,...
            'DisplayName',['Experimental data for:' ' K1 = ' num2str(K1(i)) ' K2 = ' num2str(K2(i))...
            ' K3 = ' num2str(K3(i)) ' K4 = ' num2str(K4(i))]);

        hold on;


        
        
end

        %ylim([-dispd-dispd*0.5 ,dispd+dispd*0.9]);
        xlim([0,5]);
        title('Flexible arm tip displacement')
        Errorbound = 0.1*dispd*(ones(1,20));
        ErrorTime = linspace(0,6,20);
  
  plot(ErrorTime,Errorbound+dispd,'*-','Color',[0.7 0.7 0.7],'LineWidth',1,'DisplayName','Upper error bound of 10%')
  plot(ErrorTime,dispd-Errorbound,'*-','Color',[0.7 0.7 0.7],'LineWidth',1,'DisplayName','Upper error bound of 10%')
grid minor
  xlabel('Time (s)')
  ylabel(' Tip displacement (m)')
  %legend([ 'K1=' num2str(K1(1)) ],[ 'K1=' num2str(K1(2)) ])%,[ 'K1=' num2str(K1(3)) ],[ 'K1=' num2str(K1(4)) ],[ 'K1=' num2str(K1(5)) ],'5% Error bounds');
legend('show','Location','SouthEast')
   
end


saveas(figure(2),'FlexableTip.jpg')

%% printout tbale with results

% prepeare inputs to table

K1_table = [ K1...
    %ones(1,length(K1))*K1(2)...
    %ones(1,length(K1))*K1(3)...
    %ones(1,length(K1))*K1(4)...
    %ones(1,length(K1))*K1(5)...
    ];

K2_table = [ K2...
    %ones(1,length(K2))*K2(2)...
    %ones(1,length(K2))*K2(3)...
    %ones(1,length(K2))*K2(4)...
    %ones(1,length(K2))*K2(5)...
    ];

K3_table = [ K3...
    %ones(1,length(K3))*K3(2)...
    %ones(1,length(K3))*K3(3)...
    %ones(1,length(K3))*K3(4)...
    %ones(1,length(K3))*K3(5)...
    ];

K4_table = [ K4...
    %ones(1,length(K4))*K4(2)...
    %ones(1,length(K4))*K4(3)...
    %ones(1,length(K4))*K4(4)...
    %ones(1,length(K4))*K4(5)...
    ];


Settle_table_angle =  [ SettleTime_angle' ];

Overshoot_table_angle =  [ OvershootValue_angle' ];

Settle_table_disp =  [ SettleTime_disp' ];

Overshoot_table_disp =  [ OvershootValue_disp' ];




Table_results = table(K1_table',K2_table',K3_table',K4_table',Settle_table_angle',Overshoot_table_angle',...
    Settle_table_disp',Overshoot_table_disp',...
    'VariableNames',{'K1','K2','K3','K4','Settle_time_angle','Overshoot_angle','Settle_time_disp','Overshoot_disp'})







