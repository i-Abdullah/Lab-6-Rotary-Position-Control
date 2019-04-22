
%% Constants

clear
clc
close all;


thetad = input('Wanted Angle(Radians): ');
Kg = 33.3;
Km = 0.0401; %V/(rad/sec)
Rm = 19.2; %ohms
%Rigid Arm
J_hub = 0.0005; %Kgm^2
J_load = 0.0015; %Kgm^2
J = J_hub + J_load;

%% Get Matrix of Kp and Kd values
Kp = linspace(-2,50,6);
Kd = linspace(-1.0,1.5,6);

deg = zeros(49,644);
Time = zeros(49,644);
index = 1;
for i=1:length(Kd)
    for j = 1:length(Kp)
        %% Closed Loop Values
        n1 = Kp(j)*Kg*Km/(J*Rm);
        d2 = 1;
        d1 = (Kg^2*Km^2/(J*Rm) + Kd(i)*Kg*Km/(J*Rm));
        d0 = Kp(j)*Kg*Km/(J*Rm);
        
        %% Simulation
        %%% Closed Loop System
        num = n1;
        den = [d2 d1 d0];
        sysTF = tf(num,den);
        %%% Step Response
        [x,t] = step(sysTF);
        x = (thetad)*x;
%         figure(1);clf;
        Time(index,1:length(t)) = t';
        deg(index,1:length(x)) = x';
%         plot(t,x);
%         xlabel("Time");
%         ylabel("Angle \deg ");
%         hold on

        % The goal is to have less than 5% overshoot, and reach to 5% of
        % the steady state value within 0.15 seconds, so the values would
        % be stored in the following matrix:
        
        % what this's doing that it will define the threshold of settling
        % time to be when the value is 5% of the stead state value, now
        % you'll have to look manually into when that time is
        
        SystemInfo = stepinfo(sysTF,'SettlingTimeThreshold',0.05);
        
% store settling time and maximum over shoot, settling time again is
% already defined to be within 0.05 seconds!

SettleTime(i,j) = SystemInfo.SettlingTime; % each row represent 1 Kd value, each column Kp
OvershootValue(i,j) = SystemInfo.Overshoot;  % each row represent 1 Kd value, each column Kp



index = index + 1;

    end
end

%% Plot data

%{
figure;clf;
subplot(1,7,1)
hold on;
for k = 1:6
    plot(Time(k,:),deg(k,:));
end
hold off;
title("Kd = -1.5");
legend("Kp=-2","6.67","15.33","24","32.67","41.33","50");
xlabel("Time(sec)");
ylabel("Position(theta)");
ylim([-10,10]);

subplot(1,7,2)
hold on;
for k = 7:12
    plot(Time(k,:),deg(k,:));
end
hold off;
title("Kd = -1.0");
legend("Kp=-2","6.67","15.33","24","32.67","41.33","50");
xlabel("Time(sec)");
ylabel("Position(theta)");
ylim([-10,10]);

subplot(1,7,3)
hold on;
for k = 13:18
    plot(Time(k,:),deg(k,:));
end
hold off;
title("Kd = -0.5");
legend("Kp=-2","6.67","15.33","24","32.67","41.33","50");
xlabel("Time(sec)");
ylabel("Position(theta)");
ylim([-10,10]);

subplot(1,7,4)
hold on;
for k = 19:24
    plot(Time(k,:),deg(k,:));
end
hold off;
title("Kd = 0");
legend("Kp=-2","6.67","15.33","24","32.67","41.33","50");
xlabel("Time(sec)");
ylabel("Position(theta)");
ylim([-10,10]);

subplot(1,7,5)
hold on;
for k = 25:30
    plot(Time(k,:),deg(k,:));
end
hold off;
title("Kd = 0.5");
legend("Kp=-2","6.67","15.33","24","32.67","41.33","50");
xlabel("Time(sec)");
ylabel("Position(theta)");
ylim([-10,10]);

subplot(1,7,6)
hold on;
for k = 31:36
    plot(Time(k,:),deg(k,:));
end
hold off;
title("Kd = 1.0");
legend("Kp=-2","6.67","15.33","24","32.67","41.33","50");
xlabel("Time(sec)");
ylabel("Position(theta)");
ylim([-10,10]);

subplot(1,7,7)
hold on;
for k = 37:42
    plot(Time(k,:),deg(k,:));
end
hold off;
title("Kd = 1.5");
legend("Kp=-2","6.67","15.33","24","32.67","41.33","50");
xlabel("Time(sec)");
ylabel("Position(theta)");
ylim([-10,10]);




%}


%% plot 2:

%{
figure;
for i = 1:length(Kd) 
    
    for j = 1:length(Kp) 
        subplot(1,length(Kd),i);
        Time_plot = Time(((i-1)*length(Kp))+j,:);
        deg_plot = deg(((i-1)*length(Kp))+j,:);
        
        zero_index = find(Time_plot==0);
        Time_plot(zero_index(2:end)) = [];
        
        deg_plot(zero_index(2:end)) = [];
        
        plot(Time_plot,deg_plot) ;
        hold on;
        ylim([-1,1]);
        xlim([0,1]);
        
        
        
    end
    
  title(['Kd =' num2str(Kd(i))])
  legend([ num2str(Kp(1)) ],[ num2str(Kp(2)) ],[ num2str(Kp(3)) ],[ num2str(Kp(4)) ],[ num2str(Kp(5)) ],[ num2str(Kp(6)) ] );
  

end

%}
%% plot 3:

figure;
for i = 1:length(Kd) 
    
    for j = 1:length(Kp) 
        
        figure(i)
        
        Time_plot = Time(((i-1)*length(Kp))+j,:);
        deg_plot = deg(((i-1)*length(Kp))+j,:);
        
        % clean the zeros:
        % clean everything but first index
        zero_index = find(Time_plot==0);
        Time_plot(zero_index(2:end)) = [];
        
        deg_plot(zero_index(2:end)) = [];
        
        plot(Time_plot,deg_plot,'LineWidth',1.5) ;
        hold on;
        ylim([-thetad-thetad*0.5 ,thetad+thetad*0.9]);
        xlim([0,1]);
        
        
        
    end
   
  % plot overshoot bounds
  
  Errorbound = 0.05*thetad*(ones(1,10));
  ErrorTime = linspace(0,1,10);
  
  plot(ErrorTime,Errorbound+thetad,'*-','Color',[0.7 0.7 0.7],'LineWidth',1)
  plot(ErrorTime,thetad-Errorbound,'*-','Color',[0.7 0.7 0.7],'LineWidth',1)

  title(['Kd =' num2str(Kd(i))])
  grid minor
  xlabel('Time (s)')
  ylabel(' \theta rad')
  legend([ 'Kp=' num2str(Kp(1)) ],[ 'Kp=' num2str(Kp(2)) ],[ 'Kp=' num2str(Kp(3)) ],[ 'Kp=' num2str(Kp(4)) ],[ 'Kp=' num2str(Kp(5)) ],[ 'Kp=' num2str(Kp(6)) ] ,'5% Error bounds');
  hold on

end


%% printout tbale with results

% prepeare inputs to table

Kd_table = [ ones(1,length(Kp))*Kd(1)...
    ones(1,length(Kp))*Kd(2)...
    ones(1,length(Kp))*Kd(3)...
    ones(1,length(Kp))*Kd(4)...
    ones(1,length(Kp))*Kd(5)...
    ones(1,length(Kp))*Kd(6)...
    ];

Kp_table = [ Kp...
    Kp...
    Kp...
    Kp...
    Kp...
    Kp...
    ];

Settle_table =  [ SettleTime(:,1)'...
    SettleTime(:,2)'...
    SettleTime(:,3)'...
    SettleTime(:,4)'...
    SettleTime(:,5)'...
    SettleTime(:,6)'...
    ];

Overshoot_table =  [ OvershootValue(:,1)'...
    OvershootValue(:,2)'...
    OvershootValue(:,3)'...
    OvershootValue(:,4)'...
    OvershootValue(:,5)'...
    OvershootValue(:,6)'...
    ];


Table_results = table(Kd_table',Kp_table',Settle_table',Overshoot_table',...
    'VariableNames',{'Kd','Kp','Settle_time','Overshoot'})



fprintf('NOTE :')

fprintf( '\n' )
fprintf( '\n' )


fprintf('with 5%% overshoot allowed you are limited to overshoot values of: \n')

fprintf( [num2str(thetad) char(177) num2str(thetad.*0.05) ] )

fprintf( '\n' )


fprintf('Error bound on settling time is 5%% ')

fprintf( '\n' )
