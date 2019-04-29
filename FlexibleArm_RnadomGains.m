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
	
	K1 = linspace(0,20,5);
	K2 = linspace(-50,0,5);
	K3 = linspace(0,1.5,5);
	K4 = linspace(0,1.5,5);

%     K1 = linspace(0,20,5);
% 	K2 = linspace(-50,0,5);
% 	K3 = linspace(0,0,5);
% 	K4 = linspace(0,0,5);

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
	
	%% plot
	
	figure;
	
	for j = 1
	
	for i = 1:length(K1) 
	
	        figure(j)
	
	        Time_plot = Time_angle(i,:);
	        deg_plot = deg(i,:);
	
	        % clean the zeros:
	        % clean everything but first index
	        zero_index = find(Time_plot==0);
	        Time_plot(zero_index(2:end)) = [];
	
	        deg_plot(zero_index(2:end)) = [];
	
	        plot(Time_plot,deg_plot,'LineWidth',1.5,...
            'DisplayName',[' K1 = ' num2str(K1(i)) ' K2 = ' num2str(K2(i))...
            ' K3 = ' num2str(K3(i)) ' K4 = ' num2str(K4(i))]);
	        hold on;
	
	
	
	
	end
	
	        ylim([-thetad-thetad*0.5 ,thetad+thetad*0.9]);
	        xlim([0,5]);
	        title('Flexible arm angle')
	        Errorbound = 0.05*thetad*(ones(1,20));
	        ErrorTime = linspace(0,6,20);
	
	  plot(ErrorTime,Errorbound+thetad,'*-','Color',[0.7 0.7 0.7],'LineWidth',1,'DisplayName','Upper error bound of 5%')
	  plot(ErrorTime,thetad-Errorbound,'*-','Color',[0.7 0.7 0.7],'LineWidth',1,'DisplayName','Lower error bound of 5%')
	
	  xlabel('Time (s)')
	  ylabel(' \theta rad')
	  %legend([ 'K1=' num2str(K1(1)) ],[ 'K1=' num2str(K1(2)) ])%,[ 'K1=' num2str(K1(3)) ],[ 'K1=' num2str(K1(4)) ],[ 'K1=' num2str(K1(5)) ],'5% Error bounds');
	legend('show','Location','SouthEast');
    %grid minor
    ylim([ -thetad+(thetad/1.3) thetad+(thetad/2)])
	
	end
	
	% do the plot for tip displacement
	
	for j = 1
	
	for i = 1:length(K1) 
	
	        figure(j+1)
	
	        Time_plot = Time_disp(i,:);
	        disp_plot = disp(i,:);
	
	        % clean the zeros:
	        % clean everything but first index
	        zero_index = find(Time_plot==0);
	        Time_plot(zero_index(2:end)) = [];
	
	        disp_plot(zero_index(2:end)) = [];
	
	        plot(Time_plot,disp_plot,'LineWidth',1.5...
            ,'DisplayName',[' K1 = ' num2str(K1(i)) ' K2 = ' num2str(K2(i))...
            ' K3 = ' num2str(K3(i)) ' K4 = ' num2str(K4(i))]) ;
	        hold on;
	
	
	
	
	end
	
	        %ylim([-dispd-dispd*0.5 ,dispd+dispd*0.9]);
	        xlim([0,5]);
	        title('Flexible arm tip displacement')
        Errorbound = 0.01*(ones(1,20));
        ErrorTime = linspace(0,6,20);
  
  plot(ErrorTime,Errorbound,'*-','Color',[0.7 0.7 0.7],'LineWidth',1,'DisplayName','Upper error bound of +0.01 m')
  plot(ErrorTime,-Errorbound,'*-','Color',[0.7 0.7 0.7],'LineWidth',1,'DisplayName','Lower error bound of -0.01 m')
	
	  xlabel('Time (s)')
	  ylabel(' Tip displacement (m)')
	  %legend([ 'K1=' num2str(K1(1)) ],[ 'K1=' num2str(K1(2)) ])%,[ 'K1=' num2str(K1(3)) ],[ 'K1=' num2str(K1(4)) ],[ 'K1=' num2str(K1(5)) ],'5% Error bounds');
	legend('show','Location','SouthEast')
	
	end
	
	
	
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
	
	
	
% 	fprintf('NOTE :')
% 	
% 	fprintf( '\n' )
% 	fprintf( '\n' )
% 	
% 	
% 	fprintf('with 10%% overshoot allowed you are limited to overshoot values of: \n')
% 	
% 	fprintf( [num2str(thetad) char(177) num2str(thetad.*0.1) ] )
% 	
% 	fprintf( '\n' )
% 	
% 	
% 	fprintf('Error bound on settling time is 5%% ')
% 	
% 	fprintf( '\n' )
% 	
