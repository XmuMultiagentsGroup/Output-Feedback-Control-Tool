clear;clc;close all;
f = 100;
HoldType = 'zoh';
SystemType = 'Continuous'; % or 'Discrete'
PolesType = 'Continuous'; % or 'Discrete'
% D_K = DTsystem(A,B,C,SystemType,FinalPoles,PolesType,SensorNoiseVariance,T,HoldType);


A = [0 1 0;0 -0.1 60;0 -1.4 -50];
B = [0 ;0 ;10];
C = [1 0 0];D = 0;
ObserverPoles = [-80+10*1i;-80-10*1i;-90];
RealPolesFromNotes = [-10+5*1i;-10-5*1i;-80];

%% Class Example Gain Margin and Phase Margin Calculation
FinalPoles = RealPolesFromNotes;
[Aclass,Bclass,Cclass,Dclass,K_class,L_class,CL_System_class,Gm_class,Pm_class,StepResponse_class] = FBcontrol(A,B,C,D,...
    'FeedbackType','OutputFB',...
    'ObserverPoles',ObserverPoles,...
    'SystemType','Continuous',...
    'FinalPoles',FinalPoles,...
    'Frequency',100);

%% Tracing over the performance in the nearby region by placing controller poles
len1_min = 2;len1_max= 20;
len2_min = 80;len2_max = 100;
ResponseTrace = [];
Systems = [];
counter = 1;
%for loop1 = len1_min:1:len1_max
for loop2 = len2_min:1:len2_max
    for loop1 = len1_min:1:len1_max
        FinalPoles = [-loop1+5*1i;-loop1-5*1i;-loop2];
        %FinalPoles = RealPolesFromNotes;
        
        [Af,Bf,Cf,Df,K,L,CL_System,Gm,Pm,StepResponse] = FBcontrol(A,B,C,D,...
            'FeedbackType','OutputFB',...
            'ObserverPoles',ObserverPoles,...
            'SystemType','Continuous',...
            'FinalPoles',FinalPoles,...
            'Frequency',100);
        ResponseTrace(counter,:) = [FinalPoles' StepResponse.RiseTime Gm Pm K];
        Systems{counter} = CL_System;
        counter = counter + 1;
        if(mod(counter,20)==0)
            sprintf('%f percent search complete \n',(counter/400)*100)
        end        
    end
end
% Analysis of a few trends of parameters
vector1 = ResponseTrace(:,4); % Rise Time
vector2 = ResponseTrace(:,5)/45; % Gain Margin
vector3 = ResponseTrace(:,7)/200; % K only one value of the 3-vector
plot(vector1);
hold on;
plot(vector2);
hold on;
plot(vector3);
title('Quantitative Comparison of Rise Time,Gain Margin,Feedback Gain over different pole placements');
legend('Rise Time','Gain Margin','Feedback Gain(1)');


% Finding if there exists a better pole placement for the given system

zz = [ResponseTrace(:,5) ResponseTrace(:,6)]; %gain margins and phase margins

comparison = zz > repmat([Gm_class,Pm_class],size(zz,1),1);
% By better I mean the one with both better Gm and Pm
pos_with_better_performances = sum(zz > repmat([Gm_class,Pm_class],size(zz,1),1),2) == 2;
better_pole_placements = ResponseTrace(pos_with_better_performances,1:3);


%On careful observation the value of third pole is not affecting the gain
%and phase margin much as it is far away from the origin.
% Dominating pole are [-4 - 5i,-4 + 5i,alpha], and
Better_Poles = [-3 - 5i,-3 + 5i,80];
display(Better_Poles);

% The dominating pole is the one with complex conjugate pair
% I might would have changed the imaginari part of this pole that might
% would have given me a varied response but to keep things simple I am
% settling down with tracing over only two variables
 

% Picking up the first one of them all to see the performance comparison
index_better = pos_with_better_performances;

CL_System_improved = Systems{index_better};
%CL_System_class 

figure;
step(CL_System_improved);hold on;
step(CL_System_class);
legend('Improved Margin System','Class System');  

P = bodeoptions();P.XLim = [1 1000];
figure;
bode(CL_System_improved,P);hold on;
bode(CL_System_class,P);grid on;
legend('Improved Margin System','Class System');  

Class_BW = bandwidth(CL_System_class)
New_BW = bandwidth(CL_System_improved)

display('We can see that we are able to improve the gain and phase margins of the system but at the cost of BandWidth of the system');




% Other Examples on ways to run the code

% p = FB_control(A,B,C,D,...
%     'FeedbackType','OutputFB',...
%     'ObserverPoles',NaN,...
%     'ObserverType','KalmanObserver',...
%     'SystemType','Continuous',...
%     'FinalPoles',FinalPoles,...
%     'Frequency',220);

% p = FB_control(A,B,C,D,...
%     'FeedbackType','OutputFB',...
%     'ObserverPoles',NaN,...
%     'ObserverType','KalmanObserver',... % There is some problem for now
%     with the KalmanObserver, please don't use it
%     'SystemType','Continuous',...
%     'FinalPoles',FinalPoles,...
%     'Frequency',220,...
%     'HoldType','foh',...
%     'PolesType','Continuous');

% SensorNoiseVariance = p.Results.SensorNoiseVariance;
% FeedbackType = p.Results.FeedbackType;
% ObserverType = p.Results.ObserverType;
% SystemType = p.Results.SystemType;
% HoldType = p.Results.HoldType;
% Frequency = p.Results.Frequency;
% PolesType = p.Results.PolesType;

