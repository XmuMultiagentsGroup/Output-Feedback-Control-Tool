%% Programmer : Manish Sharma
%  Email id   : sharm209@purdue.edu
%  To get help on this function type "help FB_control"
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
%     'ObserverType','KalmanObserver',...
%     'SystemType','Continuous',...
%     'FinalPoles',FinalPoles,...
%     'Frequency',220,...
%     'HoldType','foh',...
%     'PolesType','Continuous');
% Other parameters that can be initialized
% SensorNoiseVariance = p.Results.SensorNoiseVariance;
% FeedbackType = p.Results.FeedbackType;
% ObserverType = p.Results.ObserverType;
% SystemType = p.Results.SystemType;
% HoldType = p.Results.HoldType;
% Frequency = p.Results.Frequency;
% PolesType = p.Results.PolesType;
% There is some problem for now with the KalmanObserver, please don't use it


function [A,B,C,D,K,L,CL_System,Gm,Pm,StepResponse] =  FBcontrol(input,varargin)

%% Setting properties of input parser
p = inputParser;
p.FunctionName = 'FB_control';
p.CaseSensitive = 1;
p.KeepUnmatched = 0;

%% Reading the inputs
p.addRequired('A',@ValidateA);
p.addRequired('B',@ValidateB);
p.addRequired('C',@ValidateC);
p.addRequired('D',@ValidateD);

p.addParameter('FinalPoles',[],@ValidateFinalPoles)
p.addParameter('ObserverPoles',NaN);

p.addParameter('SensorNoiseVariance',NaN);
p.addParameter('FeedbackType','StateFB',@ValidateFBType); %StateFB or OutputFB
p.addParameter('ObserverType','SimpleObserver',@ValidateOBSType); %SimpleObserver, KalmanObserver
p.addParameter('SystemType','Discrete',@ValidateSystemType); %Discrete, Continuous
p.addParameter('PolesType',NaN,@ValidatePolesType); %Copy the SystemType if nothing is passed
p.addParameter('HoldType','zoh',@ValidateHoldType); %Copy the SystemType if nothing is passed
p.addParameter('Frequency',-1,@ValidateFrequency); %Copy the SystemType if nothing is passed
p.parse(input,varargin{:})

%% Verifying the dimensions of all the inputs are all matching or not
A = p.Results.A;
B = p.Results.B;
C = p.Results.C;
D = p.Results.D;

FinalPoles = p.Results.FinalPoles;
ObserverPoles = p.Results.ObserverPoles;

SensorNoiseVariance = p.Results.SensorNoiseVariance;
FeedbackType = p.Results.FeedbackType;
ObserverType = p.Results.ObserverType;
SystemType = p.Results.SystemType;
HoldType = p.Results.HoldType;
Frequency = p.Results.Frequency;
PolesType = p.Results.PolesType;
% If poles type is not defined assume it to be same as system type
if(isnan(PolesType)) 
    PolesType = p.Results.SystemType; 
end

% Varifying if the A and B matrices are compatible or not in size
n = size(A,1); % dimension of state vector
if(size(B,1) ~= size(A,1))
    error('System Matrix and Input Gain Matrix are not compatible in size')
end
m = size(B,2);  % dimension of output vector

% Varifying if Output Gain Matrix is compatible with the FeedbackType and
% A, B matrices or not
if(strcmp(FeedbackType,'OutputFB'))
    if(isnan(C))
        error('For output FB system you need to initialize "Output Gain Matrix yourself"');
    elseif(size(C,2)~=n)
        error('Size of Output Gain Matrix not matching, need to have n columns');
    end
else
    % only else condition is required as the program can only allow one of
    % the two values "StateFB" or "OutputFB"
    if(strcmp(FeedbackType,'StateFB'))
        if(~isnan(C) || C~= eye(n))
            error('For stateFB system initialize C with Identity of size n, or with NaN');
        end
        if(isnan(C))
            C = eye(n);
        end
    end
end
k = size(C,1); % output vector dimension
if(size(D,1) ~= k || size(D,2) ~= m)
    error('Size of D not compatible with other system matrices');
end

% SimpleObserver allow you to place poles for the observer and
% KalmanObserver does this job for you
if(strcmp(ObserverType,'SimpleObserver'))
    if(isnan(ObserverPoles))
        error('For simple Observer you need to initialize "Observer Poles", you may allow me to do that for you by choosing "KalmanObserver"');
    elseif(size(ObserverPoles,1)~=n || size(ObserverPoles,2)~=k)
        error('Size of Observer Gain Matrix not matching, need to have n rows and k columns where k is dimension of output vector and n is dimension of input vector');
    end
else %only else condition is required as the program can only allow one of the two values "StateFB" or "OutputFB"
    % OutputGainMatrix, nothing to be done here as we will generate
    % Kalman gain in the controller design functions
end

% Varifying the dimensions of sensor noise variance
if(~isnan(SensorNoiseVariance))
    if(size(SensorNoiseVariance,1)~=k)
        error('Dimension of Sensor Noise Variance Vector is not matching to k ');
    end
end
%Matching the dimension of number of poles placed to number of poles that
%should be placed
if(length(FinalPoles)~=n)
    error('Number of poles to be placed should be equal to the dimension of state vector');
end

if(Frequency == -1)
    % Check if the system is purely discrete
    if(~strcmp(PolesType,'Discrete') || ~strcmp(SystemType,'Discrete'))
        error('Frequncy is required for systems that are not purely discrete');
    end
end
    
% Will update the basic properties of fed pole later
% if(strcmp(SystemType,'Discrete'))
%     if(sum(abs(FinalPoles)>1)>0)
%         display('I dont think that you want to design a system that is unstable, try to place poles inside unit circle for DT systems');
%     end
% else
%     if(sum(real(FinalPoles)>0)>0)
%         error('Designed System is unstable, try to place poles on left hand side for CT systems');
%     elseif(sum(real(FinalPoles)==0)>1)
%
%     end
% end

%% Checking for controllability and Observability and stability
% These checks are more stringent than required on the system,

if(rank(ctrb(A,B))<size(A,1))
    error('System is not Controllable');
end
if(rank(obsv(A,C))<size(A,2))
    error('System is not Observable');
end


%% rest goes here
% estim(),kalman()
% all the outputs will be generated in DT
% D_K = DTsystem(A,B,C,SystemType,FinalPoles,PolesType,SensorNoiseVariance,T,HoldType);

% Warning: The closed-loop system is unstable.

[A,B,C,D,K,L,CL_System,Gm,Pm,StepResponse] = DTsystem(A,B,C,D,SystemType,FinalPoles,ObserverPoles,PolesType,ObserverType,SensorNoiseVariance,Frequency,HoldType);


% 2 - see if the estimator is working fine
% 3 - insert some noise
% 4 - Make State FB controller
% 5 - Make Output FB controller


% Call this from the panel
% p = FB_control(rand(3,3),rand(3,2),'SystemType','Discrete')
end


%% All the input validation functions go here
function y = ValidateA(A)
y = (size(A,1) == size(A,2)) && ~isempty(A);
if(~y)
    error('System Matrix should be a square matrix of size greater than 0')
end
end
function y = ValidateB(X)
y = ~isempty(X);
if(~y)
    error('Input Gain Matrix should not be empty')
end
end
function y = ValidateC(X)
y = ~isempty(X);
if(~y)
    error('Output Gain Matrix should not be empty')
end
end
function y = ValidateD(X)
y = ~isempty(X);
if(~y)
    error('Output Observer Gain Matrix should not be empty')
end
end
function y = ValidateFBType(Feedback)
y = strcmp(Feedback,'StateFB') | strcmp(Feedback,'OutputFB');
if(~y)
    error('Feedback can have only two type "StateFB" or "OutputFB"')
end
end
function y = ValidateOBSType(Observer)
y = strcmp(Observer,'KalmanObserver') | strcmp(Observer,'SimpleObserver');
if(~y)
    error('Observer can have only two types "SimpleObserver" or "KalmanObserver"')
end
end
function y = ValidateSystemType(SystemType)
y = strcmp(SystemType,'Discrete') | strcmp(SystemType,'Continuous');
if(~y)
    error('System can have only two types "Discrete" or "Continuous"')
end
end
function y = ValidatePolesType(PolesType)
y = strcmp(PolesType,'Discrete') | strcmp(PolesType,'Continuous');
if(~y)
    error('Poles can have only two types "Discrete" or "Continuous"')
end
end
function y = ValidateSensorNoiseVariance(SensorNoiseVariance)
y = ~isempty(SensorNoiseVariance);
if(~y)
    error('SensorNoiseVariance can not be empty')
end
end
function y = ValidateFinalPoles(FinalPoles)
y = ~isempty(FinalPoles);
if(~y)
    error('SensorNoiseVariance can not be empty')
end
end
function y = ValidateHoldType(HoldType)
y = strcmp(HoldType,'zoh') | strcmp(HoldType,'foh');
if(~y)
    error('Poles can have only two types "zoh" or "foh"')
end
end
function y = ValidateFrequency(Frequency)
y = ~isempty(Frequency);
if(~y)
    error('Frequency need to be defined')
end
end
