function [A,B,C,D,K,L,CL_system,Gm,Pm,StepResponse] = DTsystem(A,B,C,D,SystemType,FinalPoles,ObserverPoles,PolesType,ObserverType,SensorNoiseVariance,f,HoldType)

if(strcmp(SystemType,'Continuous'))
    D_G = c2d(ss(A,B,C,D),(1/f),HoldType);
    A= D_G.a;B = D_G.b;C = D_G.c;D = D_G.d;
elseif(strcmp(SystemType,'Discrete'))
    % Need not to do anything as we already have the required ss
    D_G = ss(A,B,C,D);D_G.Ts = 1/f;
else
    error('System Type can be either "Continuous" or "Discrete"');
end

if(strcmp(PolesType,'Continuous'))
    FinalPoles = exp(FinalPoles.*(1/f));
    K = place(A,B,FinalPoles);
    if(~isnan(ObserverPoles))
        ObserverPoles = exp(ObserverPoles.*(1/f));
        L = place(A',C',ObserverPoles);L = L';
    end
elseif(strcmp(SystemType,'Discrete'))
    % Need not to do anything as we already have the required ss
    K = place(A,B,FinalPoles);
    if(~isnan(ObserverPoles))
        L = place(A',C',ObserverPoles);L = L';
    end
else
    error('Poles Type can be either "Continuous" or "Discrete"');
end
% Have to figure out how Q and R map to sensor characteristics
if(strcmp(ObserverType,'KalmanObserver'))
    Q = 0.1;R = SensorNoiseVariance;
    % Setting this by default but should be a function of noise in the
    % system
    B1 = 0.1*ones(size(B,2),1); 
    Q = B1*B1';
    D1 = ones(size(D,1),size(D,2));
    if(isnan(R)) 
        D1 = 0.01*D1;
        R = D1*D1';
    end
    % Observer Poles not defined by user, so I am using Kalman Filter to find
    % L in some optimal sense
    [~,~,~,L] = kalman(D_G,Q,R);
elseif(strcmp(ObserverType,'SimpleObserver'))
    if(isnan(ObserverPoles))
        error('For Simple Observer you need to specify Observer poles');
    end
end
        

%display('manish');

D_C = ss(A-B*K-L*C,L,K,0,1/f);
D_T = ss(A-B*K-L*C,B,-K,1,1/f);

% Lines for debugging purposes, verification with example case in slides
% [numC,denC] = ss2tf(A-B*K-L*C,L,K,0);
% [numT,denT] = ss2tf(A-B*K-L*C,B,-K,1);
% D_Ob_ltf = D_G*D_C;
% system_1 = tf(D_Ob_ltf);
% margin(D_Ob_ltf);

N = pinv([C zeros(1,3)]*pinv([eye(3)-A+B*K B*K ;zeros(3) eye(3)-A+L*C])*[B;zeros(3,1)]);
% This CL_system equation is unstable beyond are the reason is beyond my
% perception
%CL_system = (D_G/(1 + D_G*D_C))*D_T*N;

CL_system = feedback(D_G,D_C)*D_T*N;


%CL_system = (D_G/(1 + D_G*D_C));
%CL_system = MM/(1+MM);
StepResponse = stepinfo(CL_system);
[Gm,Pm,~,~] = margin(CL_system);
% display(Gm);
% display(Pm);
end



