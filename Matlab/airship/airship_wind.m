function [sys,x0,str,ts,simStateCompliance] = airship_wind(t,x,u,flag,Lat, Lon, u_wind_selected, v_wind_selected)
%SFUNTMPL General MATLAB S-Function Template
%   With MATLAB S-functions, you can define you own ordinary differential
%   equations (ODEs), discrete system equations, and/or just about
%   any type of algorithm to be used within a Simulink block diagram.
%
%   The general form of an MATLAB S-function syntax is:
%       [SYS,X0,STR,TS,SIMSTATECOMPLIANCE] = SFUNC(T,X,U,FLAG,P1,...,Pn)
%
%   What is returned by SFUNC at a given point in time, T, depends on the
%   value of the FLAG, the current state vector, X, and the current
%   input vector, U.
%
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
%
%
%   The state vectors, X and X0 consists of continuous states followed
%   by discrete states.
%
%   Optional parameters, P1,...,Pn can be provided to the S-function and
%   used during any FLAG operation.
%
%   When SFUNC is called with FLAG = 0, the following information
%   should be returned:
%
%      SYS(1) = Number of continuous states.
%      SYS(2) = Number of discrete states.
%      SYS(3) = Number of outputs.
%      SYS(4) = Number of inputs.
%               Any of the first four elements in SYS can be specified
%               as -1 indicating that they are dynamically sized. The
%               actual length for all other flags will be equal to the
%               length of the input, U.
%      SYS(5) = Reserved for root finding. Must be zero.
%      SYS(6) = Direct feedthrough flag (1=yes, 0=no). The s-function
%               has direct feedthrough if U is used during the FLAG=3
%               call. Setting this to 0 is akin to making a promise that
%               U will not be used during FLAG=3. If you break the promise
%               then unpredictable results will occur.
%      SYS(7) = Number of sample times. This is the number of rows in TS.
%
%
%      X0     = Initial state conditions or [] if no states.
%
%      STR    = State ordering strings which is generally specified as [].
%
%      TS     = An m-by-2 matrix containing the sample time
%               (period, offset) information. Where m = number of sample
%               times. The ordering of the sample times must be:
%
%               TS = [0      0,      : Continuous sample time.
%                     0      1,      : Continuous, but fixed in minor step
%                                      sample time.
%                     PERIOD OFFSET, : Discrete sample time where
%                                      PERIOD > 0 & OFFSET < PERIOD.
%                     -2     0];     : Variable step discrete sample time
%                                      where FLAG=4 is used to get time of
%                                      next hit.
%
%               There can be more than one sample time providing
%               they are ordered such that they are monotonically
%               increasing. Only the needed sample times should be
%               specified in TS. When specifying more than one
%               sample time, you must check for sample hits explicitly by
%               seeing if
%                  abs(round((T-OFFSET)/PERIOD) - (T-OFFSET)/PERIOD)
%               is within a specified tolerance, generally 1e-8. This
%               tolerance is dependent upon your model's sampling times
%               and simulation time.
%
%               You can also specify that the sample time of the S-function
%               is inherited from the driving block. For functions which
%               change during minor steps, this is done by
%               specifying SYS(7) = 1 and TS = [-1 0]. For functions which
%               are held during minor steps, this is done by specifying
%               SYS(7) = 1 and TS = [-1 1].
%
%      SIMSTATECOMPLIANCE = Specifices how to handle this block when saving and
%                           restoring the complete simulation state of the
%                           model. The allowed values are: 'DefaultSimState',
%                           'HasNoSimState' or 'DisallowSimState'. If this value
%                           is not speficified, then the block's compliance with
%                           simState feature is set to 'UknownSimState'.


%   Copyright 1990-2010 The MathWorks, Inc.

%
% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,Lat, Lon, u_wind_selected, v_wind_selected);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;%连续变量数目
sizes.NumDiscStates  = 0;%离散变量数目
sizes.NumOutputs     = 4;%输出数
sizes.NumInputs      = 3;%输入数（控制器）
sizes.DirFeedthrough = 1;%输出函数中是否有 U 变量（系统一般是0，控制器一般是1）
sizes.NumSampleTimes = 1;   % 采样时间个数at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];  %第一个代表采样时间，第二个代表偏移量(从0开始，每隔0秒采样一次）（连续系统采样时间为0即可）
%若 sizes.NumSampleTimes = 0； 则ts 应为空： []; 

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u) %导数

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.处理离散状态更新、采样时间命中和主要时间步长要求。
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,Lat, Lon, u_wind_selected, v_wind_selected)
% Lat0 = 39.916527; %北京
% Lon0 = 116.3912757;
% alt0 = 0;
Lat0 = -8.913; 
Lon0 = -28.8985;
alt0 = 0;

wgs84 = wgs84Ellipsoid('meter');
[x0,y0,z0] = geodetic2ecef(wgs84,Lat0,Lon0,alt0);  % 坐标原点

% fprintf('X: %.3f m\nY: %.3f m\nZ: %.3f m\n', x0, y0, z0);
% [xy2lon,xy2lat,h] = ecef2geodetic(wgs84,x0,y0,z0)
% fprintf('X: %.3f m\n',h);
%% 
%输入的是北东地坐标
n = u(1); % NEU to ENU 坐标
e = u(2);
u1 = u(3); 

Lon0 = deg2rad(Lon0);
Lat0 = deg2rad(Lat0);

% enu to ecef
S = [-sin(Lon0) cos(Lon0) 0;
     -sin(Lat0)*cos(Lon0) -sin(Lat0)*sin(Lon0) cos(Lat0);
     cos(Lat0)*cos(Lon0) cos(Lat0)*sin(Lon0) sin(Lat0)]; 

dxyz = S'*[e;n;u1];
xyz = dxyz + [x0,y0,z0]'; % airship的ECEF坐标
%--------------------------------------------------------------
%enu ecef lla ecef enu
wgs84 = wgs84Ellipsoid('meter');
[xy2lat,xy2lon,] = ecef2geodetic(wgs84,xyz(1),xyz(2),xyz(3));



% [u_wind ,v_wind]= e(xy2lat,xy2lon,Lat,Lon,u_wind_selected,v_wind_selected);

u_wind = interp2(Lat, Lon, u_wind_selected, xy2lat, xy2lon,"nearest");
v_wind = interp2(Lat, Lon, v_wind_selected, xy2lat, xy2lon,"nearest");
% % ecef to enu


sys = [u_wind v_wind xy2lon xy2lat];

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.返回该区块下一击的时间。注意，结果是绝对时间。请注意，仅当您
% 在mdllnitializeSizes中的采样时间数组中指定变量离散时间采样时间【-20)时，才使用
% 此函数。
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

function [u_wind_interp,v_wind_interp] = as_wind(x,y,Lat,Lon,u_wind_selected,v_wind_selected)

airship_lat = x;
airship_lon = y;
u_wind_interp = interp2(Lat, Lon, u_wind_selected, airship_lat, airship_lon,"nearest");
v_wind_interp = interp2(Lat, Lon, v_wind_selected, airship_lat, airship_lon,"nearest");

% u_wind_interp = interp2(Lat, Lon, u_wind_selected, balloon_lat, balloon_lon,"nearest");
% v_wind_interp = interp2(Lat, Lon, v_wind_selected, balloon_lat, balloon_lon,"nearest");


% end mdlTerminate
