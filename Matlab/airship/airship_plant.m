function [sys,x0,str,ts,simStateCompliance] = airship_plant(t,x,u,flag)
switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 1,
    sys=mdlDerivatives(t,x,u);

  case 2,
    sys=mdlUpdate(t,x,u);

  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end airship_plant

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 12;%连续变量数目
sizes.NumDiscStates  = 0;%离散变量数目
sizes.NumOutputs     = 12;%输出数
sizes.NumInputs      = 9;%输入数（控制器）
sizes.DirFeedthrough = 0;%输出函数中是否有 U 变量（系统一般是0，控制器一般是1）
sizes.NumSampleTimes = 1;   % 采样    时间个数at least one sample time is needed

sys = simsizes(sizes);

  %phi滚转 theta俯仰 psi偏航  x1    y      z     p     q    r    ut  v    w
x0  = [0      0        0     0     0   20000  0     0    0    0   0    0];

str = [];

ts  = [0 0];  %第一个代表采样时间，第二个代表偏移量(从0开始，每隔0秒采样一次）（连续系统采样时间为0即可）
%若 sizes.NumSampleTimes = 0； 则ts 应为空： []; 
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlDerivatives(t,x,u) %导数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m = 5.6*10^4;
vol = 7.4*10^5;
rho = 0.089;
xg = 5;
% xg = 0;
zg = 15;
xp = 4;
yp = 1;
zp = 40;
Ix = 5*10^7;
Iy = 2.9*10^8;
Iz = 2.9*10^8;
Ixz = -6*10^4;
xi = 0; % pi/6
g = 9.81;
k1 = 0.17;
k2 = 0.83;
k3 = 0.52;
Cl1 = 2.4*10^4;
Cm1 =  7.7*10^4;
Cm2 = 7.7*10^4;
Cm3 = 7.7*10^4;
Cm4 =7.7*10^4;
Cn1 = 7.7*10^4;
Cn2 = 7.7*10^4;
Cn3 = 7.7*10^4;
Cn4 = 7.7*10^4;
Cx1 = 657;Cx2 = 657;
Cy1 = 657;Cy2 = 657;Cy3 = 657;Cy4 = 657;
Cz1 = 657;Cz2 = 657;Cz3 = 657;Cz4 = 657;Cz1 = 657;
mg = m*g;
pi = 3.141592653589793;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

phi = x(1); 
theta = x(2); 
psi = x(3);

x1 = x(4);
y = x(5);
z = x(6);

p = x(7);
q = x(8);
r = x(9);

ut = x(10);
v = x(11);
w = x(12);

u_wind = u(7);
v_wind = u(8);
w_wind = u(9);
Bf = mg;

Rr = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
      0 cos(phi)            -sin(phi);
      0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
%% 体坐标系 到 地面坐标系转换方程
Rg = [cos(theta)*cos(psi) sin(theta)*cos(psi)*sin(phi)-sin(psi)*cos(phi) sin(theta)*cos(psi)*cos(phi)+sin(psi)*sin(phi);
      cos(theta)*sin(psi) sin(theta)*sin(psi)*sin(phi)+cos(psi)*cos(phi) sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi);
      -sin(theta)         cos(theta)*sin(phi)                            cos(theta)*cos(phi)];

% 地面坐标系 到 体坐标系转换方程
Rgb = Rg';
R = blkdiag(Rr,Rg);
%xt = [phi theta psi x1 y z];
yt = [p;q;r;ut;v;w];

%ECEF to ENU

wind = Rgb*[v_wind;u_wind;w_wind]; %v为纬向风，
u_w = [ut;v;w]-wind ;% 飞艇与风的相对速度

wu = u_w(1);
wv = u_w(2);
ww = u_w(3);

M = [Ix 0 -Ixz 0 -m*zg 0;
      0 Iy+rho*vol*k3 0 m*zg 0 -m*xg;
      -Ixz 0 Iz+rho*vol*k3 0 m*xg 0;
      0 m*zg 0 m+rho*vol*k1 0 0;
      -m*zg 0 m*xg 0 m+rho*vol*k2 0;
      0 -m*xg 0 0 0 m+rho*vol*k2
      ];
N1 = [-(Iz-Iy)*q*r+Ixz*p*q+m*zg*(ut*r-w*p);
    -(Ix-Iz-rho*vol*k3)*p*r-Ixz*(p^2-r^2)-m*zg*(w*q-v*r)+m*xg*(v*p-ut*q);
    -(Iy+rho*vol*k3-Ix)*p*q-Ixz*q*r-m*xg*(ut*r-w*p);
    -(m+rho*vol*k2)*(w*q-v*r)+m*xg*(q^2+r^2)-m*zg*p*r;
    (m+rho*vol*k2)*w*p-(m+rho*vol*k1)*ut*r-m*xg*p*q-m*zg*q*r;
    (m+rho*vol*k1)*ut*q-(m+rho*vol*k2)*v*p-m*xg*r*p+m*zg*(q^2+p^2)];
N2 = [-zg*mg*cos(theta)*sin(phi);
    -zg*mg*sin(theta)-xg*mg*cos(theta)*cos(phi);
    xg*mg*cos(theta)*sin(phi);
    (Bf-mg)*sin(theta);
    -(Bf-mg)*cos(theta)*sin(phi);
    -(Bf-mg)*cos(theta)*cos(phi)];
%%
V = sqrt(wu^2+wv^2+ww^2); %飞艇相对速度
Q = 0.5*rho*(V^2); 
%%
B = [-zp*sin(xi) zp*sin(xi) yp -yp 0 0;
    zp*cos(xi) zp*cos(xi) -xp -xp 0 -2*Q*Cm4; %zp*cos(xi) zp*cos(xi) xp xp 0 -2*Q*Cm4; 
    -xp*sin(xi)+yp*cos(xi) xp*sin(xi)-yp*cos(xi) 0 0 -2*Q*Cn4 0;
    cos(xi) cos(xi) 0 0 0 0;
    sin(xi) -sin(xi) 0 0 2*Q*Cy4 0;
    0 0 1 1 0 -2*Q*Cz4];
% 
% alpha = atan2(ww,wu);
% beta = atan2(wv*cos(alpha),wu);


if wu > 0
    alpha = atan2(ww,wu);
    beta = asin(wv/V);
    La = Q*Cl1*sin(beta)*sin(abs(beta));
    Ma = -Q*(Cm1*cos(alpha/2)*sin(2*alpha))+Cm2*sin(2*alpha)+Cm3*sin(alpha)*sin(abs(alpha));
    Na = Q*(Cn1*cos(beta/2)*sin(2*beta)+Cn2*sin(2*beta)+Cn3*sin(beta)*sin(abs(beta)));
    Xa = -Q*(Cx1*cos(alpha).^2*cos(beta).^2+Cx2*sin(2*alpha)*sin(alpha/2));
    Ya = -Q*(Cy1*cos(beta/2)*sin(2*beta)+Cy2*sin(2*beta)+Cy3*sin(beta)*sin(abs(beta)));
    Za = -Q*(Cz1*cos(alpha/2)*sin(2*alpha)+Cz2*sin(2*alpha)+Cz3*sin(alpha)*sin(abs(alpha)));
else 
    alpha = atan2(ww,abs(wu));
    beta = asin(wv/V);
    La = 0;
    Ma = 0;
    Na = 0;
    Xa = Q*(Cx1*cos(alpha).^2*cos(beta).^2+Cx2*sin(2*alpha)*sin(alpha/2)) ;
    Ya = -Q*(Cy1*cos(beta/2)*sin(2*beta)+Cy2*sin(2*beta)+Cy3*sin(beta)*sin(abs(beta)));
    Za = -Q*(Cz1*cos(alpha/2)*sin(2*alpha)+Cz2*sin(2*alpha)+Cz3*sin(alpha)*sin(abs(alpha)));
    fprintf("尾向来流");
end

N3 = [La;Ma;Na;Xa;Ya;Za];

F = N1+N2+N3+B*u(1:6);
dxt = R*yt; % yt = [p;q;r;ut;v;w];
dyt = M \ F;

% sys = [dxt;dyt];
sys = [dxt(1) dxt(2) dxt(3) dxt(4)+10 dxt(5) dxt(6) dyt(1) dyt(2) dyt(3) dyt(4) dyt(5) dyt(6)];

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
function sys=mdlOutputs(t,x,u)

phi = x(1); 
theta = x(2);
psi = x(3);
x1 = x(4);
y = x(5);
z = -x(6); %%%

Rg = [cos(theta)*cos(psi) sin(theta)*cos(psi)*sin(phi)-sin(psi)*cos(phi) sin(theta)*cos(psi)*cos(phi)+sin(psi)*sin(phi);
      cos(theta)*sin(psi) sin(theta)*sin(psi)*sin(phi)+cos(psi)*cos(phi) sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi);
      -sin(theta)         cos(theta)*sin(phi)                            cos(theta)*cos(phi)];

p = x(7);
q = x(8);
r = x(9);


ut = x(10);
v = x(11);
w = -x(12); %%%
V = [ut;v;w];

%[phi theta psi x1 y z p q r ut v w]
sys = [phi theta psi x1 y z p q r V' ];

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
