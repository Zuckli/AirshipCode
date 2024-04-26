
clc
hold on;
%ncdisp 'E:\airship\dataset\download.nc'
filename = 'E:\airship\dataset\download.nc';
lon = double(ncread(filename, 'longitude'));
lat = double(ncread(filename, 'latitude'));
level = ncread(filename, 'level');
time = ncread(filename, 'time');
u_wind = ncread(filename, 'u');
v_wind = ncread(filename, 'v');

t = datenum(1900,1,1,double(time),0,0);
t = datestr(t);


% 选择时间和气压
% target_time = datetime('05-Jun-2022 01:00:00', 'Format', 'yyyy-MM-dd HH:mm');
target_level = 10; % 气压层（hPa）

time_idx = find(datenum(t) == datenum('05-Jun-2022 01:00:00'));
level_idx = find(level == target_level);
% 提取所需时间和层次的风场数据
u_wind_selected = squeeze(u_wind(:, :, level_idx, time_idx));
v_wind_selected = squeeze(v_wind(:, :, level_idx, time_idx));

[Lat, Lon] = meshgrid(lat, lon);
[Lat, Lon, u_wind_selected, v_wind_selected] = recenter(Lat, Lon, u_wind_selected, v_wind_selected);

% 
% % 气球初始位置
% balloon_ori_lon = 90;
% balloon_ori_lat = 45;
% 
% balloon_lat = balloon_ori_lat;  
% balloon_lon = balloon_ori_lon;
% 
% % 时间参数
% total_hours = 10;  % 总时间
% dt = 0.1;          % 时间步长
% time_step = 0:dt:total_hours;     % 时间步长
% num_steps = length(time_step);

% % 气球轨迹
% balloon_trawjectory_lat = zeros(1, total_hours+1);%创建0矩阵
% balloon_trajectory_lon = zeros(1, total_hours+1);
% balloon_trajectory_lat(1) = balloon_lat;
% wballoon_trajectory_lon(1) = balloon_lon;
% 
% % 模拟循环
% for t = 2:num_steps
%     % 气球当前位置风速插值
%     
%     v_wind_interp = interp2(Lat, Lon, v_wind_selected, balloon_lat, balloon_lon,"nearest");
% 
%     % 气球位置更新
%     balloon_lat = balloon_lat + v_wind_interp * dt;
%     balloon_lon = balloon_lon + u_wind_interp * dt;
% 
%     % 轨迹数组存储当前位置
%     balloon_trajectory_lat(t) = balloon_lat;
%     balloon_trajectory_lon(t) = balloon_lon;
% end


% x = out.x;
% y = out.y;
% z = out.z;
% % pp = (x.^2+y.^2).^(0.5);
% % e2 = 0.00669437999014;
% % a = 6378137;
% % b = a * (1-e2)^0.5;
% % th = atan2(z*a,pp*b);
% % xy2lon = atan2(y,x);
% % xy2lat = atan2(z + e2*a*sin(th).^3,pp-e2*a*cos(th).^3);
% % plot(xy2lon(1),xy2lat(1),'-*')
% % plot(xy2lon, xy2lat, '-', 'LineWidth', 2);
% wgs84 = wgs84Ellipsoid('meter');
% [xy2lon,xy2lat,] = ecef2geodetic(wgs84,x,y,z);
% 
xy2lon = out.lon;
xy2lat = out.lat;

u_wind_interp = interp2(Lat, Lon, u_wind_selected, xy2lat, xy2lon,"nearest");
v_wind_interp = interp2(Lat, Lon, v_wind_selected, xy2lat, xy2lon,"nearest");

figure;
earthimage;
hold on;
quiversc(Lon, Lat, u_wind_selected, v_wind_selected, 'density', 100);
xy2lon = xy2lon';
xy2lat = xy2lat';
plot(xy2lon(1,1),xy2lat(1,1),'-*')
plot(xy2lon', xy2lat', '-', 'LineWidth', 2);

% 选取每隔n个轨迹点绘制风速矢量
indices = 1:floor(length(xy2lon)/10):length(xy2lon);  % 根据轨迹点数量调整步长

% 为选中的轨迹点绘制风速矢量
quiver(xy2lon(indices), xy2lat(indices), u_wind_interp(indices)', v_wind_interp(indices)', 'r');


% plot(balloon_ori_lon,balloon_ori_lat,'-*')
% plot(balloon_trajectory_lon, balloon_trajectory_lat, '-', 'LineWidth', 2);
title('气球随风漂流');
xlabel('经度');% lon
ylabel('纬度');% lat
legend('风场', '气球轨迹');

% figure;
% earthimage;
% hold on;
% quiversc(Lon, Lat, u_wind_selected, v_wind_selected, 'density', 100);
% plot(balloon_ori_lon,balloon_ori_lat,'-*')
% plot(balloon_trajectory_lon, balloon_trajectory_lat, '-', 'LineWidth', 2);
% title('气球随风漂流');
% xlabel('经度');% lon
% ylabel('纬度');% lat
% legend('风场', '气球轨迹');

