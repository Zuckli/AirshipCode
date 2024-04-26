close all
clc
 

x = out.x;
y = out.y;
z = out.z;
% plot(x, y, '-', 'LineWidth', 2);
plot3(x,y,z,'-') 
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')
axis equal 
 