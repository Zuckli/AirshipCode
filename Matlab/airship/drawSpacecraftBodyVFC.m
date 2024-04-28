function drawSpacecraftBodyVFC(uu)
scale = 20;
% process inputs to function
pn       = uu(1);       % inertial North position
pe       = uu(2);       % inertial East position
pd       = uu(3);       % inertial Down position
phi      = uu(4);       % roll angle
theta    = uu(5);       % pitch angle
psi      = uu(6);       % yaw angle
t        = uu(7);       % time
u_wind   = uu(8);
v_wind   = uu(9);
w_wind   = 0;
fprintf("U %.3f V %.3f\n",u_wind,v_wind);
% transform vertices from NED to XYZ (for matlab rendering)
    pn1 = pn;
    pn = pe;
    pe = pn1;
    pd = -pd;

% define persistent variables
    persistent aircraftOrigin;
    persistent Vertices;
    persistent Faces;
    persistent facecolors;
    persistent windArrowHandle;
    persistent trajectory;

if t==0
    figure(1); clf;
    pos0 = [pn,pe,pd];
    plot3(pn,pe,pd,'b-*','MarkerSize',5);
    [Vertices, Faces, facecolors] = defineSpaceCraftBody;
    aircraftOrigin = drawSpacecraftBody(Vertices, Faces, facecolors, pn, pe, pd, phi, theta, psi, []);
    hold on
    trajectory = drawtrajectory(pn, pe, pd,[]);
    hold on
    windArrowHandle = drawSpacewindBody(pn, pe, pd,[],u_wind,v_wind,w_wind,scale);
    
    title('SpaceCraft')
    xlabel('East')
    ylabel('North')
    zlabel('-Down')
    view(32,47)  % set the view angle for figure
    axis([-50*scale+pn,50*scale+pn,-50*scale+pe,50*scale+pe,-50*scale+pd, 50*scale+pd]);
    grid on

    % at every other time step, redraw box
else
    drawSpacecraftBody(Vertices, Faces, facecolors, pn, pe, pd, phi, theta, psi, aircraftOrigin);
    drawSpacewindBody(pn, pe, pd, windArrowHandle,u_wind,v_wind,w_wind,scale); 
    drawtrajectory(pn, pe, pd,trajectory);
    axis([-50*scale+pn,50*scale+pn,-50*scale+pe,50*scale+pe,-50*scale+pd, 50*scale+pd]);
%     axis([-(50+px)*scale+pn,(50+px)*scale+pn,-(50+py)*scale+pe,(50+py)*scale+pe,-(50+pz)*scale+pd, (50+pz)*scale+pd]);
end

end

%
function handle = drawSpacewindBody(pn, pe, pd,handle,u_wind,v_wind,w_wind,scale)


[X,Y,Z] = meshgrid(-50*scale+pn:100*scale*0.333:50*scale+pn,-50*scale+pe:100*scale*0.333:50*scale+pe,-50*scale+pd:100*scale*0.333:50*scale+pd);

U_wind = X;V_wind = Y;W_wind = Z;
U_wind(:,:,:) = u_wind;V_wind(:,:,:) = v_wind;W_wind(:,:,:) = w_wind;

if isempty(handle)
    handle = quiver3(X,Y,Z,U_wind,V_wind,W_wind,'Color',[0 0.4470 0.7410]);
else
    set(handle,'XData',X,'YData',Y,'ZData',Z,'UData',U_wind,'VData',V_wind,'WData',W_wind);
    drawnow
end
end

%
function handle = drawSpacecraftBody(V,F,patchcolors,pn,pe,pd,phi,theta,psi,handle)

    V = rotate(V, phi, theta, psi)';  % rotate vehicle
    V = translate(V, pn, pe, pd)';  % translate vehicle
    if isempty(handle)
        handle = patch('Vertices', V, 'Faces', F,...
            'FaceVertexCData',patchcolors,...
            'FaceColor','flat');
    else
        set(handle,'Vertices',V,'Faces',F);
        drawnow
    end
end
  
%
function handle = drawtrajectory(pn, pe, pd,handle)
    if isempty(handle)
        handle = plot3(pn,pe,pd,'r-.','LineWidth', 2);
    else
        set(handle,'Xdata',[get(handle,'Xdata'),pn],'Ydata',[get(handle,'Ydata'),pe],'Zdata',[get(handle,'Zdata'),pd]);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

% define rotation matrix (right handed)
R_roll = [...
    1, 0, 0;...
    0, cos(phi), -sin(phi);...
    0, sin(phi), cos(phi)];
R_pitch = [...
    cos(theta), 0, sin(theta);...
    0, 1, 0;...
    -sin(theta), 0, cos(theta)];
R_yaw = [...
    cos(psi), -sin(psi), 0;...
    sin(psi), cos(psi), 0;...
    0, 0, 1];
% transform vertices from NED to XYZ (for matlab rendering)
Rt = [...
    0, 1, 0;...
    1, 0, 0;...
    0, 0, -1;...
    ];
R = (R_roll*R_pitch*R_yaw)'*Rt;
% note that R above either leaves the vector alone or rotates
% a vector in a left handed rotation.  We want to rotate all
% points in a right handed rotation, so we must transpose
% R = R';

% rotate vertices
pts = pts*R;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

pts = pts + repmat([pn;pe;pd],1,size(pts,2));

end

function [V,F,patchcolors] = defineSpaceCraftBody()
 scale = 20;
% Define the vertices (physical location of vertices)
V = [
     7    0   0; % point 1 y z x
     4    1  -1; % point 2
     4   -1  -1; % point 3
     4   -1   1; % point 4
     4    1   1; % point 5
   -15    0   0; % point 6
     0   10   0; % point 7
    -6   10   0; % point 8
    -6  -10   0; % point 9
     0  -10   0; % point 10
   -12    5   0; % point 11
   -15    5   0; % point 12
   -15   -5   0; % point 13
   -12   -5   0; % point 14
   -12    0   0; % point 15
   -15    0  -3; % point 16
   -15    0   0;
    7     0   0;
   ];

 V = V*scale;

% define faces as a list of vertices numbered above
F = [
    1,   2,   3,   1; % front
    1,   2,   5,   1; % front  
    1,   3,   4,   1; % front
    2,   3,   6,   2; % top
    7,   8,   9,  10; % top
   11,  12,  13,  14; % top
    2,   5,   6,   2; % right
    6,  15,  16,   6; % right
    3,   4,   6,   3; % left
    5,   4,   6,   5; % bottom
    ];

% define colors for each face
myred       = [1, 0, 0];
mygreen     = [0, 1, 0];
myblue      = [0, 0, 1];
myyellow    = [1, 1, 0];
mymagenta   = [0, 1, 1];

patchcolors = [
    myred;     % front
    myred;     % front
    myred;     % front
    mygreen;   % top
    mygreen;   % top
    mygreen;   % top
    myblue;    % right
    myblue;    % right
    myyellow;  % left
    mymagenta; % bottom
    ];
end
