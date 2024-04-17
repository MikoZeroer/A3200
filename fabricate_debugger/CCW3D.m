clear
A = [4.0000   -0.5000   -0.2791];
B = [5.5491   -0.5150   -0.2791];
O = [4.0000  -80.5000   -0.2791];

% A = [5.5491   -0.5150   -0.2791];
% B = [7.0982   -0.5300   -0.2791];
% O = [7.0982   79.4700   -0.2791];

isCCW = 0;
OA = A-O;
OB = B-O;
R = norm(OA);
u = OA/R;
w = cross(OA,OB);w=w/norm(w);
v = cross(w,u);
temp = cross(OA,OB);
theta = atan2(norm(temp),dot(OA,OB));
dot(temp,[0 0 1])
% theta = atan2(norm(cross(OB,OA)),dot(OB,OA));
% T = [[u',v',w',O'];[0,0,0,1]];
% t = linspace(0,theta,100);
% Res = T*[R*cos(t);R*sin(t);zeros(1,100);ones(1,100)];
% x=Res(1,:);y=Res(2,:);z=Res(3,:);
% clf
% plot3(x,y,z)
% hold on
% 
% t = linspace(2*pi,theta,100);