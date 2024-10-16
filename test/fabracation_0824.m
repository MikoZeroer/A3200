clear;
clc;

%计算7core波导位置
ratio=1/1.4615;%确定
r = 0.0415;
core_y = r*cos(pi/6);
core_z = r*sin(pi/6)*ratio;
core_z2 = r*ratio;

%波导阵列间距
WA_dis_y = 0.127;

%程序变量

v_scan = 0.3;
v_move = 10;
v_z = 1;
length_all = 29;
length_2d = 10;
R = 40;
delta_arc = 0.05;
next_7_core = 2;

%程序头
[Filename,Filepath]=uiputfile('*.*','Save File','*.pgm');
f=fopen(strcat(Filepath,'20230831 7芯光纤转一维波导阵列.pgm'),'wt'); 
fprintf(f,'ENABLE X Y Z\n');
fprintf(f,'METRIC\n');
fprintf(f,'SECONDS\n');   
fprintf(f,'G359\n');  
fprintf(f,'VELOCITY ON\n');  
fprintf(f,'PSOCONTROL X RESET \n');  
fprintf(f,'PSOOUTPUT X CONTROL 1 0\n');  
fprintf(f,'G92 X0 Y0 Z0\n'); 
fprintf(f,'ABSOLUTE\n');

x=0;
y=0;
z=0;


%沿着圆周遍历将会产生遮挡，因此手动划写7根转弯波导
%波导1最下
z = -core_z2;
fprintf(f,'LINEAR Z%f F%f \n',z,v_z);
fprintf(f,'PSOCONTROL X ON\n');
fprintf(f,'DWELL 2\n');
O = [x,y,z];
[x]=line_x(x,length_2d,v_scan,f);
A = [x,y,z];
delta_z = core_z2;
delta_y = WA_dis_y;

[x,y,z]=s_bend_3d(f,A,O,delta_z,delta_y,R,v_scan,delta_arc);

fprintf(f,'LINEAR X%f F%f \n',length_all,v_scan);
fprintf(f,'PSOCONTROL X OFF\n');
fprintf(f,'DWELL 0.1\n');

%波导2右下
y = core_y;
z = -core_z;
x = 0;
fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',x,y,z,v_move);
fprintf(f,'PSOCONTROL X ON\n');
fprintf(f,'DWELL 2\n');
O = [x,y,z];
[x]=line_x(x,length_2d,v_scan,f);
A = [x,y,z];
delta_z = core_z;
delta_y = WA_dis_y*2-core_y;
[x,y,z]=s_bend_3d(f,A,O,delta_z,delta_y,R,v_scan,delta_arc);
fprintf(f,'LINEAR X%f F%f \n',length_all,v_scan);
fprintf(f,'PSOCONTROL X OFF\n');
fprintf(f,'DWELL 0.1\n');

%波导3左下
y = -core_y;
z = -core_z;
x = 0;
fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',x,y,z,v_move);
fprintf(f,'PSOCONTROL X ON\n');
fprintf(f,'DWELL 2\n');
O = [x,y,z];
[x]=line_x(x,length_2d,v_scan,f);
A = [x,y,z];
delta_z = core_z;
delta_y = -(WA_dis_y*3-core_y);
[x,y,z]=s_bend_3d(f,A,O,delta_z,delta_y,R,v_scan,delta_arc);
fprintf(f,'LINEAR X%f F%f \n',length_all,v_scan);
fprintf(f,'PSOCONTROL X OFF\n');
fprintf(f,'DWELL 0.1\n');

%波导4中心
y = 0;
z = 0;
x = 0;
fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',x,y,z,v_move);
fprintf(f,'PSOCONTROL X ON\n');
fprintf(f,'DWELL 2\n');
[x]=line_x(x,length_all,v_scan,f);
fprintf(f,'PSOCONTROL X OFF\n');
fprintf(f,'DWELL 0.1\n');

%波导5右上
y = core_y;
z = core_z;
x = 0;
fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',x,y,z,v_move);
fprintf(f,'PSOCONTROL X ON\n');
fprintf(f,'DWELL 2\n');
O = [x,y,z];
[x]=line_x(x,length_2d,v_scan,f);
A = [x,y,z];
delta_z = -core_z;
delta_y = WA_dis_y*3-core_y;
[x,y,z]=s_bend_3d(f,A,O,delta_z,delta_y,R,v_scan,delta_arc);
fprintf(f,'LINEAR X%f F%f \n',length_all,v_scan);
fprintf(f,'PSOCONTROL X OFF\n');
fprintf(f,'DWELL 0.1\n');

%波导6左上
y = -core_y;
z = core_z;
x = 0;
fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',x,y,z,v_move);
fprintf(f,'PSOCONTROL X ON\n');
fprintf(f,'DWELL 2\n');
O = [x,y,z];
[x]=line_x(x,length_2d,v_scan,f);
A = [x,y,z];
delta_z = -core_z;
delta_y = -(WA_dis_y*2-core_y);
[x,y,z]=s_bend_3d(f,A,O,delta_z,delta_y,R,v_scan,delta_arc);
fprintf(f,'LINEAR X%f F%f \n',length_all,v_scan);
fprintf(f,'PSOCONTROL X OFF\n');
fprintf(f,'DWELL 0.1\n');

%波导7最上
y = 0;
z = core_z2;
x = 0;
fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',x,y,z,v_move);
fprintf(f,'PSOCONTROL X ON\n');
fprintf(f,'DWELL 2\n');
O = [x,y,z];
[x]=line_x(x,length_2d,v_scan,f);
A = [x,y,z];
delta_z = -core_z2;
delta_y = -(WA_dis_y*1);
[x,y,z]=s_bend_3d(f,A,O,delta_z,delta_y,R,v_scan,delta_arc);
fprintf(f,'LINEAR X%f F%f \n',length_all,v_scan);
fprintf(f,'PSOCONTROL X OFF\n');
fprintf(f,'DWELL 0.1\n');


%move to next
y = y - next_7_core;
x = 0;
fprintf(f,'LINEAR X%f Y%f F%f \n',x,y,v_move);
fprintf(f,'DWELL 0.1\n');
fclose(f);

%---------------------FUNCTIONS--------------------------- 
function[x]=line_x(x,length,v_scan,f)
    x = x + length;
    fprintf(f,'LINEAR X%f F%f \n',x,v_scan);
end

function[x,y,z]=s_bend_3d(f,A,O,delta_z,delta_y,R,v_scan,delta_arc)%delta_arc指弧上每一小段半径
    %计算S弯中点三维坐标
    %delta指两条平行于X轴的直线其YZ方向距离
    delta_yz = sqrt(delta_z^2+delta_y^2);
    middle_delta_yz = delta_yz/2;
    middle_delta_x = sqrt(R^2-(R-middle_delta_yz)^2);
    middle_delta_y = delta_y/2;
    middle_delta_z = delta_z/2;
    M = A + [middle_delta_x,middle_delta_y,middle_delta_z];
    
    %计算第一段圆弧圆心---------------------------------------------
    alpha = atan(middle_delta_y/middle_delta_z);
    C_delta_z = sqrt(R^2/(tan(alpha)^2+1));%仅计算绝对值
    C_delta_y = tan(alpha)*C_delta_z;
    if delta_z>0
        C = A + [0,C_delta_y,C_delta_z];%圆心
    else
        C = A - [0,C_delta_y,C_delta_z];
    end
    %计算夹角大小
    CA = C-A;
    CM = C-M;
    theta = acos(dot(CA,CM)/(norm(CA)*norm(CM)));
    %构造平面P 
    OA = A-O;
    OC = C-O;
    nP = cross(OA, OC);
    P = [OA; cross(OA, nP); nP];
    a = P(1,:);
    b = cross(a, nP);
    u = b / norm(b);
    v = cross(u, nP);
    v = v / norm(v);
    number = floor(norm(A-M)/delta_arc);
    delta_theta = theta/number;
    for i = -delta_theta:-delta_theta:-theta
        next_point = C + R*cos(i)*u + R*sin(i)*v;
        fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',next_point(1),next_point(2),next_point(3),v_scan); 
    end
    
    %另一个圆心----------------------------------------------------------
    B = A+2*[middle_delta_x,middle_delta_y,middle_delta_z];
    if delta_z>0
        C = B - [0,C_delta_y,C_delta_z];%圆心
    else
        C = B + [0,C_delta_y,C_delta_z];
    end
    %计算夹角大小
    CB = C-B;
    CM = C-M;
    theta = acos(dot(CB,CM)/(norm(CB)*norm(CM)));
    %构造平面P 
    OB = B-O;
    OC = C-O;
    nP = cross(OB, OC);
    P = [OB; cross(OB, nP); nP];   
    a = P(1,:);
    b = cross(a, nP);
    u = b / norm(b);
    v = cross(u, nP);
    v = v / norm(v);
    degree_offset = 0;
    %在划写后半段S-bend时，出现了奇怪的问题...狠狠地用if else解决了
    for i = theta:-delta_theta:0
        temp_point = C + R*cos(i)*u + R*sin(i)*v;
        if temp_point(1)>next_point(1)
            next_point = temp_point;
        else
            degree_offset = degree_offset+1;
        end
        fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',next_point(1),next_point(2),next_point(3),v_scan);
    end
    for degree = 1:degree_offset
        i = -degree*delta_theta;
        next_point = C + R*cos(i)*u + R*sin(i)*v;
        fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',next_point(1),next_point(2),next_point(3),v_scan);
    end
    x = next_point(1);
    y = next_point(2);
    z = next_point(3);
end
