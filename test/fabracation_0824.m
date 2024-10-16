clear;
clc;

%����7core����λ��
ratio=1/1.4615;%ȷ��
r = 0.0415;
core_y = r*cos(pi/6);
core_z = r*sin(pi/6)*ratio;
core_z2 = r*ratio;

%�������м��
WA_dis_y = 0.127;

%�������

v_scan = 0.3;
v_move = 10;
v_z = 1;
length_all = 29;
length_2d = 10;
R = 40;
delta_arc = 0.05;
next_7_core = 2;

%����ͷ
[Filename,Filepath]=uiputfile('*.*','Save File','*.pgm');
f=fopen(strcat(Filepath,'20230831 7о����תһά��������.pgm'),'wt'); 
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


%����Բ�ܱ�����������ڵ�������ֶ���д7��ת�䲨��
%����1����
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

%����2����
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

%����3����
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

%����4����
y = 0;
z = 0;
x = 0;
fprintf(f,'LINEAR X%f Y%f Z%f F%f \n',x,y,z,v_move);
fprintf(f,'PSOCONTROL X ON\n');
fprintf(f,'DWELL 2\n');
[x]=line_x(x,length_all,v_scan,f);
fprintf(f,'PSOCONTROL X OFF\n');
fprintf(f,'DWELL 0.1\n');

%����5����
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

%����6����
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

%����7����
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

function[x,y,z]=s_bend_3d(f,A,O,delta_z,delta_y,R,v_scan,delta_arc)%delta_arcָ����ÿһС�ΰ뾶
    %����S���е���ά����
    %deltaָ����ƽ����X���ֱ����YZ�������
    delta_yz = sqrt(delta_z^2+delta_y^2);
    middle_delta_yz = delta_yz/2;
    middle_delta_x = sqrt(R^2-(R-middle_delta_yz)^2);
    middle_delta_y = delta_y/2;
    middle_delta_z = delta_z/2;
    M = A + [middle_delta_x,middle_delta_y,middle_delta_z];
    
    %�����һ��Բ��Բ��---------------------------------------------
    alpha = atan(middle_delta_y/middle_delta_z);
    C_delta_z = sqrt(R^2/(tan(alpha)^2+1));%���������ֵ
    C_delta_y = tan(alpha)*C_delta_z;
    if delta_z>0
        C = A + [0,C_delta_y,C_delta_z];%Բ��
    else
        C = A - [0,C_delta_y,C_delta_z];
    end
    %����нǴ�С
    CA = C-A;
    CM = C-M;
    theta = acos(dot(CA,CM)/(norm(CA)*norm(CM)));
    %����ƽ��P 
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
    
    %��һ��Բ��----------------------------------------------------------
    B = A+2*[middle_delta_x,middle_delta_y,middle_delta_z];
    if delta_z>0
        C = B - [0,C_delta_y,C_delta_z];%Բ��
    else
        C = B + [0,C_delta_y,C_delta_z];
    end
    %����нǴ�С
    CB = C-B;
    CM = C-M;
    theta = acos(dot(CB,CM)/(norm(CB)*norm(CM)));
    %����ƽ��P 
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
    %�ڻ�д����S-bendʱ����������ֵ�����...�ݺݵ���if else�����
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
