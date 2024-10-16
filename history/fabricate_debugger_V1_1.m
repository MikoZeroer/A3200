%{
V1.1
2021/10/05
add:
.����֧�ֱ�ʶ��G92
.�������н�����ʾʹ��Ƭ���Լ�����Ԥ��ʱ��
.���ӹ�բ�����߿�
.��������޹ر���
.���ӿ���ʶ��
.���ӻ�ͼ�׾�Ϊplot3(0,0,0)
fixed:
.����LINEAR��ʱΪ0
changed:
.������ʶ���ж�˳��
.����CW/CCW��linspaceȡ����100->10
.����figure������ѡ�ļ�֮��
.������������ֻ��������hold
%}
%{
���Ǽ��룺
.�жϼ���������ʶ��
.ʶ����ͼָ��
.����ʱ��Ԥ��
%}
clear

[fileName,filePath]=uigetfile('*.pgm','Save File','');

if ~fileName
    error('δѡȡ�ļ�')
end
a = strfind(fileName,'.');
if ~a
    error('ѡȡ�ļ��ޡ�.��')
end
if fileName(max(a):end) ~= '.pgm'%#ok
    error('δѡȡ.pgm�ļ�')
end
figure()
plot3(0,0,0)
hold on

%% initial
f=fopen([filePath,fileName],'r');
clear filePath fileName
time = 0;
global X Y Z x0 y0 z0
X=0;Y=0;Z=0;x0=0;y0=0;z0=0;
global F changeX changeY changeZ addMode lineColor lineWidth R I J
F=0;changeX=0;changeY=0;changeZ=0;R=0;I=0;J=0;lineColor='y';lineWidth=0.1;
global changeX_G92 changeY_G92 changeZ_G92
changeX_G92=0;changeY_G92=0;changeZ_G92=0;

%% main
currentLine = fgetl(f);
while ischar(currentLine)
    % �ҵ���ǰ����ʶ��
    for bsfNum = 1:1:length(currentLine)
        if currentLine(bsfNum) == ' '
            bsfNum = bsfNum - 1;%#ok
            break
        end
    end
    bsf = currentLine(1:bsfNum);
    
    %% LINEAR
    %{
    ��num�洢��ǵ�һ������ĸ�Լ�ÿ����������һλ��λ��
    �жϲ�����ΪX��Y��Z��F������ֵ������Ӧȫ�ֱ�����
    ��ͼ����ʱ
    %}
    if strcmp(bsf,'LINEAR')
        if length(strsplit(currentLine)) == 1
            error('LINEARָ����δ��Ӳ�����')
        end
        %num���λ��Ϊ��һ�����Լ�ÿ����������һλ
        %��Ϊ��������ΪX Y Z F��λ�ñ������5��
        [num,a] = get_aixs_dis(currentLine,bsfNum,5,'LINEAR');
        %�Ƚ�num(i)+2���������������ȫ�ֱ�����
        switch_LINEAR(currentLine(num(1)),currentLine(num(1)+1:num(2)))
        if a > 2
            for i = 2:1:a-1
                switch_LINEAR(currentLine(num(i)+2),currentLine(num(i)+3:num(i+1)))
            end
        end
        %��ͼ����ʱ
        time = time + move_LINEAR();
        currentLine = fgetl(f);
        continue
    end
    %% PSOCONTROL
    %{
    ȡ'PSOCONTROL X '֮��Ĳ������ַ��������г��ո��
    ����ʣ'ON'��'OFF'��'RESET'
    %}
    if strcmp(bsf,'PSOCONTROL')
        %ȡ'PSOCONTROL X '֮��Ĳ������ַ��������г��ո�
        czf = strrep(currentLine(bsfNum+4:end),' ','');
        %�ж�'ON'��'OFF'��'RESET'
        n = 1;
        for a = strcmp(czf,{'ON','OFF','RESET'})
            if a
                break
            end
            n = n + 1;
        end
        if ~a
            error('PSOCONTROL�������������ON��OFF��RESET��')
        end
        %����բΪ���ߣ��ع�բΪ����
        switch n
            case 1
                lineColor = 'k';
                lineWidth = 1.5;
            case 2
                lineColor = 'y';
                lineWidth = 0.1;
        end
        currentLine = fgetl(f);
        continue
    end
    %% DWELL
    if strcmp(bsf,'DWELL')
        bsfNum1 = bsfNum + 2;
        %ȡDWELL��ո�ǰ��λ��
        for bsfNum2 = bsfNum1:1:length(currentLine)
            if currentLine(bsfNum2) == ' '
                bsfNum2 = bsfNum2 - 1;%#ok
                break
            end
        end
        %��ʱ
        time = time + str2double(currentLine(bsfNum1:bsfNum2));
        currentLine = fgetl(f);
        continue
    end
    %% �����˶�/����˶� addMode=1/0
    n = 0;%n=0Ϊ�����˶���n=1Ϊ����˶�
    for a = strcmp(bsf,{'INCREMENTAL','ABSOLUTE'})
        if a
            break
        end
        n = n + 1;
    end
    if a
        if n
            addMode = 1;
        else
            addMode = 0;
        end
        currentLine = fgetl(f);
        continue
    end
    %% CW
    if strcmp(bsf,'CW')
        if length(strsplit(currentLine)) == 1
            error('CWָ����δ��Ӳ�����')
        end
        %num���λ��Ϊ��һ�����Լ�ÿ����������һλ
        %����������5������X Y I J F��λ�ñ������6��
        [num,a] = get_aixs_dis(currentLine,bsfNum,6,'CW');
        %�Ƚ�num(i)+2���������������ȫ�ֱ�����
        switch_CW(currentLine(num(1)),currentLine(num(1)+1:num(2)))
        if a > 2
            for i = 2:1:a-1
                switch_CW(currentLine(num(i)+2),currentLine(num(i)+3:num(i+1)))
            end
        end
        %��ͼ����ʱ
        time = time + move_CW(1);
        currentLine = fgetl(f);
        continue
    end
    %% CCW
    if strcmp(bsf,'CCW')
        if length(strsplit(currentLine)) == 1
            error('CCWָ����δ��Ӳ�����')
        end
        %num���λ��Ϊ��һ�����Լ�ÿ����������һλ
        %����������5������X Y I J F��λ�ñ������6��
        [num,a] = get_aixs_dis(currentLine,bsfNum,6,'CCW');
        %�Ƚ�num(i)+2���������������ȫ�ֱ�����
        switch_CW(currentLine(num(1)),currentLine(num(1)+1:num(2)))
        if a > 2
            for i = 2:1:a-1
                switch_CW(currentLine(num(i)+2),currentLine(num(i)+3:num(i+1)))
            end
        end
        %��ͼ����ʱ
        time = time + move_CW(0);
        currentLine = fgetl(f);
        continue
    end
    %% G92
    if strcmp(bsf,'G92')
        if length(strsplit(currentLine)) == 1
            error('G92ָ����δ��Ӳ�����')
        end
        %num���λ��Ϊ��һ�����Լ�ÿ����������һλ
        %����������3������X Y Z��λ�ñ������4��
        [num,a] = get_aixs_dis(currentLine,bsfNum,4,'G92');
        switch_G92(currentLine(num(1)),currentLine(num(1)+1:num(2)))
        if a > 2
            for i = 2:1:a-1
                switch_G92(currentLine(num(i)+2),currentLine(num(i)+3:num(i+1)))
            end
        end
        currentLine = fgetl(f);
        continue
    end
    %% �������
    for a = strcmp(bsf,{'G359','ENABLE','METRIC','SECONDS','VELOCITY','PSOOUTPUT'})
        if a
            break
        end
    end
    if a
        currentLine = fgetl(f);
        continue
    end
    %% ����
    if isempty(bsf)
        warning('����')
        currentLine = fgetl(f);
        continue
    end
    %% ��ʶ���ʶ����
    error('��ʶ���ʶ����%s',bsf)
end
%% close
hold off
fclose(f);
clear ans a addMode bsf bsfNum bsfNum1 bsfNum2 currentLine czf f i lineColor n num R changeX changeY changeZ
msg = {'THE SIMULATION HAS BEEN COMPLETED';['Ƭ��(Y)�����ģ�',num2str(Y),'mm']};
hour = floor(time / 3600);
minute = floor((time - hour*3600) / 60);
second = floor(time - hour*3600 - minute*60);
msgTime = 'Ԥ������ʱ�䣺';
if hour
    msgTime = [msgTime,num2str(hour),'h'];
end
if minute
    msgTime = [msgTime,num2str(minute),'m'];
end
if second
    msgTime = [msgTime,num2str(second),'s'];
end
msg(3,1) = cellstr(msgTime);
msgbox(msg, 'Success');
clear msg

%% function
function [num,a] = get_aixs_dis(currentLine,bsfNum,amount,moveMode)
%{
������䣺
currentLine����ǰ���������ַ���
bsfNum��currentLine(1:bsfNum)Ϊ��ǰ����ʶ��
amount���趨Ԥ�����������������������X Y Z F����ȡ5

������䣺
num���λ��Ϊ��һ���������Լ�ÿ�����ֵ����һλ
����LINEAR X3.814386 F70.000000 
         X��     6��          ��0
num����Ϊ���ϼ�ͷ��ָ�ַ����ַ���������λ��
%}
num = zeros(1,amount);
num(1) = bsfNum + 2;
a = 1;
for bsfNum = num(1):1:length(currentLine)
    if currentLine(bsfNum) == ' '
        a = a + 1;
        if a > amount
            error([moveMode,'ָ���в���������'])
        end
        num(a) = bsfNum - 1;
    end
end
if currentLine(bsfNum) ~= ' '
    a = a + 1;
    if a > amount
        error([moveMode,'ָ���в���������'])
    end
    num(a) = bsfNum - 1;
end
num = num(1:a);
end

function switch_LINEAR(s1,s2)
global addMode
switch s1
    case 'X'
        global changeX X changeX_G92%#ok
        changeX = str2double(s2);
        if addMode
            X = changeX + changeX_G92;
        else
            X = X + changeX;
        end
    case 'Y'
        global changeY Y changeY_G92%#ok
        changeY = str2double(s2);
        if addMode
            Y = changeY + changeY_G92;
        else
            Y = Y + changeY;
        end
    case 'Z'
        global changeZ Z changeZ_G92%#ok
        changeZ = str2double(s2);
        if addMode
            Z = changeZ + changeZ_G92;
        else
            Z = Z + changeZ;
        end
    case 'F'
        global F%#ok
        F = str2double(s2);
end
end
function time_LINEAR = move_LINEAR()
global F X Y Z lineColor lineWidth x0 y0 z0
plot3([x0,X],[y0,Y],[z0,Z],[lineColor,'.-'],'linewidth',lineWidth)
time_LINEAR = sqrt((x0-X)^2+(y0-Y)^2+(z0-Z)^2)/F;
x0=X;y0=Y;z0=Z;
end

function switch_CW(s1,s2)
global addMode
switch s1
    case 'X'
        global changeX X changeX_G92%#ok
        changeX = str2double(s2);
        if addMode
            X = changeX + changeX_G92;
        else
            X = X + changeX;
        end
    case 'Y'
        global changeY Y changeY_G92%#ok
        changeY = str2double(s2);
        if addMode
            Y = changeY + changeY_G92;
        else
            Y = Y + changeY;
        end
    case 'I'
        global I%#ok
        I = str2double(s2);
    case 'J'
        global J%#ok
        J = str2double(s2);
    case 'F'
        global F%#ok
        F = str2double(s2);
    case 'Z'
        global changeZ Z changeZ_G92%#ok
        changeZ = str2double(s2);
        if addMode
            Z = changeZ + changeZ_G92;
        else
            Z = Z + changeZ;
        end
    case 'R'
        global R%#ok
        R = str2double(s2);
    otherwise
        error('CW��ʶ���������%s',s1)
end
end
function time_CW = move_CW(mode)
global F addMode X Y Z lineColor lineWidth I J R x0 y0 z0
if Z == z0
    if R
        [theta,I,J] = R2theta_CW([x0,X],[y0,Y],R);
    else
        [theta,I,J,R] = IJ2theta_CW([x0,X],[y0,Y],I,J,addMode);
    end
    if mode
        plotArrayTheta = linspace(theta(1),theta(2),10);
    else
        plotArrayTheta = linspace(theta(2),theta(1),10);
    end
    plotArrayNum = length(plotArrayTheta);
    plotArray = [I*ones(1,plotArrayNum)+R*cos(plotArrayTheta);J*ones(1,plotArrayNum)+R*sin(plotArrayTheta);z0*ones(1,plotArrayNum)];
    plot3([x0,X],[y0,Y],[z0,z0],[lineColor,'.'],'linewidth',lineWidth)
elseif Y == y0
    if R
        [theta,I,J] = R2theta_CW([x0,X],[z0,Z],R);
    else
        [theta,I,J,R] = IJ2theta_CW([x0,X],[z0,Z],I,J,addMode);
    end
    if mode
        plotArrayTheta = linspace(theta(1),theta(2),10);
    else
        plotArrayTheta = linspace(theta(2),theta(1),10);
    end
    plotArrayNum = length(plotArrayTheta);
    plotArray = [I*ones(1,plotArrayNum)+R*cos(plotArrayTheta);y0*ones(1,plotArrayNum);J*ones(1,plotArrayNum)+R*sin(plotArrayTheta)];
    plot3([x0,X],[y0,y0],[z0,Z],[lineColor,'.'],'linewidth',lineWidth)
elseif X == x0
    if R
        [theta,I,J] = R2theta_CW([y0,Y],[z0,Z],R);
    else
        [theta,I,J,R] = IJ2theta_CW([y0,Y],[z0,Z],I,J,addMode);
    end
    if mode
        plotArrayTheta = linspace(theta(1),theta(2),10);
    else
        plotArrayTheta = linspace(theta(2),theta(1),10);
    end
    plotArrayNum = length(plotArrayTheta);
    plotArray = [x0*ones(1,plotArrayNum);I*ones(1,plotArrayNum)+R*cos(plotArrayTheta);J*ones(1,plotArrayNum)+R*sin(plotArrayTheta)];
    plot3([x0,x0],[y0,Y],[z0,Z],[lineColor,'.'],'linewidth',lineWidth)
else
    error('CW�ƶ��ᳬ������')
end
plot3(plotArray(1,:),plotArray(2,:),plotArray(3,:),[lineColor,'-'],'linewidth',lineWidth)
if mode
    time_CW = R*(theta(1)-theta(2))/F;
else
    time_CW = R*(theta(2)-theta(1))/F;
end
R = 0;
x0=X;y0=Y;z0=Z;
end
function [theta,i,j] = R2theta_CW(x,y,r)
%{
���������
x��y���������յ����꣬rΪ�뾶
��rΪ��ʱ��Ϊ�ӻ�
rΪ��ʱ��Ϊ�Ż�
���������
Բ�����꼰�뾶����ֵ
%}
p = sqrt((2*r/sqrt((x(1)-x(2))^2+(y(1)-y(2))^2))^2-1);
if r > 0%�ӻ�
    i = (x(1)+x(2))/2-(y(1)-y(2))*p;
    j = (y(1)+y(2))/2-(x(1)-x(2))*p;
    c = r;
else
    i = (x(1)+x(2))/2+(y(1)-y(2))*p;
    j = (y(1)+y(2))/2+(x(1)-x(2))*p;
    c = -r;
end
theta = [acos((x(1)-i)/c),acos((x(2)-i)/c)];
%����matlab��acosֻ����[0,pi]�������ǻ�����Ҫ2pi��Χ
if y(1) < j
    theta(1) = -theta(1);
end
if y(2) < j
    theta(2) = -theta(2);
end
end
function [theta,i,j,r] = IJ2theta_CW(x,y,i,j,addmode)
if ~addmode%����˶�
    i=x(1)+i;j=y(1)+j;
end
r = sqrt((i-x(1))^2+(j-y(1))^2);
theta = [acos((x(1)-i)/r),acos((x(2)-i)/r)];
%����matlab��acosֻ����[0,pi]�������ǻ�����Ҫ2pi��Χ
if y(1) < j
    theta(1) = -theta(1);
end
if y(2) < j
    theta(2) = -theta(2);
end
end

function switch_G92(s1,s2)
switch s1
    case 'X'
        global X changeX_G92%#ok
        changeX_G92 = X - str2double(s2);
    case 'Y'
        global Y changeY_G92%#ok
        changeY_G92 = Y - str2double(s2);
    case 'Z'
        global Z changeZ_G92%#ok
        changeZ_G92 = Z - str2double(s2);
    otherwise
        error('G92��ʶ���������%s',s1)
end
end