%{
V1.1
2021/10/05
add:
.增加支持标识符G92
.增加运行结束显示使用片长以及运行预估时间
.增加光闸开关线宽
.增加清除无关变量
.增加空行识别
.增加画图首句为plot3(0,0,0)
fixed:
.修正LINEAR计时为0
changed:
.调整标识符判断顺序
.调整CW/CCW中linspace取点数100->10
.调整figure弹出在选文件之后
.调整整个程序只调用两次hold
%}
%{
考虑加入：
.判断计数，排序识别
.识别作图指令
.修正时间预测
%}
clear

[fileName,filePath]=uigetfile('*.pgm','Save File','');

if ~fileName
    error('未选取文件')
end
a = strfind(fileName,'.');
if ~a
    error('选取文件无“.”')
end
if fileName(max(a):end) ~= '.pgm'%#ok
    error('未选取.pgm文件')
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
    % 找到当前语句标识符
    for bsfNum = 1:1:length(currentLine)
        if currentLine(bsfNum) == ' '
            bsfNum = bsfNum - 1;%#ok
            break
        end
    end
    bsf = currentLine(1:bsfNum);
    
    %% LINEAR
    %{
    用num存储标记第一个轴字母以及每个距离的最后一位的位置
    判断操作符为X、Y、Z、F，并将值置于相应全局变量中
    画图并计时
    %}
    if strcmp(bsf,'LINEAR')
        if length(strsplit(currentLine)) == 1
            error('LINEAR指令中未添加操作符')
        end
        %num标记位置为第一个轴以及每个距离的最后一位
        %认为最多操作符为X Y Z F，位置标记至多5个
        [num,a] = get_aixs_dis(currentLine,bsfNum,5,'LINEAR');
        %比较num(i)+2；将后面的数置于全局变量中
        switch_LINEAR(currentLine(num(1)),currentLine(num(1)+1:num(2)))
        if a > 2
            for i = 2:1:a-1
                switch_LINEAR(currentLine(num(i)+2),currentLine(num(i)+3:num(i+1)))
            end
        end
        %画图并计时
        time = time + move_LINEAR();
        currentLine = fgetl(f);
        continue
    end
    %% PSOCONTROL
    %{
    取'PSOCONTROL X '之后的操作符字符串并进行除空格后
    可能剩'ON'、'OFF'、'RESET'
    %}
    if strcmp(bsf,'PSOCONTROL')
        %取'PSOCONTROL X '之后的操作符字符串并进行除空格
        czf = strrep(currentLine(bsfNum+4:end),' ','');
        %判断'ON'、'OFF'、'RESET'
        n = 1;
        for a = strcmp(czf,{'ON','OFF','RESET'})
            if a
                break
            end
            n = n + 1;
        end
        if ~a
            error('PSOCONTROL后操作符不属于ON、OFF、RESET：')
        end
        %开光闸为黑线，关光闸为黄线
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
        %取DWELL后空格前的位置
        for bsfNum2 = bsfNum1:1:length(currentLine)
            if currentLine(bsfNum2) == ' '
                bsfNum2 = bsfNum2 - 1;%#ok
                break
            end
        end
        %计时
        time = time + str2double(currentLine(bsfNum1:bsfNum2));
        currentLine = fgetl(f);
        continue
    end
    %% 绝对运动/相对运动 addMode=1/0
    n = 0;%n=0为绝对运动，n=1为相对运动
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
            error('CW指令中未添加操作符')
        end
        %num标记位置为第一个轴以及每个距离的最后一位
        %最多操作符有5个，如X Y I J F，位置标记至多6个
        [num,a] = get_aixs_dis(currentLine,bsfNum,6,'CW');
        %比较num(i)+2；将后面的数置于全局变量中
        switch_CW(currentLine(num(1)),currentLine(num(1)+1:num(2)))
        if a > 2
            for i = 2:1:a-1
                switch_CW(currentLine(num(i)+2),currentLine(num(i)+3:num(i+1)))
            end
        end
        %画图并计时
        time = time + move_CW(1);
        currentLine = fgetl(f);
        continue
    end
    %% CCW
    if strcmp(bsf,'CCW')
        if length(strsplit(currentLine)) == 1
            error('CCW指令中未添加操作符')
        end
        %num标记位置为第一个轴以及每个距离的最后一位
        %最多操作符有5个，如X Y I J F，位置标记至多6个
        [num,a] = get_aixs_dis(currentLine,bsfNum,6,'CCW');
        %比较num(i)+2；将后面的数置于全局变量中
        switch_CW(currentLine(num(1)),currentLine(num(1)+1:num(2)))
        if a > 2
            for i = 2:1:a-1
                switch_CW(currentLine(num(i)+2),currentLine(num(i)+3:num(i+1)))
            end
        end
        %画图并计时
        time = time + move_CW(0);
        currentLine = fgetl(f);
        continue
    end
    %% G92
    if strcmp(bsf,'G92')
        if length(strsplit(currentLine)) == 1
            error('G92指令中未添加操作符')
        end
        %num标记位置为第一个轴以及每个距离的最后一位
        %最多操作符有3个，如X Y Z，位置标记至多4个
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
    %% 配置语句
    for a = strcmp(bsf,{'G359','ENABLE','METRIC','SECONDS','VELOCITY','PSOOUTPUT'})
        if a
            break
        end
    end
    if a
        currentLine = fgetl(f);
        continue
    end
    %% 空行
    if isempty(bsf)
        warning('空行')
        currentLine = fgetl(f);
        continue
    end
    %% 不识别标识符：
    error('不识别标识符：%s',bsf)
end
%% close
hold off
fclose(f);
clear ans a addMode bsf bsfNum bsfNum1 bsfNum2 currentLine czf f i lineColor n num R changeX changeY changeZ
msg = {'THE SIMULATION HAS BEEN COMPLETED';['片长(Y)总消耗：',num2str(Y),'mm']};
hour = floor(time / 3600);
minute = floor((time - hour*3600) / 60);
second = floor(time - hour*3600 - minute*60);
msgTime = '预计运行时间：';
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
输入参输：
currentLine：当前处理行总字符串
bsfNum：currentLine(1:bsfNum)为当前语句标识符
amount：设定预处理语句最大操作符数，若有X Y Z F，则取5

输出参输：
num标记位置为第一个操作符以及每个数字的最后一位
例：LINEAR X3.814386 F70.000000 
         X↑     6↑          ↑0
num所存为以上箭头所指字符在字符串中所在位置
%}
num = zeros(1,amount);
num(1) = bsfNum + 2;
a = 1;
for bsfNum = num(1):1:length(currentLine)
    if currentLine(bsfNum) == ' '
        a = a + 1;
        if a > amount
            error([moveMode,'指令中操作符过多'])
        end
        num(a) = bsfNum - 1;
    end
end
if currentLine(bsfNum) ~= ' '
    a = a + 1;
    if a > amount
        error([moveMode,'指令中操作符过多'])
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
        error('CW非识别操作符：%s',s1)
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
    error('CW移动轴超过两个')
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
输入参数：
x，y包含起点和终点坐标，r为半径
当r为正时，为劣弧
r为负时，为优弧
输出参数：
圆心坐标及半径绝对值
%}
p = sqrt((2*r/sqrt((x(1)-x(2))^2+(y(1)-y(2))^2))^2-1);
if r > 0%劣弧
    i = (x(1)+x(2))/2-(y(1)-y(2))*p;
    j = (y(1)+y(2))/2-(x(1)-x(2))*p;
    c = r;
else
    i = (x(1)+x(2))/2+(y(1)-y(2))*p;
    j = (y(1)+y(2))/2+(x(1)-x(2))*p;
    c = -r;
end
theta = [acos((x(1)-i)/c),acos((x(2)-i)/c)];
%由于matlab的acos只返回[0,pi]，而我们画弧需要2pi范围
if y(1) < j
    theta(1) = -theta(1);
end
if y(2) < j
    theta(2) = -theta(2);
end
end
function [theta,i,j,r] = IJ2theta_CW(x,y,i,j,addmode)
if ~addmode%相对运动
    i=x(1)+i;j=y(1)+j;
end
r = sqrt((i-x(1))^2+(j-y(1))^2);
theta = [acos((x(1)-i)/r),acos((x(2)-i)/r)];
%由于matlab的acos只返回[0,pi]，而我们画弧需要2pi范围
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
        error('G92非识别操作符：%s',s1)
end
end