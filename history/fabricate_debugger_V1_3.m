clear
%% 更新日志
%{
%{
V1.3
2021/12/1
.改善标识符识别逻辑
.加入是否画图，.pgm中的"'plotSwitch 1"即为开启画图，"'plotSwitch 0"为关闭，默认开启
.加入X、Y、Z轴标识
.改善costY变量以存储最大Y值（之前直接用最后的Y做为使用量
.加入进度条，可以在line 47中更改刷新间隔行数，默认为1000行
.加入速度颜色线，色阶为cool，若颜色变化不明显可在line 48、49中更改最大速度及最小...
速度，默认为100mm/s及0，最大速度设置过低会报错，当前速度低于最小值会画黑线
%}
%{
V1.2
2021/10/20
add:
.增加版本更新功能，基于在matlab的搜索路径中加入函数debuggerVersionCheck()实现
.增加.pgm注释识别，欲在新版本debugger中实现部分绘图功能
.增加.pgm程序语句使用情况计数
change:
.figure()->clf，不新建图窗
remove:
.移除画图首句plot3(0,0,0)，以便于平面观察非3D图
%}
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
%}
%% 进度条刷新间隔行数及最大速度
rowPeriod = 1000;
maxVelocity = 100;
minVelocity = 0;
%% 版本检查
if(exist('debuggerVersionCheck','file'))
    if debuggerVersionCheck('1_3')
        error('已更新，请重新打开')
    end
end
%% 选取.pgm文件
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


%% initial
clf;hold on;
xlabel('X','Color','r');ylabel('Y','Color','r');zlabel('Z','Color','r');
f=fopen([filePath,fileName],'r');
% 计算文本总行数
rowTotal = 0;
rowNow = 0;
while ~feof(f)
    rowTotal = rowTotal + sum(fread(f,10000,'char')==10);
end
fclose(f);
f=fopen([filePath,fileName],'r');
clear filePath fileName
time = 0;
global X Y Z x0 y0 z0 costY
X=0;Y=0;Z=0;x0=0;y0=0;z0=0;costY=0;
global F changeX changeY changeZ addMode lineColor lineWidth R I J
F=0;changeX=0;changeY=0;changeZ=0;R=0;I=0;J=0;lineColor='y';lineWidth=0.1;
global changeX_G92 changeY_G92 changeZ_G92 plotSwitch
changeX_G92=0;changeY_G92=0;changeZ_G92=0;plotSwitch=1;
global lineColorF
% if maxVelocity - minVelocity > 100
lineColorF = colormap(cool(fix(maxVelocity - minVelocity + 1)));
% else
%     lineColorF = colormap(cool(100));
% end
minVelocity = minVelocity - 1;
%语句计数，[LINEAR,PSOCONTROL,DWELL,绝对运动/相对运动,CW/CCW,G92,配置语句]
count=zeros(1,7);
waitBar = waitbar(0,'1','name','SIMULATING...');

%% main
currentLine = fgetl(f);
while ischar(currentLine)
    rowNow = rowNow + 1;
    if mod(rowNow,rowPeriod) == 0
        waitbar(rowNow/rowTotal,waitBar,sprintf('%.2g%%',rowNow/rowTotal*100));
    end
    %% 找到当前语句标识符
    s = strsplit(currentLine);
    s(cellfun(@isempty,s))=[];  %除空cell
    %% 空行
    if isempty(s)
        currentLine = fgetl(f);
        continue
    end
	%% 注释行 & 绘制判断
    if currentLine(1) == "'"
        if strcmp(s{1},"'plotSwitch")   %绘制判断
            switch s{2}
                case "1"
                    plotSwitch=1;
                case "0"
                    plotSwitch=0;
                otherwise
                    error("'plotSwitchCW非识别操作符：%s",s{2})
            end
        end
        currentLine = fgetl(f);
        continue
    end
    bsf = s{1};
    %% LINEAR
    %{
    用num存储标记第一个轴字母以及每个距离的最后一位的位置
    判断操作符为X、Y、Z、F，并将值置于相应全局变量中
    画图并计时
    %}
    if strcmp(bsf,'LINEAR')
        %最多操作符为X Y Z F，一条语句中操作符数目至多4个
        [czf,num,a] = get_aixs_dis(s,currentLine,4);
        for i = 1:a
            switch_LINEAR(czf{i},num(i));
        end
        %画图并计时
        count(1)=count(1)+1;
        if fix(F) > maxVelocity
            opentoline([pwd,filesep,'fabricate_debugger_V1_3.m'],48,0)
            error('最大速度设置过低当前F = %d，maxVelocity = %d',fix(F),fix(maxVelocity))
        end
        time = time + move_LINEAR(fix(F - minVelocity));
        if isnan(X)
            error('!')
        end
        currentLine = fgetl(f);
        continue
    end
    %% PSOCONTROL
    %{
    取'PSOCONTROL X '之后的操作符字符串并进行除空格后
    可能剩'ON'、'OFF'、'RESET'
    %}
    if strcmp(bsf,'PSOCONTROL')
        if length(s) ~= 3
            error('elements num do not match in PSOCONTROL:%s',currentLine);
        end
        %取'PSOCONTROL X '之后的操作符字符串并进行除空格
        czf = s{3};
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
        count(2)=count(2)+1;
        currentLine = fgetl(f);
        continue
    end
    %% DWELL
    if strcmp(bsf,'DWELL')
        if length(s) ~= 2
            error('elements num do not match in DWELL:%s',currentLine);
        end
        %计时
        count(3)=count(3)+1;
        time = time + str2double(s{2});
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
        count(4)=count(4)+1;
        currentLine = fgetl(f);
        continue
    end
    %% CW
    if strcmp(bsf,'CW')
        %最多操作符有5个，如X Y I J F，位置标记至多5个
        [czf,num,a] = get_aixs_dis(s,currentLine,5);
        for i = 1:a
            switch_CW(czf{i},num(i));
        end
        %画图并计时
        count(5)=count(5)+1;
        if fix(F) > maxVelocity
            opentoline([pwd,filesep,'fabricate_debugger_V1_3.m'],48,0)
            error('最大速度设置过低当前F = %d，maxVelocity = %d',fix(F),fix(maxVelocity))
        end
        time = time + move_CW(1,fix(F - minVelocity));
        currentLine = fgetl(f);
        continue
    end
    %% CCW
    if strcmp(bsf,'CCW')
        %最多操作符有5个，如X Y I J F，位置标记至多5个
        [czf,num,a] = get_aixs_dis(s,currentLine,5);
        %比较num(i)+2；将后面的数置于全局变量中
        for i = 1:a
            switch_CW(czf{i},num(i));
        end
        %画图并计时
        count(5)=count(5)+1;
        if fix(F) > maxVelocity
            opentoline([pwd,filesep,'fabricate_debugger_V1_3.m'],48,0)
            error('最大速度设置过低当前F = %d，maxVelocity = %d',fix(F),fix(maxVelocity))
        end
        time = time + move_CW(0,fix(F - minVelocity));
        currentLine = fgetl(f);
        continue
    end
    %% G92
    if strcmp(bsf,'G92')
        %最多操作符有3个，如X Y Z，位置标记至多3个
        [czf,num,a] = get_aixs_dis(s,currentLine,3);
        for i = 1:a
            switch_G92(czf{i},num(i));
        end
        count(6)=count(6)+1;
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
        count(7)=count(7)+1;
        currentLine = fgetl(f);
        continue
    end
    %% 不识别标识符：
    error([currentLine,'——不识别标识符：%s'],bsf)
end
%% close
minVelocity = minVelocity + 1;
c = colorbar('Ticks',0:0.1:1,'TickLabels',{fix(linspace(minVelocity,maxVelocity,11))});
c.Label.String = '开光闸的运动速度';
clear c lineColorF plotSwitch rowNow rowTotal
hold off
fclose(f);
clear a addMode bsf currentLine czf f i lineColor lineWidth R I J n num changeX changeY changeZ changeX_G92 changeY_G92 changeZ_G92 x0 y0 z0
delete(waitBar);
msg = {'THE SIMULATION HAS BEEN COMPLETED';['片长(Y)总消耗：',num2str(costY),'mm']};
hour = floor(time / 3600);
minute = floor((time - hour*3600) / 60);
second = floor(time - hour*3600 - minute*60);
msgTime = '预计加工时间：';
if hour
    msgTime = [msgTime,num2str(hour),'h'];
end
if minute
    msgTime = [msgTime,num2str(minute),'m'];
end
if second
    msgTime = [msgTime,num2str(second),'s'];
end
clear second minute hour
msg(3,1) = {msgTime};
% msg(4:12,1) = {'';'程序语句使用情况：';['LINEAR：',num2str(count(1))];...
%     ['PSOCONTROL：',num2str(count(2))];['DWELL：',num2str(count(3))];...
%     ['绝对运动/相对运动：',num2str(count(4))];['CW/CCW：',num2str(count(5))];...
%     ['G92：',num2str(count(6))];['配置语句：',num2str(count(7))]};
msgbox(msg, 'Success');
%clear msg

%% function
function [czf,num,a] = get_aixs_dis(s,currentLine,amount)
%{
输入参输：
s：currentLine中的元素
currentLine：当前处理行字符串
amount：预处理指令所支持的最大操作符数

输出参输：
czf：指令操作符
num：操作符后数字
a：操作符数目
例：currentLine = 'LINEAR X10.160903 F40.000000 '
num=[3.8144,70];czf={'X','F'};a=2;
%}
a = length(s) - 1;
if a == 0
    error([currentLine,'—无操作符'])
end
if a > amount
    error([currentLine,'—操作符过多'])
end
num = zeros(1,a);
czf = cell(1,a);
s = s(2:end);
for i = 1:a
    num(i) = str2double(s{i}(2:end));
    czf(i) = {s{i}(1)};
end
end

function switch_LINEAR(s1,num)
global addMode
switch s1
    case 'X'
        global changeX X changeX_G92%#ok
        changeX = num;
        if addMode
            X = changeX + changeX_G92;
        else
            X = X + changeX;
        end
    case 'Y'
        global changeY Y changeY_G92 costY%#ok
        changeY = num;
        if addMode
            Y = changeY + changeY_G92;
        else
            Y = Y + changeY;
        end
        if Y < costY
            costY = Y;
        end
    case 'Z'
        global changeZ Z changeZ_G92%#ok
        changeZ = num;
        if addMode
            Z = changeZ + changeZ_G92;
        else
            Z = Z + changeZ;
        end
    case 'F'
        global F%#ok
        F = num;
end
end
function time_LINEAR = move_LINEAR(lineColorFNow)
global F X Y Z lineColor lineWidth x0 y0 z0 plotSwitch lineColorF
if plotSwitch
    if lineColor == 'y'
        plot3([x0,X],[y0,Y],[z0,Z],[lineColor,'.-'],'linewidth',lineWidth);
    elseif lineColorFNow > 0
        plot3([x0,X],[y0,Y],[z0,Z],'.-','color',lineColorF(lineColorFNow,:),'linewidth',lineWidth);
    else
        plot3([x0,X],[y0,Y],[z0,Z],[lineColor,'.-'],'linewidth',lineWidth);
    end
end
time_LINEAR = sqrt((x0-X)^2+(y0-Y)^2+(z0-Z)^2)/F;
x0=X;y0=Y;z0=Z;
end

function switch_CW(s1,num)
global addMode
switch s1
    case 'X'
        global changeX X changeX_G92%#ok
        changeX = num;
        if addMode
            X = changeX + changeX_G92;
        else
            X = X + changeX;
        end
    case 'Y'
        global changeY Y changeY_G92%#ok
        changeY = num;
        if addMode
            Y = changeY + changeY_G92;
        else
            Y = Y + changeY;
        end
    case 'I'
        global I%#ok
        I = num;
    case 'J'
        global J%#ok
        J = num;
    case 'F'
        global F%#ok
        F = num;
    case 'Z'
        global changeZ Z changeZ_G92%#ok
        changeZ = num;
        if addMode
            Z = changeZ + changeZ_G92;
        else
            Z = Z + changeZ;
        end
    case 'R'
        global R%#ok
        R = num;
    otherwise
        error('CW非识别操作符：%s',s1)
end
end
function time_CW = move_CW(mode,lineColorFNow)
global F addMode X Y Z lineColor lineWidth I J R x0 y0 z0 plotSwitch lineColorF
%% plotArray
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
else
    error('CW移动轴超过两个')
end
%% plot3
if plotSwitch
    if lineColor == 'y'
        plot3([x0,X],[y0,Y],[z0,Z],[lineColor,'.'],'linewidth',lineWidth)
        plot3(plotArray(1,:),plotArray(2,:),plotArray(3,:),[lineColor,'-'],'linewidth',lineWidth)
    elseif lineColorFNow > 0
        plot3([x0,X],[y0,Y],[z0,Z],'.-','color',lineColorF(lineColorFNow,:),'linewidth',lineWidth)
        plot3(plotArray(1,:),plotArray(2,:),plotArray(3,:),'.-','color',lineColorF(lineColorFNow,:),'linewidth',lineWidth)
    else
        plot3([x0,X],[y0,Y],[z0,Z],[lineColor,'.'],'linewidth',lineWidth)
        plot3(plotArray(1,:),plotArray(2,:),plotArray(3,:),[lineColor,'-'],'linewidth',lineWidth)
    end
end
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

function switch_G92(s1,num)
switch s1
    case 'X'
        global X changeX_G92%#ok
        changeX_G92 = X - num;
    case 'Y'
        global Y changeY_G92%#ok
        changeY_G92 = Y - num;
    case 'Z'
        global Z changeZ_G92%#ok
        changeZ_G92 = Z - num;
    otherwise
        error('G92非识别操作符：%s',s1)
end
end