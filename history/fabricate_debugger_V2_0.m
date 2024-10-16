%% 更新日志
%{
%{
V2.0
.重构整个框架逻辑结构
%}
%{
%{
V1.6
2022/07/19
.增加函数is_dwell，修复变向dwell判断
%}
%{
V1.5
2022/06/21
.加入以'pgmVariables.mat'命名，在当前文件夹下，缓存上次运行数据
.改善colorbar色阶
%}
%{
V1.4
2022/05/04
.加入变方向是否dwell判定
.修复CW、CCW画线
%}
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
%}
%% 版本检查
% if(exist('debuggerVersionCheck','file'))
%     if debuggerVersionCheck('1_5')
%         error('已更新，请重新打开')
%     end
% end
%% 保存提供加工程序的工作区
save('pgmVariables.mat')
clear all
%% 选取.pgm文件
[fileName,filePath]=uigetfile('*.pgm','Save File','');

if ~fileName
    error('未选取文件');
end
a = strfind(fileName,'.');
if ~a
    error('选取文件无“.”');
end
if fileName(max(a):end) ~= '.pgm'%#ok
    error('未选取.pgm文件');
end
%% 预设参数
global PointBefor PointNow PointG92 isABSOLUTE costY...	% 位置状态
    VelocityBefor VelocityNow Velocity_min Velocity_max notdwell ...    % 运动状态
    plotSwitch pgmF lineColorF lineWidth lineColor   % 绘图参数
rowPeriod = 1000;   % 进度条间隔，越大运行越快；减小可以观察画线
Velocity_max = 70;   % 若开光闸运行速度在最大与最小之外，线为黑色
Velocity_min = 20;
figure(1);clf;hold on;
view(0,90);%YOX
% view(90,0);%ZOY
% view(0,0);%ZOX
xlabel('X','Color','r');ylabel('Y','Color','r');zlabel('Z','Color','r');

%% 计算文本总行数
tic
f=fopen([filePath,fileName],'r');
rowTotal = 0;
while ~feof(f)
    rowTotal = rowTotal + sum(fread(f,10000,'char')==10);
end
fclose(f);
toc
fprintf('.pgm总行数%d\n',rowTotal);
%{
%% 找出速度最大值、最小值
clearvars -except filePath fileName
Velocity_max = [];
Velocity_min = [];
tic
f=fopen([filePath,fileName],'r');
for temp = 1:rowTotal
    currentLine = fgetl(f);    
    Velocity_temp = textscan(currentLine,'%*[^F] F%f');
    Velocity_temp = Velocity_temp{:};
    if ~isempty(Velocity_temp)
        Velocity_max = max(Velocity_temp,Velocity_max);
        Velocity_min = min(Velocity_temp,Velocity_min);
    end
end
fclose(f);
toc
fprintf('最大速度：%f\n最小速度：%f\n',Velocity_max,Velocity_min);
clearvars -except filePath fileName
Velocity_max = [];
Velocity_min = [];
tic
f=fopen([filePath,fileName],'r');
for temp = 1:rowTotal
    currentLine = fgetl(f);
    s = textscan(currentLine,'%s ');
    switch s{1,1}
        case 'PSOCONTROL',if s{1,2} == 'ON',Velocity_cmp = 1;else,Velocity_cmp = 0;end%#ok
        case {'LINEAR','CW','CCW'}
            Velocity_temp = textscan(currentLine,'%*[^F] F%f');
            Velocity_temp = Velocity_temp{:};
            if ~isempty(Velocity_temp)
                Velocity_max = max(Velocity_temp,Velocity_max);
                Velocity_min = min(Velocity_temp,Velocity_min);
            end
    end
end
toc
fprintf('最大速度：%f\n最小速度：%f\n',Velocity_max,Velocity_min);
%}
%% main
f=fopen([filePath,fileName],'r');
Category = {'LINEAR';'CW&CCW';'DWELL'};Time = zeros(length(Category),1);
time = table(Category,Time);time.Properties.VariableUnits = {'' 's'};
Category = {'LINEAR';'DWELL';'PSOCONTROL';'CW&CCW';'INCREMENTAL&ABSOLUTE';'Other'};
Count = zeros(length(Category),1);count = table(Category,Count);
rowNow = 0; % 当前行数
PointBefor=[0,0,0];PointNow=[0,0,0];PointG92=[0,0,0];costY=0;
VelocityBefor=[0,0,0];VelocityNow=[0,0,0];notdwell=[0,0,0];
plotSwitch=1;pgmF=nan;lineWidth=0.1;lineColor=[1 1 1];
lineColorF = colormap(cool(fix(Velocity_max - Velocity_min + 1)));
Velocity_min=Velocity_min-1;waitBar=waitbar(0,'1','name','SIMULATING...');
while ~feof(f)
    rowNow = rowNow + 1;
    if mod(rowNow,rowPeriod) == 0
        waitbar(rowNow/rowTotal,waitBar,sprintf('rowTotal=%d,rowNow=%d,%.3g%%',...
            rowTotal,rowNow,rowNow/rowTotal*100));
    end
    currentLine = fgetl(f);
    s = textscan(currentLine,'%s ');
    if isempty(s),continue;end % 空行
    switch s{1}{1}
        case 'LINEAR'
            time.Time(1)=time.Time(1)+LINEAR(s{1}(2:end)');count.Count(1)=count.Count(1)+1;
        case 'DWELL',time.Time(3)=time.Time(3)+str2double(s{1}(2));count.Count(2)=count.Count(2)+1;
            notdwell=[0,0,0];
        case 'PSOCONTROL',count.Count(3)=count.Count(3)+1;
            switch s{1}{3}
                case 'ON',lineColor = [0 0 0];lineWidth = 1.5;
                case 'OFF',lineColor = [1 1 0];lineWidth = 0.1;
                case 'RESET'
                otherwise,error([mfilename,'：操作符非法'],'PSOCONTROL:非法操作符<%s>',s{1}{3});
            end
        case 'CW',time.Time(2)=time.Time(2)+CW(0,s{1}(2:end)');count.Count(4)=count.Count(4)+1;
        case 'CCW',time.Time(2)=time.Time(2)+CW(1,s{1}(2:end)');count.Count(4)=count.Count(4)+1;
        case 'INCREMENTAL',isABSOLUTE=0;count.Count(5)=count.Count(5)+1;
        case 'ABSOLUTE',isABSOLUTE=1;count.Count(5)=count.Count(5)+1;
        case 'G92',G92(s{1}(2:end)');count.Count(6)=count.Count(6)+1;
        case {'G359','ENABLE','METRIC','SECONDS','VELOCITY','PSOOUTPUT'},count.Count(6)=count.Count(6)+1;
        otherwise
            if currentLine(1) == "'"    % 注释行
                if strcmp(s{1}{1},"'plotSwitch")   %绘制判断
                    switch s{1}{2}
                        case '1',plotSwitch=1;
                        case '0',plotSwitch=0;
                        otherwise,error("'plotSwitch非识别操作符：%s",s{1}{2});
                    end
                end
                continue;
            end
            error([mfilename,'：标识符非法'],[currentLine,'\n非法标识符<%s>，line %d'],s{1}{1},rowNow);
    end
end
%% close
fclose(f);delete(waitBar);hold off;
clearvars -except costY time count Velocity_max Velocity_min;
% colorbar
numV = Velocity_max - Velocity_min;
Velocity_min = Velocity_min + 1;
if numV > 10,c = colorbar('Ticks',0:0.1:1,'TickLabels',{fix(linspace(Velocity_min,Velocity_max,11))});
else,c = colorbar('Ticks',linspace(0,1,numV),'TickLabels',{fix(linspace(Velocity_min,Velocity_max,numV))});
end
c.Label.String = '开光闸的运动速度';
% msgbox
msg = {'THE SIMULATION HAS BEEN COMPLETED';['片长(Y)总消耗：',num2str(costY),'mm']};
time_all = sum(time.Time);
hour = floor(time_all / 3600);minute = floor((time_all - hour*3600) / 60);second = floor(time_all - hour*3600 - minute*60);
msgTime = '预计加工时间：';
if hour,msgTime = [msgTime,num2str(hour),'h'];end
if minute,msgTime = [msgTime,num2str(minute),'m'];end
if second,msgTime = [msgTime,num2str(second),'s'];end
msg(3,1) = {msgTime};msgbox(msg, 'Success');
figure(2);
subplot(1,2,1);pie(count.Count,count.Category);title('Count','Color','red');
subplot(1,2,2);pie(time.Time,time.Category);title('Count','Color','red');
% temp=tiledlayout(1,2,'TileSpacing','compact');
% ax1=nexttile;pie(ax1,count.Count);title(Count);
% ax2=nexttile;pie(ax2,time.Time);title(Time);
% lgd=legend(count.Category);lgd.Layout.Tile = 'Summary';
% load & delete 'pgmVariables.mat'
fabricate_debugger_costY = msg{2};
fabricate_debugger_costTime = msg{3};
clearvars -except fabricate_debugger_costY fabricate_debugger_costTime
load('pgmVariables.mat');delete('pgmVariables.mat');

%% function
function SWITCH(s)
global PointNow PointG92 isABSOLUTE pgmF pgmI pgmJ pgmR
temp = str2double(s(2:end));
switch s(1)
    case 'X'
        if isABSOLUTE,PointNow(1)=temp+PointG92(1);
        else,PointNow(1)=temp+PointNow(1);
        end
    case 'Y'
        if isABSOLUTE,PointNow(2)=temp+PointG92(2);
        else,PointNow(2)=temp+PointNow(2);
        end
    case 'Z'
        if isABSOLUTE,PointNow(3)=temp+PointG92(3);
        else,PointNow(3)=temp+PointNow(3);
        end
    case 'F',pgmF=temp;
    case 'I',pgmI=temp;
    case 'J',pgmJ=temp;
    case 'R',pgmR=temp;
    otherwise,error('SWITCH:未能识别%f',s(1));
end
end
function G92(s)
global PointG92 PointNow
for temp = s
    switch temp
        case 'X',PointG92(1)=PointNow(1) - str2double(s(2:end));
        case 'Y',PointG92(2)=PointNow(2) - str2double(s(2:end));
        case 'Z',PointG92(3)=PointNow(3) - str2double(s(2:end));
        otherwise,error([mfilename,'：操作符非法'],'G92:非法操作符<%f>',s(1));
    end
end
end

function time = LINEAR(s)
global PointBefor PointNow costY...	% 位置状态
    VelocityBefor VelocityNow Velocity_min Velocity_max notdwell ...	% 运动状态
    plotSwitch pgmF lineColorF lineWidth lineColor	% 绘图参数
PointBefor = PointNow;VelocityBefor = VelocityNow;  % 保存前一状态
for temp=s,SWITCH(temp{:});end
if PointNow(2)<costY;costY=PointNow(2);end
VelocityNow=PointNow-PointBefor;
L=norm(VelocityNow);    % 移动距离
VelocityNow=VelocityNow/norm(VelocityNow);
% 判断前一状态与现在状态是否符合物理逻辑关系
notdwell = is_dwell(VelocityBefor,VelocityNow,notdwell);
% 画图
if plotSwitch
    temp = plot3([PointBefor(1),PointNow(1)],[PointBefor(2),PointNow(2)],...
        [PointBefor(3),PointNow(3)],'.-','LineWidth',lineWidth);
    lineColorFNow = fix(pgmF - Velocity_min);
    if lineColorFNow>Velocity_min&&lineColorFNow<Velocity_max
        temp.Color = lineColorF(lineColorFNow,:);
    else,temp.Color = lineColor;
    end
end
time = L/pgmF;
end
function time = CW(isCCW,s)
global PointBefor PointNow pgmI pgmJ pgmR costY...	% 位置状态
    VelocityBefor VelocityNow Velocity_min Velocity_max notdwell ...	% 运动状态
    plotSwitch pgmF lineColorF lineWidth lineColor	% 绘图参数
curveNum = 9;  % curveNum-1段直线描述一段弧
PointBefor = PointNow;VelocityBefor = VelocityNow;  % 保存前一状态
for temp = s,SWITCH(temp{:});end
PB2PE = PointNow - PointBefor;
Vertical = cross(VelocityBefor,PB2PE);
Vertical = Vertical/norm(Vertical);
% if isCCW;Vertical=-Vertical;end
if isempty(pgmR)
    % 输入为IJ，I对应指定终点第一个轴起始坐标的相对偏移量
    O = PointBefor;
    notdwell = zeros(1,3);
    switch s{1}(1)
        case 'X',O(1) = O(1)+pgmI;notdwell(1)=1;
        case 'Y',O(2) = O(2)+pgmI;notdwell(2)=1;
        case 'Z',O(3) = O(3)+pgmI;notdwell(3)=1;
    end
    switch s{2}(1)
        case 'X',O(1) = O(1)+pgmJ;notdwell(1)=1;
        case 'Y',O(2) = O(2)+pgmJ;notdwell(2)=1;
        case 'Z',O(3) = O(3)+pgmJ;notdwell(3)=1;
    end
    % 半径为起始位置与指定偏移量之间的距离
    pgmR = norm(O - PointBefor);
else    % 输入为R
    notdwell = zeros(1,3);
    % 运动轴置为未dwell
    switch s{1}(1)
        case 'X',notdwell(1)=1;
        case 'Y',notdwell(2)=1;
        case 'Z',notdwell(3)=1;
    end
    switch s{2}(1)
        case 'X',notdwell(1)=1;
        case 'Y',notdwell(2)=1;
        case 'Z',notdwell(3)=1;
    end
    Fr1 = cross(VelocityBefor,Vertical);	% 起点-R*向量Fr1为圆心
    O = PointBefor - pgmR*Fr1;
    pgmR = norm(O - PointBefor);
end
n1=(PointBefor-O)/pgmR;n2=cross(Vertical,n1); %参数方程径向向量
O2PE=PointNow - O;
theta=atan2(dot(O2PE,n2),dot(O2PE,n1));
% if theta<0,theta=theta+2*pi;end
% if isCCW,theta=2*pi-theta;end
T = [n1',n2',O']; % 坐标系变换矩阵
VelocityNow = (T*[sin(theta);cos(theta);0])';
% 画图
if plotSwitch
    if isCCW,t = linspace(theta,0,curveNum);else,t = linspace(0,theta,curveNum);end
    A = [pgmR*cos(t);pgmR*sin(t);ones(1,curveNum)];	% 圆平面坐标系的参数方程[x,y,1]
    B = T*A;    % 局部坐标系{X'OY'}转换到{XYZ}
    x = B(1,:);y = B(2,:);z = B(3,:);
    if any(y<costY);costY=min(y(y<costY));end
    temp = plot3(x,y,z,'-','LineWidth',lineWidth);
    temp2 = plot3([x(1) x(end)],[y(1) y(end)],[z(1) z(end)],'.'); % 首尾两点标记
    lineColorFNow = fix(pgmF - Velocity_min);
    if lineColorFNow>Velocity_min&&lineColorFNow<Velocity_max
        temp.Color = lineColorF(lineColorFNow,:);
    else,temp.Color = lineColor;
    end,temp2.Color = temp.Color;
end
time = abs(theta)*pi/pgmF;
pgmR=[];pgmI=[];pgmJ=[];
end

function notdwell = is_dwell(v0,v1,notdwell)
%{
判断3轴大幅度变相前是否dwell

输入参输：
v0：前一状态末速度
v1：现状态初速度
notdwell：三轴dwell情况，1--未dewell
输出结果：
若大幅变相未dwell，报错轴、行数、行内容
无误则更新notdwell
%}
ismove = ones(1,3);
for temp = find(v1==0)  % 认为最终速度和起始速度都为0的轴为未运动轴
    if ~v0(temp) % 未运动轴
        ismove(temp) = 0;
    end
end
if any(notdwell&ismove)	% 运动轴未dwell
    if cross(v0,v1) > 0.1 % 夹角大于5.74°
        % 可以报错了
        global currentLine rowNow %#ok
        temp = {'x','y','z'};temp = temp(notdwell==1);temp = [temp{:}];
        error(['变向',temp,'未DWELL：%s，line %d，请尝试',...
            '调大程序的判断条件0.1，或直接备注掉is_dwell函数'],currentLine,rowNow)
    end
end
notdwell = ismove;
end