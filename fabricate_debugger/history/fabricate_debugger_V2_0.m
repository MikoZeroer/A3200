%% ������־
%{
%{
V2.0
.�ع���������߼��ṹ
%}
%{
%{
V1.6
2022/07/19
.���Ӻ���is_dwell���޸�����dwell�ж�
%}
%{
V1.5
2022/06/21
.������'pgmVariables.mat'�������ڵ�ǰ�ļ����£������ϴ���������
.����colorbarɫ��
%}
%{
V1.4
2022/05/04
.����䷽���Ƿ�dwell�ж�
.�޸�CW��CCW����
%}
%{
V1.3
2021/12/1
.���Ʊ�ʶ��ʶ���߼�
.�����Ƿ�ͼ��.pgm�е�"'plotSwitch 1"��Ϊ������ͼ��"'plotSwitch 0"Ϊ�رգ�Ĭ�Ͽ���
.����X��Y��Z���ʶ
.����costY�����Դ洢���Yֵ��֮ǰֱ��������Y��Ϊʹ����
.�����������������line 47�и���ˢ�¼��������Ĭ��Ϊ1000��
.�����ٶ���ɫ�ߣ�ɫ��Ϊcool������ɫ�仯�����Կ���line 48��49�и�������ٶȼ���С...
�ٶȣ�Ĭ��Ϊ100mm/s��0������ٶ����ù��ͻᱨ����ǰ�ٶȵ�����Сֵ�ử����
%}
%{
V1.2
2021/10/20
add:
.���Ӱ汾���¹��ܣ�������matlab������·���м��뺯��debuggerVersionCheck()ʵ��
.����.pgmע��ʶ�������°汾debugger��ʵ�ֲ��ֻ�ͼ����
.����.pgm�������ʹ���������
change:
.figure()->clf�����½�ͼ��
remove:
.�Ƴ���ͼ�׾�plot3(0,0,0)���Ա���ƽ��۲��3Dͼ
%}
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
%}
%}
%% �汾���
% if(exist('debuggerVersionCheck','file'))
%     if debuggerVersionCheck('1_5')
%         error('�Ѹ��£������´�')
%     end
% end
%% �����ṩ�ӹ�����Ĺ�����
save('pgmVariables.mat')
clear all
%% ѡȡ.pgm�ļ�
[fileName,filePath]=uigetfile('*.pgm','Save File','');

if ~fileName
    error('δѡȡ�ļ�');
end
a = strfind(fileName,'.');
if ~a
    error('ѡȡ�ļ��ޡ�.��');
end
if fileName(max(a):end) ~= '.pgm'%#ok
    error('δѡȡ.pgm�ļ�');
end
%% Ԥ�����
global PointBefor PointNow PointG92 isABSOLUTE costY...	% λ��״̬
    VelocityBefor VelocityNow Velocity_min Velocity_max notdwell ...    % �˶�״̬
    plotSwitch pgmF lineColorF lineWidth lineColor   % ��ͼ����
rowPeriod = 1000;   % �����������Խ������Խ�죻��С���Թ۲컭��
Velocity_max = 70;   % ������բ�����ٶ����������С֮�⣬��Ϊ��ɫ
Velocity_min = 20;
figure(1);clf;hold on;
view(0,90);%YOX
% view(90,0);%ZOY
% view(0,0);%ZOX
xlabel('X','Color','r');ylabel('Y','Color','r');zlabel('Z','Color','r');

%% �����ı�������
tic
f=fopen([filePath,fileName],'r');
rowTotal = 0;
while ~feof(f)
    rowTotal = rowTotal + sum(fread(f,10000,'char')==10);
end
fclose(f);
toc
fprintf('.pgm������%d\n',rowTotal);
%{
%% �ҳ��ٶ����ֵ����Сֵ
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
fprintf('����ٶȣ�%f\n��С�ٶȣ�%f\n',Velocity_max,Velocity_min);
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
fprintf('����ٶȣ�%f\n��С�ٶȣ�%f\n',Velocity_max,Velocity_min);
%}
%% main
f=fopen([filePath,fileName],'r');
Category = {'LINEAR';'CW&CCW';'DWELL'};Time = zeros(length(Category),1);
time = table(Category,Time);time.Properties.VariableUnits = {'' 's'};
Category = {'LINEAR';'DWELL';'PSOCONTROL';'CW&CCW';'INCREMENTAL&ABSOLUTE';'Other'};
Count = zeros(length(Category),1);count = table(Category,Count);
rowNow = 0; % ��ǰ����
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
    if isempty(s),continue;end % ����
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
                otherwise,error([mfilename,'���������Ƿ�'],'PSOCONTROL:�Ƿ�������<%s>',s{1}{3});
            end
        case 'CW',time.Time(2)=time.Time(2)+CW(0,s{1}(2:end)');count.Count(4)=count.Count(4)+1;
        case 'CCW',time.Time(2)=time.Time(2)+CW(1,s{1}(2:end)');count.Count(4)=count.Count(4)+1;
        case 'INCREMENTAL',isABSOLUTE=0;count.Count(5)=count.Count(5)+1;
        case 'ABSOLUTE',isABSOLUTE=1;count.Count(5)=count.Count(5)+1;
        case 'G92',G92(s{1}(2:end)');count.Count(6)=count.Count(6)+1;
        case {'G359','ENABLE','METRIC','SECONDS','VELOCITY','PSOOUTPUT'},count.Count(6)=count.Count(6)+1;
        otherwise
            if currentLine(1) == "'"    % ע����
                if strcmp(s{1}{1},"'plotSwitch")   %�����ж�
                    switch s{1}{2}
                        case '1',plotSwitch=1;
                        case '0',plotSwitch=0;
                        otherwise,error("'plotSwitch��ʶ���������%s",s{1}{2});
                    end
                end
                continue;
            end
            error([mfilename,'����ʶ���Ƿ�'],[currentLine,'\n�Ƿ���ʶ��<%s>��line %d'],s{1}{1},rowNow);
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
c.Label.String = '����բ���˶��ٶ�';
% msgbox
msg = {'THE SIMULATION HAS BEEN COMPLETED';['Ƭ��(Y)�����ģ�',num2str(costY),'mm']};
time_all = sum(time.Time);
hour = floor(time_all / 3600);minute = floor((time_all - hour*3600) / 60);second = floor(time_all - hour*3600 - minute*60);
msgTime = 'Ԥ�Ƽӹ�ʱ�䣺';
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
    otherwise,error('SWITCH:δ��ʶ��%f',s(1));
end
end
function G92(s)
global PointG92 PointNow
for temp = s
    switch temp
        case 'X',PointG92(1)=PointNow(1) - str2double(s(2:end));
        case 'Y',PointG92(2)=PointNow(2) - str2double(s(2:end));
        case 'Z',PointG92(3)=PointNow(3) - str2double(s(2:end));
        otherwise,error([mfilename,'���������Ƿ�'],'G92:�Ƿ�������<%f>',s(1));
    end
end
end

function time = LINEAR(s)
global PointBefor PointNow costY...	% λ��״̬
    VelocityBefor VelocityNow Velocity_min Velocity_max notdwell ...	% �˶�״̬
    plotSwitch pgmF lineColorF lineWidth lineColor	% ��ͼ����
PointBefor = PointNow;VelocityBefor = VelocityNow;  % ����ǰһ״̬
for temp=s,SWITCH(temp{:});end
if PointNow(2)<costY;costY=PointNow(2);end
VelocityNow=PointNow-PointBefor;
L=norm(VelocityNow);    % �ƶ�����
VelocityNow=VelocityNow/norm(VelocityNow);
% �ж�ǰһ״̬������״̬�Ƿ���������߼���ϵ
notdwell = is_dwell(VelocityBefor,VelocityNow,notdwell);
% ��ͼ
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
global PointBefor PointNow pgmI pgmJ pgmR costY...	% λ��״̬
    VelocityBefor VelocityNow Velocity_min Velocity_max notdwell ...	% �˶�״̬
    plotSwitch pgmF lineColorF lineWidth lineColor	% ��ͼ����
curveNum = 9;  % curveNum-1��ֱ������һ�λ�
PointBefor = PointNow;VelocityBefor = VelocityNow;  % ����ǰһ״̬
for temp = s,SWITCH(temp{:});end
PB2PE = PointNow - PointBefor;
Vertical = cross(VelocityBefor,PB2PE);
Vertical = Vertical/norm(Vertical);
% if isCCW;Vertical=-Vertical;end
if isempty(pgmR)
    % ����ΪIJ��I��Ӧָ���յ��һ������ʼ��������ƫ����
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
    % �뾶Ϊ��ʼλ����ָ��ƫ����֮��ľ���
    pgmR = norm(O - PointBefor);
else    % ����ΪR
    notdwell = zeros(1,3);
    % �˶�����Ϊδdwell
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
    Fr1 = cross(VelocityBefor,Vertical);	% ���-R*����Fr1ΪԲ��
    O = PointBefor - pgmR*Fr1;
    pgmR = norm(O - PointBefor);
end
n1=(PointBefor-O)/pgmR;n2=cross(Vertical,n1); %�������̾�������
O2PE=PointNow - O;
theta=atan2(dot(O2PE,n2),dot(O2PE,n1));
% if theta<0,theta=theta+2*pi;end
% if isCCW,theta=2*pi-theta;end
T = [n1',n2',O']; % ����ϵ�任����
VelocityNow = (T*[sin(theta);cos(theta);0])';
% ��ͼ
if plotSwitch
    if isCCW,t = linspace(theta,0,curveNum);else,t = linspace(0,theta,curveNum);end
    A = [pgmR*cos(t);pgmR*sin(t);ones(1,curveNum)];	% Բƽ������ϵ�Ĳ�������[x,y,1]
    B = T*A;    % �ֲ�����ϵ{X'OY'}ת����{XYZ}
    x = B(1,:);y = B(2,:);z = B(3,:);
    if any(y<costY);costY=min(y(y<costY));end
    temp = plot3(x,y,z,'-','LineWidth',lineWidth);
    temp2 = plot3([x(1) x(end)],[y(1) y(end)],[z(1) z(end)],'.'); % ��β������
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
�ж�3�����ȱ���ǰ�Ƿ�dwell

������䣺
v0��ǰһ״̬ĩ�ٶ�
v1����״̬���ٶ�
notdwell������dwell�����1--δdewell
��������
���������δdwell�������ᡢ������������
���������notdwell
%}
ismove = ones(1,3);
for temp = find(v1==0)  % ��Ϊ�����ٶȺ���ʼ�ٶȶ�Ϊ0����Ϊδ�˶���
    if ~v0(temp) % δ�˶���
        ismove(temp) = 0;
    end
end
if any(notdwell&ismove)	% �˶���δdwell
    if cross(v0,v1) > 0.1 % �нǴ���5.74��
        % ���Ա�����
        global currentLine rowNow %#ok
        temp = {'x','y','z'};temp = temp(notdwell==1);temp = [temp{:}];
        error(['����',temp,'δDWELL��%s��line %d���볢��',...
            '���������ж�����0.1����ֱ�ӱ�ע��is_dwell����'],currentLine,rowNow)
    end
end
notdwell = ismove;
end