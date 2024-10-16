% ������־
%{
V2.2
2023/4/26
.��������.pgm�ӹ����ƣ��ٶ�colorbar������ǰ�����ϸ������ػ�
.���뻭ͼ���
.����figure1��figure2���У�����figure2����figure1��λ���ǽ������Ҳ�
.�޸�0.x�ٶȵĻ�ͼ��ɫ
V2.1
.�޸�CW������ɫ
.�޸�notdwell�ж�
.�޸�pieͼtime��Ŀ�ı�
.�޸�LINEARָ��δ�˶�����
V2.0
.�ع�
V1.6
2022/07/19
.���Ӻ���is_dwell���޸�����dwell�ж�
V1.5
2022/06/21
.������'pgmVariables.mat'�������ڵ�ǰ�ļ����£������ϴ���������
.����colorbarɫ��
V1.4
2022/05/04
.����䷽���Ƿ�dwell�ж�
.�޸�CW��CCW����
V1.3
2021/12/1
.���Ʊ�ʶ��ʶ���߼�
.�����Ƿ�ͼ��.pgm�е�"'plotSwitch 1"��Ϊ������ͼ��"'plotSwitch 0"Ϊ�رգ�Ĭ�Ͽ���
.����X��Y��Z���ʶ
.����costY�����Դ洢���Yֵ��֮ǰֱ��������Y��Ϊʹ����
.�����������������line 47�и���ˢ�¼��������Ĭ��Ϊ1000��
.�����ٶ���ɫ�ߣ�ɫ��Ϊcool������ɫ�仯�����Կ���line 48��49�и�������ٶȼ���С...
�ٶȣ�Ĭ��Ϊ100mm/s��0������ٶ����ù��ͻᱨ����ǰ�ٶȵ�����Сֵ�ử����
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
%% Ԥ�����
global Velocity_min Velocity_max dtime  %#ok
rowPeriod = 100;   % �����������Խ������Խ�죻��С���Թ۲컭��
Velocity_max = [];	% ������բ�����ٶ����������С֮�⣬��Ϊ��ɫ
Velocity_min = [];	% ��Ϊ[]���Զ���������բ�ٶ�
dtime = 0;  % ��ͼ���
continue_fabricata = 0; % ��Ϊ1�����������ӹ�ģʽ,��ͼcolorbar�����ݵ�ǰ����

%% �����ṩ�ӹ�����Ĺ�����
if ~continue_fabricata
    save('temp.mat',"Velocity_min","Velocity_max","dtime","rowPeriod","continue_fabricata")
    clearvars Velocity_min Velocity_max dtime rowPeriod continue_fabricata
    save('pgmVariables.mat')
    clear
    load("temp.mat")
    delete("temp.mat")
end
%% ѡȡ.pgm�ļ�
dbclear if error
[fileName,filePath]=uigetfile('*.pgm','Save File','');

if ~fileName,error('δѡȡ�ļ�');end
a = strfind(fileName,'.');
if ~a,error('ѡȡ�ļ��ޡ�.��');end
if fileName(max(a):end) ~= '.pgm',error('δѡȡ.pgm�ļ�');end%#ok
dbstop if error
%% �����ı�������
tic,f=fopen([filePath,fileName],'r');
rowTotal = 0;
while ~feof(f)
    rowTotal = rowTotal + sum(fread(f,10000,'char')==10);
end
fclose(f);toc
fprintf('.pgm������%d\n',rowTotal);

%% �ҳ��ٶ����ֵ����Сֵ
vmax=0;vmin=0;
if isempty(Velocity_max),vmax=1;Velocity_max=-inf;end
if isempty(Velocity_min),vmin=1;Velocity_min = inf;end
if vmax||vmin
    tic,f=fopen([filePath,fileName],'r');
    while ~feof(f)
        currentLine = fgetl(f);
        s = textscan(currentLine,'%s ');
        switch s{1}{1}
            case 'PSOCONTROL'
                switch s{1}{3} 
                    case 'ON',t=1;
                    case 'OFF',t=0;
                    case 'RESET',t=-1;
                end
            case {'LINEAR','CW','CCW'}
                if t==1
                    Velocity_temp = textscan(currentLine,'%*[^F] F%f');
                    Velocity_temp = Velocity_temp{:};
                    if vmax,Velocity_max = max(Velocity_temp,Velocity_max);end
                    if vmin,Velocity_min = min(Velocity_temp,Velocity_min);end
                end
            case "'plotSwitch"
                switch s{1}{2}
                    case '1',t=1;
                    case '0',t=0;
                end
        end
    end,toc
    fprintf('����ٶȣ�%f\n��С�ٶȣ�%f\n',Velocity_max,Velocity_min);
end

%% main
f=fopen([filePath,fileName],'r');
Category = {'LINEAR';'CW&CCW';'DWELL'};Time = zeros(length(Category),1);
time = table(Category,Time);time.Properties.VariableUnits = {'' 's'};
Category = {'LINEAR';'DWELL';'PSOCONTROL';'CW&CCW';'INCREMENTAL&ABSOLUTE';'Other'};
Count = zeros(length(Category),1);count = table(Category,Count);
rowNow = 0; % ��ǰ����
f1=figure(1);hold on;
if ~exist("PointNow","var")
    clf;hold on;
    xlabel('X','Color','r');ylabel('Y','Color','r');zlabel('Z','Color','r');
    view([-1,-5,-3]);
    global PointBefor PointNow PointG92 isABSOLUTE costY...	% λ��״̬
        VelocityBefor VelocityNow Vindex notdwell ...    % �˶�״̬
        plotSwitch pgmF lineColorF lineWidth lineColor  %#ok    % ��ͼ����
    PointBefor=[0,0,0];PointNow=[0,0,0];PointG92=[0,0,0];costY=0;
    VelocityBefor=[0,0,0];VelocityNow=[0,0,0];Vindex = 1;
    plotSwitch=1;pgmF=nan;lineWidth=0.1;lineColor=[1 1 1];
end
if Velocity_min<1,Vindex = 1/Velocity_min;end
Velocity_min=Vindex*Velocity_min-1;Velocity_max=Vindex*Velocity_max;
lineColorF = colormap(cool(fix(Velocity_max - Velocity_min)));
notdwell=[0,0,0];
waitBar=waitbar(0,'1','name','SIMULATING...');
while ~feof(f)
    rowNow = rowNow + 1;
    if mod(rowNow,rowPeriod) == 0
        waitbar(rowNow/rowTotal,waitBar,sprintf('%.3g%%��rowNow=%d\nrowTotal=%d',...
            rowNow/rowTotal*100,rowNow,rowTotal));
    end
    currentLine = fgetl(f);
    if isempty(currentLine),continue;end % ����
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
        case 'G92',G92(s{1}{2:end});count.Count(6)=count.Count(6)+1;
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
% colorbar
numV = Velocity_max - Velocity_min;
Velocity_min = Velocity_min + 1;
if numV > 10,c = colorbar('Ticks',0:0.1:1,'TickLabels',{linspace(Velocity_min,Velocity_max,11)/Vindex});
else,c = colorbar('Ticks',linspace(0,1,numV),'TickLabels',{linspace(Velocity_min,Velocity_max,numV)/Vindex});
end,c.Label.String = '����բ���˶��ٶ�';
% msgbox
msg = {'THE SIMULATION HAS BEEN COMPLETED';['Ƭ��(Y)�����ģ�',num2str(costY),'mm']};
time_all = sum(time.Time);
hour = floor(time_all / 3600);minute = floor((time_all - hour*3600) / 60);second = floor(time_all - hour*3600 - minute*60);
msgTime = 'Ԥ�Ƽӹ�ʱ�䣺';
if hour,msgTime = [msgTime,num2str(hour),'h'];end
if minute,msgTime = [msgTime,num2str(minute),'m'];end
if second,msgTime = [msgTime,num2str(second),'s'];end
msg(3,1) = {msgTime};
% plot summary pie
f2 = figure(2);f2.Position = [f1.Position(1)+f1.Position(3),f1.Position(2:4)];
subplot(1,2,1);pie(count.Count,count.Category);title('Count','Color','red');
subplot(1,2,2);pie(time.Time,time.Category);title('Time','Color','red');
disp(time),disp(count),disp(msg)
fabricate_debugger_costY = msg{2};fabricate_debugger_costTime = msg{3};
if ~continue_fabricata
    clearvars -except fabricate_debugger_costY fabricate_debugger_costTime time count;
    load('pgmVariables.mat');delete('pgmVariables.mat');
end

%% function
function SWITCH(s)
global PointNow PointG92 isABSOLUTE pgmF pgmI pgmJ pgmR Vindex  %#ok
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
    case 'F',pgmF=temp*Vindex;
    case 'I',pgmI=temp;
    case 'J',pgmJ=temp;
    case 'R',pgmR=temp;
    otherwise,error('SWITCH:δ��ʶ��%f',s(1));
end
end
function G92(varargin)
global PointG92 PointNow    %#ok
for temp = varargin
    switch temp{1}(1)
        case 'X',PointG92(1)=PointNow(1) - str2double(temp{1}(2:end));
        case 'Y',PointG92(2)=PointNow(2) - str2double(temp{1}(2:end));
        case 'Z',PointG92(3)=PointNow(3) - str2double(temp{1}(2:end));
        otherwise,error([mfilename,'���������Ƿ�'],'G92:�Ƿ�������<%f>',s(1));
    end
end
end

function time = LINEAR(s)
global PointBefor PointNow costY...	% λ��״̬
    VelocityBefor VelocityNow Velocity_min Velocity_max Vindex notdwell ...	% �˶�״̬
    plotSwitch pgmF lineColorF lineWidth lineColor dtime    %#ok    % ��ͼ����
PointBefor = PointNow;VelocityBefor = VelocityNow;  % ����ǰһ״̬
for temp=s,SWITCH(temp{:});end
if PointNow(2)<costY;costY=PointNow(2);end
VelocityNow=PointNow-PointBefor;
if ~any(VelocityNow);time=0;return;end  % ��������ֱ�ӷ���
L=norm(VelocityNow);    % �ƶ�����
VelocityNow=VelocityNow/norm(VelocityNow);
% �ж�ǰһ״̬������״̬�Ƿ���������߼���ϵ
notdwell = is_dwell(VelocityBefor,VelocityNow,notdwell);
% ��ͼ
if plotSwitch
    if dtime,pause(dtime);end
    temp = plot3([PointBefor(1),PointNow(1)],[PointBefor(2),PointNow(2)],...
        [PointBefor(3),PointNow(3)],'.-','LineWidth',lineWidth);
    if ~any(lineColor)&&pgmF>Velocity_min&&pgmF<=Velocity_max
        temp.Color = lineColorF(ceil(pgmF - Velocity_min),:);
    else,temp.Color = lineColor;
    end
end
time = L/pgmF*Vindex;
end
function time = CW(isCCW,s)
global PointBefor PointNow pgmI pgmJ pgmR costY...	% λ��״̬
    VelocityBefor VelocityNow Velocity_min Velocity_max Vindex notdwell ...	% �˶�״̬
    plotSwitch pgmF lineColorF lineWidth lineColor dtime    %#ok    % ��ͼ����
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
    O=PointBefor-pgmR*Fr1;
end
n1=(PointBefor-O)/pgmR;n2=cross(Vertical,n1); %�������̾�������
O2PE=PointNow - O;
theta=atan2(dot(O2PE,n2),dot(O2PE,n1));
% if theta<0,theta=theta+2*pi;end
% if isCCW,theta=2*pi-theta;end
T = [n1',n2',O']; % ����ϵ�任����
VelocityNow = (T*[-sin(theta);cos(theta);0])';
% ��ͼ
if plotSwitch
    if dtime,pause(dtime);end
    if isCCW,t = linspace(theta,0,curveNum);else,t = linspace(0,theta,curveNum);end
    A = [pgmR*cos(t);pgmR*sin(t);ones(1,curveNum)];	% Բƽ������ϵ�Ĳ�������[x,y,1]
    B = T*A;    % �ֲ�����ϵ{X'OY'}ת����{XYZ}
    x = B(1,:);y = B(2,:);z = B(3,:);
    if any(y<costY);costY=min(y(y<costY));end   % �ж�Y���ĳ���
    temp = plot3(x,y,z,'-','LineWidth',lineWidth);
    temp2 = plot3([x(1) x(end)],[y(1) y(end)],[z(1) z(end)],'.'); % ��β������
    if ~any(lineColor)&&pgmF>Velocity_min&&pgmF<=Velocity_max
        temp.Color = lineColorF(fix(pgmF - Velocity_min),:);
    else,temp.Color = lineColor;
    end,temp2.Color = temp.Color;
end
time = abs(theta)*pi/pgmF*Vindex;
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
ismove(v1==0) = v1(v1==0);  % ��Ϊ��״̬���ٶ�Ϊ0����Ϊδ�˶���\
if any(notdwell&ismove)	% �˶���δdwell
    if dot(v0,v1) < 0.99 % �нǴ���8.11��
        % ���Ա�����
        global currentLine rowNow %#ok
        temp = {'x','y','z'};temp = temp(notdwell==1);temp = [temp{:}];
        error(['����',temp,'δDWELL��%s��line %d���볢��',...
            '���������ж�����0.1����ֱ�ӱ�ע��is_dwell����'],currentLine,rowNow)
    end
end
notdwell = ismove;
end