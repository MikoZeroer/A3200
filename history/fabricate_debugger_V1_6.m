%% ������־
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
%% �����ṩ�ӹ�����Ĺ�����
save('pgmVariables.mat')
clear
%% ������ˢ�¼������������ٶ�
rowPeriod = 1000;
maxVelocity = 70;
minVelocity = 38;
%% �汾���
% if(exist('debuggerVersionCheck','file'))
%     if debuggerVersionCheck('1_5')
%         error('�Ѹ��£������´�')
%     end
% end
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


%% �����ı�������
f=fopen([filePath,fileName],'r');
rowTotal = 0;
rowNow = 0;
while ~feof(f)
    rowTotal = rowTotal + sum(fread(f,10000,'char')==10);
end
fclose(f);
%% initial
f=fopen([filePath,fileName],'r');
clear filePath fileName
time = 0;   % �ӹ�ʱ��
global X Y Z x0 y0 z0 costY v dwell
X=0;Y=0;Z=0;x0=0;y0=0;z0=0;costY=0;
v=zeros(1,3);dwell=zeros(1,3);%δdwellΪ1����dwellΪ0
global F changeX changeY changeZ addMode lineColor lineWidth R I J
F=0;changeX=0;changeY=0;changeZ=0;R=0;I=0;J=0;lineColor='y';lineWidth=0.1;
global changeX_G92 changeY_G92 changeZ_G92 plotSwitch
changeX_G92=0;changeY_G92=0;changeZ_G92=0;plotSwitch=1; % G92���浱ǰλ��
global lineColorF
% if maxVelocity - minVelocity > 100
lineColorF = colormap(cool(fix(maxVelocity - minVelocity + 1)));
% else
%     lineColorF = colormap(cool(100));
% end
minVelocity = minVelocity - 1;
count=zeros(1,7);   % ������
% [LINEAR,PSOCONTROL,DWELL,�����˶�/����˶�,CW/CCW,G92,�������]
waitBar = waitbar(0,'1','name','SIMULATING...');    % ������
clf;hold on;
view(0,90);%YOX
% view(90,0);%ZOY
% view(0,0);%ZOX
xlabel('X','Color','r');ylabel('Y','Color','r');zlabel('Z','Color','r');

%% main
currentLine = fgetl(f);
while ischar(currentLine)
    %% ���½�����
    rowNow = rowNow + 1;
    if mod(rowNow,rowPeriod) == 0
        waitbar(rowNow/rowTotal,waitBar,sprintf('%.3g%%',rowNow/rowTotal*100));
    end
    %% �ҵ���ǰ����ʶ��
    s = strsplit(currentLine);
    s(cellfun(@isempty,s))=[];  %����cell
    %% ����
    if isempty(s),currentLine = fgetl(f);continue;end
    %% ע���� & �����ж�
    if currentLine(1) == "'"
        if strcmp(s{1},"'plotSwitch")   %�����ж�
            switch s{2}
                case "1",plotSwitch=1;
                case "0",plotSwitch=0;
                otherwise,error("'plotSwitch��ʶ���������%s",s{2});
            end
        end
        currentLine = fgetl(f);continue;
    end
    bsf = s{1};
    %%
    t_bsf = 0;
    if strcmp(bsf,'LINEAR')
        %{
    ��num�洢��ǵ�һ������ĸ�Լ�ÿ����������һλ��λ��
    �жϲ�����ΪX��Y��Z��F������ֵ������Ӧȫ�ֱ�����
    ��ͼ����ʱ
        %}
        %��������ΪX Y Z F��һ������в�������Ŀ����4��
        [czf,num,a] = get_aixs_dis(s,currentLine,4);
        for i = 1:a,switch_LINEAR(czf{i},num(i));end
        t=[X-x0,Y-y0,Z-z0];t=t/norm(t);% tΪ��ǰ�ٶ�����
        if isnan(t);error('����tΪNAN��');end
        [a,dwell] = is_dwell(v,t,dwell);
        if a
            temp = {'x','y','z'};temp = temp(dwell==1);temp = [temp{:}];
            error(['����',temp,'δDWELL��%s��line %d���볢��',...
                '���������ж�����0.1����ֱ�ӱ�ע��is_dwell����'],currentLine,rowNow)
        end
        v=t;
        %��ͼ����ʱ
        if fix(F) > maxVelocity;maxVelocityError(maxVelocity);end
        time = time + move_LINEAR(fix(F - minVelocity));
        if isnan(X);error('����XΪNAN��');end
        count(1)=count(1)+1;
    elseif strcmp(bsf,'PSOCONTROL')
        %{
    ȡ'PSOCONTROL X '֮��Ĳ������ַ��������г��ո��
    ����ʣ'ON'��'OFF'��'RESET'
        %}
        if length(s) ~= 3;error('elements num do not match in PSOCONTROL:%s',currentLine);end
        %ȡ'PSOCONTROL X '֮��Ĳ������ַ��������г��ո�
        czf = s{3};
        %�ж�'ON'��'OFF'��'RESET'
        n = 1;
        for a = strcmp(czf,{'ON','OFF','RESET'})
            if a;break;end
            n = n + 1;
        end
        if ~a;error('PSOCONTROL�������������ON��OFF��RESET��');end
        %����բΪ���ߣ��ع�բΪ����
        switch n
            case 1,lineColor = 'k';lineWidth = 1.5;
            case 2,lineColor = 'y';lineWidth = 0.1;
        end
        count(2)=count(2)+1;
    elseif strcmp(bsf,'DWELL')
        if length(s) ~= 2;error('elements num do not match in DWELL:%s',currentLine);end
        dwell=zeros(1,3);
        %��ʱ
        time = time + str2double(s{2});
        count(3)=count(3)+1;
        % �����˶�/����˶� addMode=1/0
    elseif strcmp(bsf,'INCREMENTAL')
        addMode = 0;
        count(4)=count(4)+1;
    elseif strcmp(bsf,'ABSOLUTE')
        addMode = 1;
        count(4)=count(4)+1;
    elseif strcmp(bsf,'CW')
        %����������5������X Y I J F��λ�ñ������5��
        [czf,num,a] = get_aixs_dis(s,currentLine,5);
        for i = 1:a;switch_CW(czf{i},num(i));end
        %��ͼ����ʱ
        if fix(F) > maxVelocity;maxVelocityError(maxVelocity);end
        time = time + move_CW(1,fix(F - minVelocity),currentLine,rowNow);
        count(5)=count(5)+1;
    elseif strcmp(bsf,'CCW')
        %����������5������X Y I J F��λ�ñ������5��
        [czf,num,a] = get_aixs_dis(s,currentLine,5);
        %�Ƚ�num(i)+2���������������ȫ�ֱ�����
        for i = 1:a;switch_CW(czf{i},num(i));end
        %��ͼ����ʱ
        if fix(F) > maxVelocity;maxVelocityError(maxVelocity);end
        time = time + move_CW(0,fix(F - minVelocity),currentLine,rowNow);
        count(5)=count(5)+1;
    elseif strcmp(bsf,'G92')
        %����������3������X Y Z��λ�ñ������3��
        [czf,num,a] = get_aixs_dis(s,currentLine,3);
        for i = 1:a;switch_G92(czf{i},num(i));end
        count(6)=count(6)+1;
    else
        for a = strcmp(bsf,{'G359','ENABLE','METRIC','SECONDS','VELOCITY','PSOOUTPUT'})
            if a;break;end
        end
        if a;count(7)=count(7)+1;else;t_bsf = 1;end
    end
    currentLine = fgetl(f);
    if t_bsf;error([currentLine,'������ʶ���ʶ����%s��line %d'],bsf,rowNow);end
end
%% close
delete(waitBar);
fclose(f);
clearvars -except costY time count maxVelocity minVelocity;
hold off;
%% colorbar
numV = maxVelocity - minVelocity;
minVelocity = minVelocity + 1;
if numV > 10
    c = colorbar('Ticks',0:0.1:1,'TickLabels',{fix(linspace(minVelocity,maxVelocity,11))});
else
    c = colorbar('Ticks',linspace(0,1,numV),'TickLabels',{fix(linspace(minVelocity,maxVelocity,numV))});
end
c.Label.String = '����բ���˶��ٶ�';
%% msgbox
msg = {'THE SIMULATION HAS BEEN COMPLETED';['Ƭ��(Y)�����ģ�',num2str(costY),'mm']};
hour = floor(time / 3600);minute = floor((time - hour*3600) / 60);second = floor(time - hour*3600 - minute*60);
msgTime = 'Ԥ�Ƽӹ�ʱ�䣺';
if hour
    msgTime = [msgTime,num2str(hour),'h'];
end
if minute
    msgTime = [msgTime,num2str(minute),'m'];
end
if second
    msgTime = [msgTime,num2str(second),'s'];
end
msg(3,1) = {msgTime};
% msg(4:12,1) = {'';'�������ʹ�������';['LINEAR��',num2str(count(1))];...
%     ['PSOCONTROL��',num2str(count(2))];['DWELL��',num2str(count(3))];...
%     ['�����˶�/����˶���',num2str(count(4))];['CW/CCW��',num2str(count(5))];...
%     ['G92��',num2str(count(6))];['������䣺',num2str(count(7))]};
msgbox(msg, 'Success');
%% load & delete 'pgmVariables.mat'
fabricate_debugger_costY = msg{2};
fabricate_debugger_costTime = msg{3};
clearvars -except fabricate_debugger_costY fabricate_debugger_costTime
load('pgmVariables.mat');delete('pgmVariables.mat');

%% function
function [czf,num,a] = get_aixs_dis(s,currentLine,amount)
%{
������䣺
s��currentLine�е�Ԫ��
currentLine����ǰ�������ַ���
amount��Ԥ����ָ����֧�ֵ�����������

������䣺
czf��ָ�������
num��������������
a����������Ŀ
����currentLine = 'LINEAR X10.160903 F40.000000 '
num=[3.8144,70];czf={'X','F'};a=2;
%}
a = length(s) - 1;
if a == 0;error([currentLine,'���޲�����']);end
if a > amount;error([currentLine,'������������']);end
num = zeros(1,a);czf = cell(1,a);s = s(2:end);
for i = 1:a;num(i) = str2double(s{i}(2:end));czf(i) = {s{i}(1)};end
end

function switch_LINEAR(s1,num)
global addMode
switch s1
    case 'X'
        global changeX X changeX_G92;changeX = num;%#ok
        if addMode;X = changeX + changeX_G92;else;X = X + changeX;end
    case 'Y'
        global changeY Y changeY_G92 costY;changeY = num;%#ok
        if addMode;Y = changeY + changeY_G92;else;Y = Y + changeY;end
        if Y < costY;costY = Y;end
    case 'Z'
        global changeZ Z changeZ_G92;changeZ = num;%#ok
        if addMode;Z = changeZ + changeZ_G92;else;Z = Z + changeZ;end
    case 'F';global F;F = num;%#ok
    otherwise;error('��LINEAR������%s',s1);
end
end
function time_LINEAR = move_LINEAR(lineColorFNow)
global F X Y Z lineColor lineWidth x0 y0 z0 plotSwitch lineColorF v dwell
if plotSwitch
    if lineColor == 'y'
        plot3([x0,X],[y0,Y],[z0,Z],[lineColor,'.-'],'linewidth',lineWidth);
    elseif lineColorFNow > 0
        plot3([x0,X],[y0,Y],[z0,Z],'.-','color',lineColorF(lineColorFNow,:),'linewidth',lineWidth);
    else
        plot3([x0,X],[y0,Y],[z0,Z],[lineColor,'.-'],'linewidth',lineWidth);
    end
end
temp = [X-x0,Y-y0,Z-z0];
time_LINEAR = norm(temp)/F;
x0=X;y0=Y;z0=Z;
end

function switch_CW(s1,num)
global addMode
switch s1
    case 'X'
        global changeX X changeX_G92;changeX = num;%#ok
        if addMode;X = changeX + changeX_G92;else;X = X + changeX;end
    case 'Y'
        global changeY Y changeY_G92;changeY = num;%#ok
        if addMode;Y = changeY + changeY_G92;else;Y = Y + changeY;end
    case 'I';global I;I = num;%#ok
    case 'J';global J;J = num;%#ok
    case 'F';global F;F = num;%#ok
    case 'Z'
        global changeZ Z changeZ_G92;changeZ = num;%#ok
        if addMode;Z = changeZ + changeZ_G92;else;Z = Z + changeZ;end
    case 'R';global R;R = num;%#ok
    otherwise;error('CW��ʶ���������%s',s1);
end
end
function time_CW = move_CW(mode,lineColorFNow,currentLine,rowNow)
global F addMode X Y Z lineColor lineWidth I J R x0 y0 z0 plotSwitch lineColorF v dwell
if Z == z0
    if R
        [theta,I,J] = R2theta_CW([x0,X],[y0,Y],R);
    else
        [theta,I,J,R] = IJ2theta_CW([x0,X],[y0,Y],I,J,addMode);
    end
    if mode
        plotArrayTheta = linspace(theta(1),theta(2),10);
        t=[sin(theta(1)),-cos(theta(1)),0];vt=[sin(theta(2)),-cos(theta(2)),0];
    else
        plotArrayTheta = linspace(theta(2),theta(1),10);
        t=[-sin(theta(2)),cos(theta(2)),0];vt=[-sin(theta(1)),cos(theta(1)),0];
    end
    plotArrayNum = length(plotArrayTheta);
    plotArray = [I*ones(1,plotArrayNum)+R*cos(plotArrayTheta);J*ones(1,plotArrayNum)+R*sin(plotArrayTheta);z0*ones(1,plotArrayNum)];
elseif Y == y0
    if R;[theta,I,J] = R2theta_CW([x0,X],[z0,Z],R);else;[theta,I,J,R] = IJ2theta_CW([x0,X],[z0,Z],I,J,addMode);end
    if mode;plotArrayTheta = linspace(theta(1),theta(2),10);t=[sin(theta(1)),-cos(theta(1)),0];vt=[sin(theta(2)),-cos(theta(2)),0];
    else;plotArrayTheta = linspace(theta(2),theta(1),10);t=[-sin(theta(2)),cos(theta(2)),0];vt=[-sin(theta(1)),cos(theta(1)),0];end
    plotArrayNum = length(plotArrayTheta);
    plotArray = [I*ones(1,plotArrayNum)+R*cos(plotArrayTheta);y0*ones(1,plotArrayNum);J*ones(1,plotArrayNum)+R*sin(plotArrayTheta)];
elseif X == x0
    if R;[theta,I,J] = R2theta_CW([y0,Y],[z0,Z],R);else;[theta,I,J,R] = IJ2theta_CW([y0,Y],[z0,Z],I,J,addMode);end
    if mode;plotArrayTheta = linspace(theta(1),theta(2),10);t=[sin(theta(1)),-cos(theta(1)),0];vt=[sin(theta(2)),-cos(theta(2)),0];
    else;plotArrayTheta = linspace(theta(2),theta(1),10);t=[-sin(theta(2)),cos(theta(2)),0];vt=[-sin(theta(1)),cos(theta(1)),0];end
    plotArrayNum = length(plotArrayTheta);
    plotArray = [x0*ones(1,plotArrayNum);I*ones(1,plotArrayNum)+R*cos(plotArrayTheta);J*ones(1,plotArrayNum)+R*sin(plotArrayTheta)];
else;error('CW�ƶ��ᳬ������');
end
% whether dwell
[a,dwell] = is_dwell(v,t,dwell);
if a
    temp = {'x','y','z'};temp = temp(dwell==1);temp = [temp{:}];
    error(['����',temp,'δDWELL��%s��line %d���볢��',...
        '���������ж�����0.1����ֱ�ӱ�ע��is_dwell����'],currentLine,rowNow)
end
v=vt;
% plot3
if plotSwitch
    if lineColor == 'y'
        plot3([x0,X],[y0,Y],[z0,Z],[lineColor,'.'],'linewidth',lineWidth);
        plot3(plotArray(1,:),plotArray(2,:),plotArray(3,:),[lineColor,'-'],'linewidth',lineWidth);
    elseif lineColorFNow > 0
        plot3([x0,X],[y0,Y],[z0,Z],'.','color',lineColorF(lineColorFNow,:),'linewidth',lineWidth);
        plot3(plotArray(1,:),plotArray(2,:),plotArray(3,:),'-','color',lineColorF(lineColorFNow,:),'linewidth',lineWidth);
    else
        plot3([x0,X],[y0,Y],[z0,Z],[lineColor,'.'],'linewidth',lineWidth);
        plot3(plotArray(1,:),plotArray(2,:),plotArray(3,:),[lineColor,'-'],'linewidth',lineWidth);
    end
end
if mode;time_CW = R*(theta(1)-theta(2))/F;else;time_CW = R*(theta(2)-theta(1))/F;end
R=0;x0=X;y0=Y;z0=Z;
end
function [theta,i,j] = R2theta_CW(x,y,r)
%{
���������
x��y���������յ����꣬rΪ�뾶
��rΪ��ʱ��Ϊ�ӻ�
rΪ��ʱ��Ϊ�Ż�
���������
�Ƕȼ�Բ������
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
if y(1) < j;theta(1) = -theta(1);end
if y(2) < j;theta(2) = -theta(2);end
end

function switch_G92(s1,num)
switch s1
    case 'X';global X changeX_G92;changeX_G92 = X - num;%#ok
    case 'Y';global Y changeY_G92;changeY_G92 = Y - num;%#ok
    case 'Z';global Z changeZ_G92;changeZ_G92 = Z - num;%#ok
    otherwise;error('G92��ʶ���������%s',s1);
end
end

function maxVelocityError(maxVelocity)
global F;opentoline(mfilename('fullpath'),53,0);
error('����ٶ����ù��ͣ���ǰF = %d��maxVelocity = %d',fix(F),fix(maxVelocity));
end

function [a,isdwell] = is_dwell(v0,v1,isdwell)
%{
�ж�3�����ȱ���ǰ�Ƿ�dwell
dwell = 1 --- δdwell

%}
a = 0;ismove = ones(1,3);
for temp = find(v1==0)  % ��Ϊ�����ٶȺ���ʼ�ٶȶ�Ϊ0����Ϊδ�˶���
    if ~v0(temp) % δ�˶���
        ismove(temp) = 0;
    end
end
if any(isdwell&ismove)	% �˶���δdwell
    if cross(v0,v1) > 0.1 % �нǴ���5.74��
        a = 1;  % ���Ա�����
    end
end
isdwell = ismove;
end