%clear all; close all; clc; format shortG;

visualize = 0;

l1; %76.947;    % Top Y
l2; %1128.03;   % Back X
l3; %980.061;   % Bottom Y
l4; %1249.17;   % G Back X
l5; %406.386;   % G Top  Y
l6; %655.165;   % G Bottom Y

% l1 = 76.947;    % Top Y
% l2 = 1128.03;   % Back X
% l3 = 980.061;   % Bottom Y
% l4 = 1249.17;   % G Back X
% l5 = 406.386;   % G Top  Y
% l6 = 655.165;   % G Bottom Y

% wall inputs
walls = [l1 l2 l3 l4 l5 l6]'
%walls = [76.947 1128.03 980.061 1249.17 406.386 655.165]';

% puck inputs
% px = 359;
% py = 594;
%py = 200;
% px = 100;
% py = 500;
px;
py;

% dx = 800;
% dy = 500;
% dr = 80;

% defender inputs blue 1, red 2, yellow 3
% defender = [844 841 523 847;
%             411 624  721 856;
%             38  38   38 38];
% defender = [0;
%             0;
%             0];
% defender = [600;
%             200;
%             dr];

defender;

% radius of imaginary circle off image for line plotting
r = 5000;

% min and max angle of theta shots
minTheta = -72;
maxTheta = 72;

if visualize
    % plot pitch
    xlim = 1300;
    ylim = 1049;
    line([0 walls(2)], [walls(1),walls(1)])      % l1
    line([walls(2), walls(2)], [walls(1),walls(5)])    % l2
    pl = line([walls(2), walls(2)], [walls(5),walls(6)]);    % l2
    pl.Color = 'red';
    pl.LineStyle = '--';
    line([walls(2), walls(2)], [walls(6),walls(3)])    % l2
    line([0 walls(2)], [walls(3),walls(3)])      % l3
    line([walls(4), walls(4)], [walls(5),walls(6)])    % l4
    line([walls(2) walls(4)], [walls(5),walls(5)])     % l5
    line([walls(2) walls(4)], [walls(6),walls(6)])     % l6
    axis([0 walls(4) walls(1) walls(3)])
    set(gca,'Ydir','reverse');
    hold on
    plot(px,py,'r*')
end

%% calculate all initial shots
theta = minTheta:0.9:maxTheta;

% x,y on imaginary circle
init_x = r*cos(theta*pi/180)+px;
init_y = r*sin(theta*pi/180)+py;

% calculate line coefficients from puck to imaginary circle
for i = 1:length(theta)
    coeffp_c(i,:) = polyfit([px, init_x(i)], [py, init_y(i)], 1);
end
init_m = coeffp_c(:,1);
init_c = coeffp_c(:,2);

% calculate line coefficients from puck to defenders
for i = 1:size(defender,2)
    coeffp_d(i,:) = polyfit([px, defender(1,i)], [py, defender(2,i)], 1);
end
d_m = coeffp_d(:,1);
d_c = coeffp_d(:,2);

if visualize
    % plot pucks
    for i = 1:size(defender,2)
        th = 0:pi/50:2*pi;
        xunit = defender(3,i) * cos(th) + defender(1,i);
        yunit = defender(3,i) * sin(th) + defender(2,i);
        h = plot(xunit, yunit);
        hold on
    end
end

% calculate intersecting triangles between puck and defenders
xpd = defender(1,:)-px;
ypd = defender(2,:)-py;
pd = sqrt((xpd.^2) + (ypd.^2));
thd = (asin(defender(3,:)./pd))*180/pi;
thpd = (acos(xpd./pd))*180/pi;
for i = 1:size(defender,2)
    if ypd(i) > 0
        thpd(i) = -thpd(i);
    end
end

% makes theta NaN if intersecting defenders
for j = 1:size(defender,2)
    for i = 1:length(theta)
        if ~(theta(i) >= -thpd(j)+thd(j) || theta(i) <= -thpd(j)-thd(j))
            theta(i) = NaN;
        end
    end
end

% calculate interceptions of l2
% bellow goal
xpgb = atan((walls(6)-py)/(walls(2)-px))*180/pi;
xpl3 = atan((walls(3)-py)/(walls(2)-px))*180/pi;

for i = 1:length(theta)
    if ~(theta(i) <= xpgb || theta(i) >= xpl3)
        theta(i) = NaN;
    end
end

% above goal
xpgt = atan((py-walls(5))/(walls(2)-px))*180/pi;
xpl1 = atan((py-walls(1))/(walls(2)-px))*180/pi;

for i = 1:length(theta)
    if ~(theta(i) <= -xpl1 || theta(i) >= -xpgt)
        theta(i) = NaN;
    end
end

% calculate intersection points between inital shots and walls
l1_inter = (walls(1) - init_c)./init_m;
l2_inter = (init_m*walls(2))+init_c;
l3_inter = (walls(3) - init_c)./init_m;

% eliminate shots behind puck and not in required areas
l1_inter(l1_inter>=walls(2)) = NaN;
l1_inter(l1_inter<=px) = NaN;
l2_inter(l2_inter<=walls(5)) = NaN;
l2_inter(l2_inter>=walls(6)) = NaN;
l3_inter(l3_inter>=walls(2)) = NaN;
l3_inter(l3_inter<=px) = NaN;

for i = 1:length(l3_inter)
    %plot([l3_inter(i) walls(2)], [walls(3) yil3l2(i)],'g')
    %hold on;
end

% top shot puck collisions
xil1d = defender(1,:)-l1_inter;
yil2d = defender(2,:)-walls(1);
il1d = sqrt((xil1d.^2)+(yil2d.^2));
rhod = (asin((defender(3,:))./il1d))*180/pi;
rhoid = (acos(xil1d./il1d))*180/pi;

for j = 1:size(defender,2)
    for i = 1:size(l1_inter)
        if  (-theta(i) <= rhod(i,j)+rhoid(i,j) && -theta(i) >= rhoid(i,j)-rhod(i,j))
            theta(i) = NaN;
            l1_inter(i) = NaN;
            yil1l2(i) = NaN;
        end
    end
end

xid = defender(1,:)-l3_inter;
yid = walls(3)-defender(2,:);
id = sqrt((xid.^2)+(yid.^2));
rhod = (asin((defender(3,:))./id))*180/pi;
rhoid = (acos(xid./id))*180/pi;

for j = 1:size(defender,2)
    for i = 1:size(l3_inter)
        if  (theta(i) <= rhod(i,j)+rhoid(i,j) && theta(i) >= rhoid(i,j)-rhod(i,j))
            theta(i) = NaN;
            l3_inter(i) = NaN;
            yil3l2(i) = NaN;
        end
    end
end


% top bounce
for i = 1:length(l1_inter)
    xil1l2(i) = walls(2)-l1_inter(i);
    yil1l2(i) = -xil1l2(i)*tan(theta(i)*pi/180)+walls(1);
    if ~(yil1l2(i) <= walls(5) || yil1l2(i) >= walls(6))
        if visualize
            plot([l1_inter(i) walls(2)], [walls(1) yil1l2(i)],'g')
            hold on
        end
    else
        l1_inter(i) = NaN;
        theta(i) = NaN;
        yil1l2(i) = NaN;
    end
    
end

% bottom bounce
for i = 1:length(l3_inter)
    xil3l2(i) = walls(2)-l3_inter(i);
    yil3l2(i) = walls(3)-(xil3l2(i)*tan(theta(i)*pi/180));
    if ~(yil3l2(i) <= walls(5) || yil3l2(i) >= walls(6))
        if visualize
            plot([l3_inter(i) walls(2)], [walls(3) yil3l2(i)],'g')
            hold on
        end
    else
        l3_inter(i) = NaN;
        theta(i) = NaN;
        yil3l2(i) = NaN;
    end
end

for i = 1:length(l1_inter)
    if ~isnan(theta(i))
        if visualize
            plot([px init_x(i)], [py init_y(i)],'g')
        end
    else
        l1_inter(i) = NaN;
        l2_inter(i) = NaN;
        l3_inter(i) = NaN;
    end
end

if visualize
    % plot wall interceptions
    plot(l1_inter,walls(1),'ro') %l1 intercepts
    hold on
    plot(walls(2),l2_inter,'r*') %goal straight intercepts
    hold on
    plot(l3_inter,walls(3),'ro') %l3 intercepts
    hold on
    plot(walls(2),yil1l2,'bo') %l1 goal intercepts
    hold on
    plot(walls(2),yil3l2,'bo') %l2 goal intercepts
    hold on
end

% theta, X intercept points wall rebounds, Y intercepts points wall
% rebounds.
outputB = NaN(length(theta),3);
outputS = NaN(length(theta),3);
outputB(:,1) = theta;
outputS(:,1) = theta;

% l1 intercept points
for i = 1:length(l1_inter)
    if ~isnan(l1_inter(i))
        outputB(i,2) = l1_inter(i);
    end
end

% l1 goal intercept points
for i = 1:length(yil1l2)
    if ~isnan(yil1l2(i))
        outputB(i,3) = yil1l2(i);
    end
end

% l3 intercept points
for i = 1:length(l3_inter)
    if ~isnan(l3_inter(i))
        outputB(i,2) = l3_inter(i);
    end
end

% l3 goal intercept points
for i = 1:length(yil3l2)
    if ~isnan(yil3l2(i))
        outputB(i,3) = yil3l2(i);
    end
end

% l2 intercept points
for i = 1:length(l2_inter)
    if ~isnan(l2_inter(i))
        outputS(i,2) = walls(2);
        outputS(i,3) = l2_inter(i);
    end
end

outputB(any(isnan(outputB),2),:) = [];
outputS(any(isnan(outputS),2),:) = [];
outputB;
outputS;