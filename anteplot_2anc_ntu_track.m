close all;
clear all;

mobAntOff =[0.172, 0.283, 0.004;
            0.182, -0.280, -0.006;
            -0.376, -0.293, 0.003;
            -0.386, 0.262, 0.000]';
   
% ancPos = [0, 0, 1.5]';
% ancAntOff = [0.547, -0.065, 0.013;
%             -0.558, -0.086, 0.022]';

% niv20170731 backwards
% ancPos = [0.043, -0.581, 1.719;
%           0.075, 0.416, 1.742]';

% niv20170801 forward
% ancPos = [0.57, 0.04, 1.753;
%         -0.424, 0.035, 1.778]';

% niv20170802 forward
% ancPos = [3.307, 2.414, 1.708;
%          2.553, 3.051, 1.737]';
  
% % niv20170802 forward
% ancPos = [3.474, -0.369, 1.733;
%           3.461, 0.625, 1.77]';
      
% Near anchors
ancPos = [0.04, -0.57, 1.753;
          0.035, 0.424, 1.778]';
      
% Translating the coordinate from UWB frame to vicon frame      
ancPos = [-ancPos(2, :);
          ancPos(1, :);
          ancPos(3, :)];
     
ancAntOff = [0, 0, 0;
             0, 0, 0;
             0, 0, 0;
             0, 0, 0]';

centerOffset = [0.1; 0.0; -0.07];

% Choose one of the log file in the switch below
logname = 'niv20170821_U';

cres_sim = false;
main_vel_plot = false;

switch logname
    
    case 'niv20170821_N'
        % N
        tstart = 65;
        tend = 165;

        
    case 'niv20170821_Tmod'
        % T data
        tstart = 0;
        tend = 69.98;

        
    case 'niv20170821_U'
        main_vel_plot = true;
        % U data
        tstart = 100;
        tend = 160;
        
    case 'niv20170822_Cres'
        cres_sim = true;
        tstart = 81;
        tend = 248;
        
    otherwise
        msgbox('not recognized log, exiting!');
        return;
end

mkdir('exp/ntu_track/');

flightdata = csvread(['bagcsv/' logname, '.csv'], 1, 0);

%--Trimming data
t = flightdata(:, 1)' - flightdata(1, 1);

I = find( t > tstart & t < tend);
flightdata = flightdata(I, :);
%--Trimming data

[K, ~] = size(flightdata);

t = flightdata(:, 1)';
t = t - t(1);

ttakeoff = t(1);
tlanding = t(end);

vcTarP = flightdata(:, 25:27)';
vcTarV = flightdata(:, 28:30)';
% vcTarP = [-vcTarP(2, :); vcTarP(1, :); vcTarP(3, :)];
vcTarQ = flightdata(:, 31:34)';
vcTarQ = [vcTarQ(4, :); vcTarQ(1:3, :)]';
vcTarDCM = quat2dcm(vcTarQ);

vcP = flightdata(:, 2:4)';

vcV = flightdata(:, 5:7)';
vcEul = flightdata(:, 8:10)';
vcDCM = zeros(3, 3, K);

px4P = flightdata(:, 11:13)';
px4P = [-px4P(2, :); px4P(1, :); px4P(3, :)] + vcTarP;
px4V = flightdata(:, 14:16)';
px4V = [-px4V(2, :); px4V(1, :); px4V(3, :)];

px4Eul = flightdata(:, 17:19)';
px4Lidar = flightdata(:, 20)';

corflow = flightdata(:, 21:23)';
flowpsr = flightdata(:, 24)';

%% Calculate some errors and convert the angles
rmsex = rms(vcP(1, :) - px4P(1, :));
rmsey = rms(vcP(2, :) - px4P(2, :));
rmsez = rms(vcP(3, :) - px4P(3, :));

stdx = std(vcP(1, :) - px4P(1, :));
stdy = std(vcP(2, :) - px4P(2, :));
stdz = std(vcP(3, :) - px4P(3, :));

rmsevx = rms(vcV(1, :) - px4V(1, :));
rmsevy = rms(vcV(2, :) - px4V(2, :));
rmsevz = rms(vcV(3, :) - px4V(3, :));

stdvx = std(vcV(1, :) - px4V(1, :));
stdvy = std(vcV(2, :) - px4V(2, :));
stdvz = std(vcV(3, :) - px4V(3, :));

uwbD = flightdata(:, end-3)';
ancAntId = floor(flightdata(:, end-2)'/16) + 1;
mobAntId = mod(flightdata(:, end-2)', 16) + 1;
rqstrId = flightdata(:, end)'-1;
rspdrId = flightdata(:, end-1)'+1;

[~, mobAnts] = size(mobAntOff);
[~, ancs] = size(ancPos);

edgeTotal = max(mobAnts*ancs);

f = [];

% uwbAllD = zeros(antCount, K);
vcD = zeros(1, K);
vcDCM = zeros(3, 3, K);
px4DCM = zeros(3, 3, K);

px4EulinVC = zeros(3, K);

vcMobAntPos = zeros(3, mobAnts*2, K);
vcAntPosCompact = zeros(3, K);

vcVbody = zeros(3, K);

for k = 1:K
    vcRo = vcEul(1, k);
    vcPi = vcEul(2, k);
    vcYa = vcEul(3, k);
    
    vcRx = [1, 0, 0; 0, cos(vcRo), -sin(vcRo); 0, sin(vcRo), cos(vcRo)];
    vcRy = [cos(vcPi), 0, sin(vcPi); 0, 1, 0; -sin(vcPi), 0, cos(vcPi)];
    vcRz = [cos(vcYa), -sin(vcYa), 0; sin(vcYa), cos(vcYa), 0; 0, 0, 1];
    
    vcDCM(:, :, k) = vcRx*vcRy*vcRz;
    
    px4Ro = px4Eul(1, k);
    px4Pi = px4Eul(2, k);
    px4Ya = px4Eul(3, k);
    
    px4Rx = [1, 0, 0; 0, cos(px4Ro), -sin(px4Ro); 0, sin(px4Ro), cos(px4Ro)];
    px4Ry = [cos(px4Pi), 1, sin(px4Pi); 0, 1, 0; -sin(px4Pi), 0, cos(px4Pi)];
    px4Rz = [cos(px4Ya), -sin(px4Ya), 0; sin(px4Ya), cos(px4Ya), 0; 0, 0, 1];
    
    px4DCM(:, :, k) = [0 1, 0; 1 0 0; 0 0 -1]*px4Rz*px4Ry*px4Rx*[1 0, 0; 0 -1 0; 0 0 -1];
    
    [px4Eul(1, k), px4Eul(2, k), px4Eul(3, k)] = dcm2angle(px4DCM(:, :, k)', 'XYZ');
    [vcEul(1, k), vcEul(2, k), vcEul(3, k)] = dcm2angle(vcDCM(:, :, k)', 'XYZ');
    
    for n=1:2
        for s=1:2
            vcMobAntPos(:, (n-1)*2 + s, k) = vcDCM(:, :, k)*mobAntOff(:, (n-1)*2 + s) + vcP(:, k);
        end
    end
    vcD(k) = norm(vcMobAntPos(:, (rqstrId(k)-1)*2 + mobAntId(k), k)...
                - (ancPos(:, rspdrId(k)) + ancAntOff(:, ancAntId(k))));
    vcAntPosCompact(:, k) = vcMobAntPos(:, (rqstrId(k)-1)*2 + mobAntId(k), k);
    
    vcVbody(:, k) = vcDCM(:, :, k)'*vcV(:, k);
    
    if flowpsr(k) > 40
        corflowvalid(:, k) = corflow(:, k);
    elseif k > 1
        corflowvalid(:, k) = corflowvalid(:, k-1);
    else
        corflowvalid(:, k) = [0; 0; 0];
    end
end

maxTarVx = max(abs(vcTarV(1, :)));
maxTarVy = max(abs(vcTarV(2, :)));
maxTarVz = max(abs(vcTarV(3, :)));

rmseyaw = rms(vcEul(3, :)*180/pi - px4Eul(3, :)*180/pi);
rmsepitch = rms(vcEul(2, :)*180/pi - px4Eul(2, :)*180/pi);
rmseroll = rms(vcEul(1, :)*180/pi - px4Eul(1, :)*180/pi);

stdyaw = std(vcEul(3, :)*180/pi - px4Eul(3, :)*180/pi);
stdpitch = std(vcEul(2, :)*180/pi - px4Eul(2, :)*180/pi);
stdroll = std(vcEul(1, :)*180/pi - px4Eul(1, :)*180/pi);

errsig = round([rmsex, rmsey, rmsez, maxTarVx, maxTarVy, maxTarVz;
                stdx, stdy, stdz, 0, 0, 0], 3);
%% Calculate some errors and convert the angles


%% Plot the 3D trajectories
figpos = [1920 1080-675 600 600];
anifig = figure('name', '3D Pos', 'position', figpos, 'color', [1 1 1]);
hold on;
vctarhd = plot3(vcTarP(1, :),...
                vcTarP(2, :),...
                vcTarP(3, :),...
                'color', [0, 0.75, 0], 'linewidth', 1.5);
vcquadhd = plot3(vcP(1, :),...
                 vcP(2, :),...
                 vcP(3, :),...
                 'r', 'linewidth', 1.5);
px4quadhd = plot3(px4P(1, :),...
                  px4P(2, :),...
                  px4P(3, :),...
                  'b', 'linewidth', 1.5);
trajax = gca;
set(gca, 'DataAspectRatio', [1 1 1], 'fontname', 'cambria', 'fontsize', 20);
xlim([-2.5, 2.5]);
ylim([-2.5, 2.5]);
% zlim([0, 2]);
xlabel('Pos. x [m]', 'interpreter', 'latex');
ylb = ylabel('Pos. y [m]', 'interpreter', 'latex');%, 'position', [-9.2995 -14.2466 2.6102]);
zlabel('Pos. z [m]', 'interpreter', 'latex');

view([0, 90]);

% lghd = legend('Target''s pos. (Vicon)', 'MAV''s pos. (Vicon)', 'MAV'' pos. estimate + target''s pos.');
% set(lghd, 'interpreter', 'latex', 'fontsize', 20, 'position', [0.436 0.9 0.168 0.096]);
grid on;

xticks([-2.5:0.5:2.5]);
yticks([-2.5:0.5:2.5]);

tightfig(gcf);
set(gcf, 'position', figpos);

set(gcf, 'PaperPositionMode', 'auto');
saveas(gcf, ['exp/ntu_track/' logname '_traj.png'], 'png');
%% Plot the 3D trajectories


%% Plot the position over time
figpos = [1920+600 1080-675 600 600];
figure('name', 'Position','position', figpos, 'color', [1 1 1]);
subplot(3, 1, 1);
plot(t, vcP(1, :), 'r', 'linewidth', 1.5);
hold on;
plot(t, px4P(1, :), 'b', 'linewidth', 1.5);
grid on;
ylim([-2.5, 2.5]);
set(gca, 'xlim', [t(1) t(end)]);
% xlabel('Time [s]', 'interpreter', 'latex', 'fontsize', 14);
ylabel('Pos. x [m]', 'interpreter', 'latex', 'fontsize', 14);
set(gca, 'fontname', 'cambria', 'fontsize', 20);
grid on;

subplot(3, 1, 2);
plot(t, vcP(2, :), 'r', 'linewidth', 1.5);
hold on;
plot(t, px4P(2, :), 'b', 'linewidth', 1.5);
grid on;
ylim([-2.5, 2.5]);
set(gca, 'xlim', [t(1) t(end)]);
% xlabel('Time [s]', 'interpreter', 'latex', 'fontsize', 14);
ylabel('Pos. y [m]', 'interpreter', 'latex', 'fontsize', 14);
set(gca, 'fontname', 'cambria', 'fontsize', 20);

subplot(3, 1, 3);
% plot(t, vcP(3, :), 'r', t, px4P(3, :), 'b', t, px4Lidar, 'color', [0, 0.75, 0]);
plot(t, vcP(3, :), 'r', 'linewidth', 1.5);
hold on;
plot(t, px4P(3, :), 'b', 'linewidth', 1.5);
grid on;
ylim([0, 1.5]);
set(gca, 'xlim', [t(1) t(end)]);
xlabel('Time [s]', 'interpreter', 'latex', 'fontsize', 14);
ylabel('Pos. z [m]', 'interpreter', 'latex', 'fontsize', 14);
set(gca, 'fontname', 'cambria', 'fontsize', 20);

lghd = legend('Vicon', 'Estimate');
set(lghd, 'interpreter', 'latex',...
    'fontsize', 20,...
    'position', [0.2, 0.125, 0.259, 0.114]);

tightfig(gcf);
set(gcf, 'position', figpos);

set(gcf, 'PaperPositionMode', 'auto');
saveas(gcf, ['exp/ntu_track/' logname '_pos.png'], 'png');
%% Plot the position over time


%% Plot the velocity over time
figpos = [1920 1080-1275 600 600];
figure('name', 'Velocity','position', figpos, 'color', [1 1 1]);
subplot(3, 1, 1);
plot(t, vcV(1, :), 'r', 'linewidth', 1.5);
hold on;
plot(t, px4V(1, :), 'b', 'linewidth', 1.5);
hold on;
plot(t, vcTarV(1, :), 'linewidth', 1.5, 'color', [0, 0.75, 0]);
grid on;
ylim([-0.6, 0.6]);
set(gca, 'xlim', [t(1) t(end)]);
% xlabel('Time [s]', 'interpreter', 'latex', 'fontsize', 14);
ylabel('Vel. x [m/s]', 'interpreter', 'latex', 'fontsize', 14);
set(gca, 'fontname', 'cambria', 'fontsize', 20);
grid on;

subplot(3, 1, 2);
plot(t, vcV(2, :), 'r', 'linewidth', 1.5);
hold on;
plot(t, px4V(2, :), 'b', 'linewidth', 1.5);
hold on;
plot(t, vcTarV(2, :), 'linewidth', 1.5, 'color', [0, 0.75, 0]);
grid on;
ylim([-0.6, 0.6]);
set(gca, 'xlim', [t(1) t(end)]);
% xlabel('Time [s]', 'interpreter', 'latex', 'fontsize', 14);
ylabel('Vel. y [m/s]', 'interpreter', 'latex', 'fontsize', 14);
set(gca, 'fontname', 'cambria', 'fontsize', 20);

subplot(3, 1, 3);
% plot(t, vcP(3, :), 'r', t, px4P(3, :), 'b', t, px4Lidar, 'color', [0, 0.75, 0]);
plot(t, vcV(3, :), 'r', 'linewidth', 1.5);
hold on;
plot(t, px4V(3, :), 'b', 'linewidth', 1.5);
hold on;
plot(t, vcTarV(3, :), 'linewidth', 1.5, 'color', [0, 0.75, 0]);
grid on;
ylim([-0.25, 0.25]);
set(gca, 'xlim', [t(1) t(end)]);
xlabel('Time [s]', 'interpreter', 'latex', 'fontsize', 14);
ylabel('Vel. z [m/s]', 'interpreter', 'latex', 'fontsize', 14);
set(gca, 'fontname', 'cambria', 'fontsize', 20);

lghd = legend('MAV''s velocity (Vicon)', 'MAV''s velocity (Estimate)', 'Target''s Velocity');

if ~main_vel_plot
    set(lghd, 'interpreter', 'latex', 'fontsize', 20,...
        'position', [0.385851356620053,0.655878196769177,0.604630620093228,0.168243606461645]);
else
    set(lghd, 'interpreter', 'latex', 'fontsize', 20,...
        'position', [0.385851356620053,0.302544863435844, 0.604630620093228, 0.168243606461645]);
end

tightfig(gcf);
set(gcf, 'position', figpos);

set(gcf, 'PaperPositionMode', 'auto');
saveas(gcf, ['exp/ntu_track/' logname '_vel.png'], 'png');
%% Plot the velocity over time


%% Plot the angle over time
figpos = [1920+600 1080-1275 600 600];
figure('name', 'Angle','position', figpos, 'color', [1 1 1]);
subplot(3, 1, 3);
plot(t, vcEul(1, :)*180/pi, 'r', 'linewidth', 1.5);
hold on;
plot(t, px4Eul(1, :)*180/pi, 'b', 'linewidth', 1.5);
grid on;
ylim([-10, 10]);
set(gca, 'xlim', [t(1) t(end)]);
xlabel('Time [s]', 'interpreter', 'latex', 'fontsize', 14);
ylabel('Roll [deg]', 'interpreter', 'latex', 'fontsize', 14);
set(gca, 'fontname', 'cambria', 'fontsize', 20);

subplot(3, 1, 2);
plot(t, vcEul(2, :)*180/pi, 'r', 'linewidth', 1.5);
hold on;
plot(t, px4Eul(2, :)*180/pi, 'b', 'linewidth', 1.5);
grid on;
ylim([-10, 10]);
set(gca, 'xlim', [t(1) t(end)]);
% xlabel('Time [s]', 'interpreter', 'latex', 'fontsize', 14);
ylabel('Pitch [deg]', 'interpreter', 'latex', 'fontsize', 14);
set(gca, 'fontname', 'cambria', 'fontsize', 20);

subplot(3, 1, 1);
% plot(t, vcP(3, :), 'r', t, px4P(3, :), 'b', t, px4Lidar, 'color', [0, 0.75, 0]);
plot(t, vcEul(3, :)*180/pi, 'r', 'linewidth', 1.5);
hold on;
plot(t, px4Eul(3, :)*180/pi, 'b', 'linewidth', 1.5);
grid on;
ylim([85, 100]);
set(gca, 'xlim', [t(1) t(end)]);
% xlabel('Time [s]', 'interpreter', 'latex', 'fontsize', 14);
ylabel('Yaw [deg]', 'interpreter', 'latex', 'fontsize', 14);
lghd = legend('Vicon', 'Estimate');
set(lghd, 'interpreter', 'latex', 'fontsize', 20,...
    'position', [0.728,0.87,0.259,0.114]);
set(gca, 'fontname', 'cambria', 'fontsize', 20);

tightfig(gcf);
set(gcf, 'position', figpos);

set(gcf, 'PaperPositionMode', 'auto');
saveas(gcf, ['exp/ntu_track/' logname '_ang.png'], 'png');
%% Plot the angle over time


%% Plot the distance measurement vs ground truth
% figpos = [1920+800 9 1600 1200];
% figure('name', 'distance measurement',...
%        'position', figpos, 'color', 'w');
% f = [];
% Y = [];
% for m = 1:2
%     for n = 1:2
%         for l = 1:2
%             ax_idx = (m-1)*4 + (n-1)*2 + l;
%             subplot(edgeTotal/2, 2, ax_idx);
%             hold on;
%             I = (find(rqstrId == m & mobAntId == l & rspdrId == n));
%             Y = [Y; (uwbD(I) - vcD(I))'];
%             f = [f, mean(Y)];
%             plot(t(I), vcD(I), 'r', 'linewidth', 1.5);
%             plot(t(I), uwbD(I) - f(end), 'linewidth', 1.5);
%             maxdist = round(max(vcD(I))+1);
%             ylim([1, maxdist]);
%             if(ax_idx >= edgeTotal - 1)
%                 xlabel('Time [s]');
%             end
%             ylabel('Dist. [m]');
%             text(mean(xlim), maxdist-0.25, ['$d_{' num2str((m-1)*2 + l),...
%                           ',' num2str(n), '}$'],...
%                  'interpreter', 'latex',...
%                  'fontname', 'cambria', 'fontsize', 20);
%             grid on;
%             set(gca, 'fontname', 'cambria', 'fontsize', 20);
%             if ax_idx == 1
%                 lghd = legend('Vicon', 'UWB');
%                 set(lghd, 'interpreter', 'latex', 'fontsize', 20);
%             end
%         end
%     end
% end
% 
% tightfig(gcf);
% set(gcf, 'position', figpos);
% 
% set(gcf, 'PaperPositionMode', 'auto');
% saveas(gcf, ['exp/ntu_track/' logname '_dis.png'], 'png');
%% Plot the distance measurement vs ground truth


%% Plot the position error
figpos = [1920+1200 -50 1200 600];
figure('name', 'Pos. Err', 'position', figpos, 'color', 'w');
hold on;
plot(t, abs(vcP(1, :) - px4P(1, :)), 'r', 'linewidth', 1.5);
plot(t, abs(vcP(2, :) - px4P(2, :)), 'color', [0, 0.75, 0], 'linewidth', 1.5);
plot(t, abs(vcP(3, :) - px4P(3, :)), 'b', 'linewidth', 1.5);
set(gca, 'fontname', 'cambria', 'fontsize', 20);
grid on;
% ylim([0, 0.5]);
xlabel('Time [s]');
ylabel('Abs. error [m]');

lghd = legend('x', 'y', 'z');
set(lghd, 'interpreter', 'latex', 'fontsize', 20, 'location', 'best');

tightfig(gcf);
set(gcf, 'position', figpos);

set(gcf, 'PaperPositionMode', 'auto');
saveas(gcf, ['exp/ntu_track/' logname '_pos_err.png'], 'png');
%% Plot the position error


%% Plot the velocity vs flow
figpos = [1920+1200 1080-675 1200 600];
figure('name', 'Vels and Flow', 'position', figpos, 'color', 'w');

subplot(2, 1, 1);
hold on;
plot(t, vcV(1, :), 'r', 'linewidth', 1.5);
plot(t, px4V(1, :), 'b', 'linewidth', 1.5);
plot(t, corflow(1, :).*vcP(3, :), 'color', [0, 0.75, 0], 'linewidth', 1.5);
grid on;
ylabel('Vel. [m/s]');
set(gca, 'fontname', 'cambria', 'fontsize', 20);

subplot(2, 1, 2);
hold on;
plot(t, vcV(2, :), 'r', 'linewidth', 1.5);
plot(t, px4V(2, :), 'b', 'linewidth', 1.5);
plot(t, -corflow(2, :).*vcP(3, :), 'color', [0, 0.75, 0], 'linewidth', 1.5);
grid on;
ylabel('Vel. [m/s]');
set(gca, 'fontname', 'cambria', 'fontsize', 20);
xlabel('Time [s]');

lghd = legend('Vicon', 'Estimate', 'Corr. flow');
set(lghd, 'interpreter', 'latex', 'fontsize', 20,...
    'position', [0.497, 0.368, 0.142, 0.168]);

tightfig(gcf);
set(gcf, 'position', figpos);

set(gcf, 'PaperPositionMode', 'auto');
saveas(gcf, ['exp/ntu_track/' logname '_vel_flow.png'], 'png');
%% Plot the velocity vs flow


%% Animate the 3D plot
vidfilename = ['exp/ntu_track/' logname '_ani.mp4'];
gridcellmarks = zeros(600, 600);
vo = VideoWriter(vidfilename, 'MPEG-4');
vo.FrameRate = K/(t(end)-t(1));
open(vo);

% anihd = figure('name', '3D Pos', 'position', [865 200 600 600], 'color', [1 1 1]);
% view([-22.8, 10.8]);
tarMarkerhd = plot3(trajax,...
                    vcTarP(1, 1),...
                    vcTarP(2, 1),...
                    vcTarP(3, 1),...
                    'o', 'markersize', 8, 'MarkerFaceColor', [0, 1, 0], 'MarkerEdgeColor', 'w');
set(get(get(tarMarkerhd,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

quadVcMarkerhd = plot3(trajax,...
                       vcP(1, 1),...
                       vcP(2, 1),...
                       vcP(3, 1),...
                       'o', 'markersize', 8, 'MarkerFaceColor', [1, 0, 0], 'MarkerEdgeColor', 'w');
set(get(get(quadVcMarkerhd,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

quadEstMarkerhd = plot3(trajax,...
                        px4P(1, 1),...
                        px4P(2, 1),...
                        px4P(3, 1),...
                        'o', 'markersize', 8, 'MarkerFaceColor', [0, 0, 1], 'MarkerEdgeColor', 'w');
set(get(get(quadEstMarkerhd,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

figure(anifig);

if cres_sim
    
    tarHeading(:, 1) = [0 -1  0;...
                        1  0  0;...
                        0  0  1]*vcTarDCM(:, :, k)^(-1)*[0.5; 0; 0];
    setpoint(:, 1) = [0 -1  0;...
                      1  0  0;...
                      0  0  1]*vcTarDCM(:, :, k)^(-1)*[-2; 0; 0] + vcTarP(:, k);

    quiv = arrow(vcTarP(:, 1),...
                 vcTarP(:, 1) + tarHeading(:, 1),...
                 'BaseAngle', 45,...
                 'linestyle', ':',...
                 'linewidth', 2);

    sphd = plot3(trajax,...
                 [vcTarP(1, 1), setpoint(1, 1)],...
                 [vcTarP(2, 1), setpoint(2, 1)],...
                 [vcTarP(3, 1), setpoint(3, 1)],...
                 'linestyle', ':',...
                 'linewidth', 2,...
                 'color', [0.1, 0.1, 0.1]);
	
end

for k = 1:K-1
    
    if cres_sim
        
        tarHeading(:, k) = [0 -1  0;...
                            1  0  0;...
                            0  0  1]*vcTarDCM(:, :, k)^(-1)*[0.5; 0; 0];
                    
        setpoint(:, k) = [0 -1  0;...
                          1  0  0;...
                          0  0  1]*vcTarDCM(:, :, k)^(-1)*[-2; 0; 0] + vcTarP(:, k);
                  
        arrow(quiv, 'Start', vcTarP(:, k),...
                    'Stop', vcTarP(:, k) + tarHeading(:, k));
                
        set(sphd,...
            'xdata', [vcTarP(1, k), setpoint(1, k)],...
            'ydata', [vcTarP(2, k), setpoint(2, k)],...
            'zdata', [vcTarP(3, k), setpoint(3, k)]);
    end
    
    set(vctarhd, 'xdata', vcTarP(1, 1:k), 'ydata', vcTarP(2, 1:k), 'zdata', vcTarP(3, 1:k));
    set(vcquadhd, 'xdata', vcP(1, 1:k), 'ydata', vcP(2, 1:k), 'zdata', vcP(3, 1:k));
    set(px4quadhd, 'xdata', px4P(1, 1:k), 'ydata', px4P(2, 1:k), 'zdata', px4P(3, 1:k));
    
    set(tarMarkerhd, 'xdata', vcTarP(1, k), 'ydata', vcTarP(2, k), 'zdata', vcTarP(3, k));
    set(quadVcMarkerhd, 'xdata', vcP(1, k), 'ydata', vcP(2, k), 'zdata', vcP(3, k));
    set(quadEstMarkerhd, 'xdata', px4P(1, k), 'ydata', px4P(2, k), 'zdata', px4P(3, k));
    
    F = getframe(anifig);
    writeVideo(vo, F.cdata);
end
close(vo)
%% Animate the 3D plot
