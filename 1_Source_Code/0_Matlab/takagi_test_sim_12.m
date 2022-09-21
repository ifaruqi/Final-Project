clear; clc;

% ----- 5.Set parameter kontrol Takagi ------------------------------- %
b = takagi_function_sim;
b.n = 2;                        % jumlah kereta
b.l = 0.27 * ones(1,b.n);       % panjang kereta (27 cm)
b.vMax = 0.3;                   % kecepatan maksimum (m/s)
b.aMax = 0.1;                   % perlambatan maksimum (m/s^2)
b.dSafe = 0.27;                 % jarak aman antar kereta (m)
b.k = 1;                        % konstanta perlambatan

b.tS = zeros(1,b.n);            % incremental timer waktu kereta berhenti di stasiun
b.maxS = 2;                     % lama waktu maksimum kereta berhenti di stasiun (s)
s1 = [1 ; 0.3];                 % stasiun kecil 1
s2 = [2.5 ; 0.3];               % stasiun kecil 2
s3 = [4 ; 0.3];                 % stasiun kecil 3
s4 = [5.5 ; 1];                 % stasiun besar
b.s = [s4, s3;                  % stasiun tempat kereta 1 berhenti
    s4, s2;                     % stasiun tempat kereta 2 berhenti
    s4, s1];                    % stasiun tempat kereta 3 berhenti
b.sLast = zeros(1,b.n);         % stasiun terakhir berhenti
b.strt_ctrl = zeros(1,b.n);     % kondisi apakah sedang start control
debugg_sim = {'init','init'};% display kontrol


[b.v, vvCtrl] = deal(zeros(1,b.n));
xCtrl(1,1) = 1.3010;
xCtrl(1,2) = 0.6315;
% for i=2:b.n
%     xCtrl(1,i) = (xCtrl(1,i-1) - b.l(i-1)) ...  % initial position of following train B,C is right behind
%         - tkg_distance(b,i);                    % xB = xA - lA - dmin
% end
b.x = xCtrl;
i = 1;


while i<=400
    time = 5*0.04;
    % ----- 13.Kontrol Takagi ---------------------------------------- %
    for j = 1:b.n                               % for every train (A,B,C)
        b.x(j) = xCtrl(i,j);                    % position
        b.v(j) = vvCtrl(i,j);                    % velocity
        if b.v(j) > 0                           % IF train run
            if trn_ahead(b,j)                   %   case 1 : train ahead
                b.a(j) = tkg_sync(b,j);         %       activate sync control
                b.strt_ctrl(j) = 0;             %       release start control switch
                                                debugg_sim{i+1,j} = 'sync';
            elseif sta_ahead(b,j)               %   case 2 : station ahead
                b.a(j) = b.k*b.aMax;            %       brake at max
                                                debugg_sim{i+1,j} = 'brak';
            elseif b.v(j) >= b.vMax             %   case 3 : max speed
                b.a(j) = 0;                     %       stop accelerating
                                                debugg_sim{i+1,j} = 'maxx';
            elseif b.strt_ctrl(j)               %   case 4 : start control switch hold
                b.a(j) = tkg_start(b,j);        %       activate start control
                                                debugg_sim{i+1,j} = 'strt';
            else                                %   case 5 : else
                b.a(j) = -b.k*b.aMax;           %       accelerate at max
                                                debugg_sim{i+1,j} = 'accl';
            end
            b.tS(j) = 0;                        %   reset dwell timer after run
        elseif b.v(j) < 0                       % IF speed below minimum
            b.a(j) = 0;                         %       stop accelerating
            b.v(j) = 0;                         %       stop running
                                                debugg_sim{i+1,j} = 'stop';
        else                                    % IF train stop
            if trn_dwell(b,j)                   %   case 1 : train still dwelling
                b.a(j) = 0;                     %       wait
                b.tS(j) = trn_count(b,j,time);  %       count up
                [~,b.sLast(j)] = sta_near(b,j); %       remember the last station
                                                debugg_sim{i+1,j} = 'wait';
            elseif trn_ahead(b,j)               %   case 2 : train ahead
                b.a(j) = tkg_start(b,j);        %       activate start control
                b.strt_ctrl(j) = 1;             %       hold start control switch
                                                debugg_sim{i+1,j} = 'strt';
            else                                %   case 3 : else
                b.a(j) = -b.k*b.aMax;           %       accelerate at max
                                                debugg_sim{i+1,j} = 'accl';
            end
        end
        
        vvCtrl(i+1,j) = b.v(j) - b.a(j)*time;
        xCtrl(i+1,j) = b.x(j) + b.v(j)*time - b.a(j)*time^2/2;
        
        if xCtrl(i+1,j) > 6.12
            xCtrl(i+1,j) = xCtrl(i+1,j) - 6.12;
        end
    end
    
    i = i + 1;
end

% Plot graph
% figure(1),
% plot(t,xCtrl); hold on                          % plot head of all trains
% plot(t,xCtrl - repmat(b.l,size(xCtrl,1),1));    % plot tail of all trains
% xlabel('time - s'); ylabel('distance - m');
% title('Distance Time Graph of MBSS'); grid on
% legend('Train A', 'Train B', 'Train C', 'Location', 'NorthWest');

load('Trial 12.mat');

h1 = figure(1);
set(h1, 'Position', [0 480 750 275]);
subplot(1,2,1),
    rectangle('Position',[2.35 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[3.85 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[5 -0.005 0.8 0.01],'FaceColor','y'); hold on; ...
    plot(lBuff(1:136,1),vCtrl(1:136,1),'b','LineWidth',2); hold on; ...
    plot(lBuff(1:164,2),vCtrl(1:164,2),'r','LineWidth',2);
    title('Percobaan Dua Kereta - Putaran 1'); ...
    axis([0 6 -0.01 0.31]); xlabel('l (m)'); ylabel('v (m/s)'); legend('kereta A','kereta B'); grid on
subplot(1,2,2),
    rectangle('Position',[2.35 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[3.85 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[5 -0.005 0.8 0.01],'FaceColor','y'); hold on; ...
    plot(lBuff(142:267,1),vCtrl(142:267,1),'b','LineWidth',2); hold on; ...
    plot(lBuff(168:300,2),vCtrl(168:300,2),'r','LineWidth',2); ...
    title('Percobaan Dua Kereta - Putaran 2'); ...
    axis([0 6 -0.01 0.31]); xlabel('l (m)'); ylabel('v (m/s)'); legend('kereta A','kereta B'); grid on

h2 = figure(2);
set(h2, 'Position', [0 0 750 275]);
%subplot(1,2,1),
    rectangle('Position',[2.35 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[3.85 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[5 -0.005 0.8 0.01],'FaceColor','y'); hold on; ...
    plot(xCtrl(1:142,1),vvCtrl(1:142,1),'b--','LineWidth',2); hold on; ...
    plot(xCtrl(1:159,2),vvCtrl(1:159,2),'r--','LineWidth',2); ...
    title('Simulasi Dua Kereta - Putaran 1'); ...
    axis([0 6 -0.01 0.31]); xlabel('l (m)'); ylabel('v (m/s)'); legend('kereta A','kereta B'); grid on
% subplot(1,2,2),
%     plot(xCtrl(143:298,1),vvCtrl(143:298,1),'b--','LineWidth',2); hold on; ...
%     plot(xCtrl(160:316,2),vvCtrl(160:316,2),'r--','LineWidth',2); ...
%     title('Simulasi Dua Kereta - Putaran 2'); ...
%     axis([0 6 -0.01 0.31]); xlabel('l (m)'); ylabel('v (m/s)'); legend('kereta A','kereta B'); grid on

h3 = figure(3);
set(h3, 'Position', [530 0 750 750]);
subplot(2,2,1),
    rectangle('Position',[3.85 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[5 -0.005 0.8 0.01],'FaceColor','y'); hold on; ...
    plot(xCtrl(1:142,1),vvCtrl(1:142,1),'b--','LineWidth',2); hold on; ...
    plot(lBuff(1:136,1),vCtrl(1:136,1),'b','LineWidth',2);
    title('Simulasi Vs Percobaan Dua Kereta - Kereta A Putaran 1'); ...
    axis([0 6 -0.01 0.31]); xlabel('l (m)'); ylabel('v (m/s)'); legend('Simulasi','Percobaan'); grid on
subplot(2,2,2),
    rectangle('Position',[3.85 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[5 -0.005 0.8 0.01],'FaceColor','y'); hold on; ...
    plot(xCtrl(143:298,1),vvCtrl(143:298,1),'b--','LineWidth',2); hold on; ...
    plot(lBuff(142:267,1),vCtrl(142:267,1),'b','LineWidth',2);
    title('Simulasi Vs Percobaan Dua Kereta - Kereta A Putaran 2'); ...
    axis([0 6 -0.01 0.31]); xlabel('l (m)'); ylabel('v (m/s)'); grid on
subplot(2,2,3),
    rectangle('Position',[2.35 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[5 -0.005 0.8 0.01],'FaceColor','y'); hold on; ...
    plot(xCtrl(1:159,2),vvCtrl(1:159,2),'r--','LineWidth',2); hold on; ...
    plot(lBuff(1:164,2),vCtrl(1:164,2),'r','LineWidth',2); 
    title('Simulasi Vs Percobaan Dua Kereta - Kereta B Putaran 1'); ...
    axis([0 6 -0.01 0.31]); xlabel('l (m)'); ylabel('v (m/s)'); legend('Simulasi','Percobaan'); grid on
subplot(2,2,4),
    rectangle('Position',[2.35 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[5 -0.005 0.8 0.01],'FaceColor','y'); hold on; ...
    plot(xCtrl(160:316,2),vvCtrl(160:316,2),'r--','LineWidth',2); hold on; ...
    plot(lBuff(168:300,2),vCtrl(168:300,2),'r','LineWidth',2);
    title('Simulasi Vs Percobaan Dua Kereta - Kereta B Putaran 2'); ...
    axis([0 6 -0.01 0.31]); xlabel('l (m)'); ylabel('v (m/s)'); grid on

figure(4),
plot(dtBuff(1:300,1),lBuff(1:300,1),'b','LineWidth',2); hold on;
plot(dtBuff(1:300,2),lBuff(1:300,2),'r','LineWidth',2); hold on;
% plot(dtBuff(1:300,1),xCtrl(1:300,1),'b--','LineWidth',2); hold on;
% plot(dtBuff(1:300,2),xCtrl(1:300,2),'r--','LineWidth',2);
title('Grafik Posisi terhadap Waktu Percobaan Dua Kereta'); ...
xlabel('t (s)'); ylabel('l (m)'); legend('Kereta A','Kereta B'); grid on
% Animate graph
% figure,
% transit_station = [1,2.5,4];
% main_station = 5.8;
% for j=1:500
%     clf; axis([-1 7 -2 2]);
%     xlabel('time / s'); ylabel('distance / m');
%     title('Train A (blue) - Train B (green)');
%     rectangle('Position',[xCtrl(j,1)-b.l(1), -.1, b.l(1), .27],'FaceColor','b');
%     rectangle('Position',[xCtrl(j,2)-b.l(2), -.1, b.l(2), .27],'FaceColor','g');
%     rectangle('Position',[xCtrl(j,3)-b.l(3), -.1, b.l(3), .27],'FaceColor','r');
%     hold on; plot(transit_station,0,'kx',main_station,0,'ko');
%     pause(0.03);
% end