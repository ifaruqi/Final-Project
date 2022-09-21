clear; clc;

% ----- 5.Set parameter kontrol Takagi ------------------------------- %
b = takagi_function_sim;
b.n = 1;                        % jumlah kereta
b.l = 0.27 * ones(1,b.n);       % panjang kereta (27 cm)
b.vMax = 0.5;                   % kecepatan maksimum (m/s)
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
debugg_sim = {'init'};          % display kontrol


[b.v, vvCtrl] = deal(zeros(1,b.n));
xCtrl(1,1) = 0.6585;
b.x = xCtrl;
i = 1;


while i<=400
    time = 4*0.04;
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
            elseif b.v(j) >= b.vMax-b.aMax*time %   case 3 : max speed
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

load('Trial 4.mat');

h5 = figure(5);
set(h5, 'Position', [0 480 750 275]);
subplot(1,2,1),
    rectangle('Position',[3.85 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[5 -0.005 0.8 0.01],'FaceColor','y'); hold on; ...
    plot(xCtrl(1:170,1),vvCtrl(1:170,1),'b--','LineWidth',2); hold on; ...
    plot(lBuff(1:165,1),vCtrl(1:165,1),'b','LineWidth',2);
    title('Simulasi dan Kontrol Satu Kereta - Putaran 1'); ...
    axis([0 6 -0.01 0.51]); xlabel('l (m)'); ylabel('v (m/s)'); legend('Simulasi','Percobaan'); grid on
subplot(1,2,2),
    rectangle('Position',[3.85 -0.005 0.3 0.01],'FaceColor','y'); hold on; ...
    rectangle('Position',[5 -0.005 0.8 0.01],'FaceColor','y'); hold on; ...
    plot(xCtrl(171:333,1),vvCtrl(171:333,1),'b--','LineWidth',2); hold on; ...
    plot(lBuff(166:344,1),vCtrl(166:344,1),'b','LineWidth',2);
    title('Simulasi dan Kontrol Satu Kereta - Putaran 2'); ...
    axis([0 6 -0.01 0.51]); xlabel('l (m)'); ylabel('v (m/s)'); legend('Simulasi','Percobaan'); grid on