close all; clear; delete (instrfindall); clc; fclose('all');


% ----- 1.Komunikasi MQTT nodemcu matlab ------------------------------ %
myMQTT = mqtt('tcp://192.168.43.248','ClientID','0001','Port',1882);
topic1 = 'SinyalKontrol1'; mySub1 = subscribe(myMQTT,'datasensor1', 'QoS', 0);
topic2 = 'SinyalKontrol2'; mySub2 = subscribe(myMQTT,'datasensor2', 'QoS', 0);


% ----- 2.Baca data lookup table -------------------------------------- %
n       = csvread('mapDatabase.csv',0,0,[0 0 11 0]);  % nomer track
x0      = csvread('mapDatabase.csv',0,1,[0 1 11 1]);  % posisi segmen track (x)
y0      = csvread('mapDatabase.csv',0,2,[0 2 11 2]);  % posisi segmen track (y)
l0      = csvread('mapDatabase.csv',0,3,[0 3 11 3]);  % posisi segmen track dalam 1D
plusmin = csvread('mapDatabase.csv',0,4,[0 4 11 4]);  % plus minus pada map matching
xCenter = csvread('mapDatabase.csv',0,5,[0 5 11 5]);  % pusat lingkaran track belok (x)
yCenter = csvread('mapDatabase.csv',0,6,[0 6 11 6]);  % pusat lingkaran track belok (y)
x0(13) = x0(1); y0(13) = y0(1);


% ----- 3.Set matrix dan parameter kalman ----------------------------- %
f = @(x, v, th)[ ...
    (x(1)+v(1)) ;
    (x(2)+v(2)) ];                  % (posisi) matriks state transition
h = @(x, v, th)[x(1)+.27*cosd(th);x(2)+.27*sind(th)];% (posisi) matriks output
P = eye(2);                         % (posisi) kovariansi
pKalman = {P,P};
sigma_q = 100;                      % (posisi) tingkat kepercayaan prediksi
sigma_r = 3;                        % (posisi) tingkat kepercayaan pengukuran
R = sigma_r^2;                      % (posisi) noise pengukuran


% ----- 4.Set parameter kontrol Takagi ------------------------------- %
b = takagi_function;
b.n = 1;                            % jumlah kereta
b.l = 0.27 * ones(1,b.n);           % panjang kereta (27 cm)
b.vMax = 0.6;                       % kecepatan maksimum (m/s)
b.aMax = 0.1;                       % perlambatan maksimum (m/s^2)
b.dSafe = 0.27;                     % jarak aman antar kereta (m)
b.k = 1;                            % konstanta percepatan/perlambatan
b.kb = 2.5;                         % konstanta pengereman
b.bg = 0.5;                         % kompensasi pengereman (m)

b.tS = zeros(1,b.n);                % incremental timer waktu kereta berhenti di stasiun
b.maxS = 1;                         % lama waktu maksimum kereta berhenti di stasiun (s)
s1 = [1 ; 0.3];                     % stasiun kecil 1
s2 = [2.5 ; 0.3];                   % stasiun kecil 2
s3 = [4 ; 0.3];                     % stasiun kecil 3
s4 = [5.5 ; 1];                     % stasiun besar
b.s = [s4, s3;                      % stasiun tempat kereta 1 berhenti
    s4, s2;                         % stasiun tempat kereta 2 berhenti
    s4, s1];                        % stasiun tempat kereta 3 berhenti
b.sLast = zeros(1,b.n);             % stasiun terakhir berhenti
b.strt_ctrl = zeros(1,b.n);         % kondisi apakah sedang start control
debugg = {'init','init'};           % display kontrol


% ----- 5.Inisiasi data awal ------------------------------------------ %
[tBuff, xBuff, yBuff, rBuff, lBuff, vBuff, thBuff, xmBuff, ymBuff, dtBuff, xBool, yBool, ...
    vCtrl, eSpeed, pwmMotor, pwmTreshold, arduBuff] = deal(zeros(1,b.n));
[xKalman, xPred] = deal(zeros(1,2*b.n));
for j=1:b.n
    pause(0.04); dat = read(eval(['mySub' num2str(j)])); datkon = str2num(dat);
    [xKalman(1,2*j-1:2*j), xPred(1,2*j-1:2*j)] = deal([datkon(1)/1000, datkon(2)/1000]);
end
[pwmMotor(1,1), pwmTreshold(1,1)] = deal(750);      % set pwm treshold
[pwmMotor(1,2), pwmTreshold(1,2)] = deal(850);      % set pwm treshold


% ----- 6.Membuat lintasan ideal ------------------------------------- %
ll = 2; cc = 0.49; rr = 0.165;      % marvelmind distance; curve center; track from point zero
Rad = cc-rr; margin = 0.05;         % curved track radius; marvelmind error margin
figure, rectangle('Position',[rr  rr (ll-2*rr) (ll-2*rr)],'Curvature',(Rad));
axis ([0 ll 0 ll]); hold on


% ----- 7.Inisiasi -------------------------------------------------- %
i = 1; tic;                     % starts timing
for j = 1:b.n                   % for every train
        
    % ----- 7.Transfer data dari nodemcu ------------------------- %
    pause(0.04);
    dat = read(eval(['mySub' num2str(j)])); % membaca data
    datkon = str2num(dat);   % konversi tipe data string menjadi numerik
    xmarvel = datkon(1)/1000;   % data posisi x
    ymarvel = datkon(2)/1000;   % data posisi y dari sensor Marvelmind
    rfid = datkon(3);           % data nomer RFID
    speed = datkon(4);          % data kecepatan rotary encoder
    ardu_PWM = datkon(6);       % data pwm yang diterima nodemcu
    time = 0.04;                % data waktu sampling
    
    
    % ----- 8.Menentukan track dimana kereta berada -------------- %
    [ xBool(1), xBool(13)] = deal(xmarvel - x0(1));
    [ yBool(1), yBool(13)] = deal(ymarvel - y0(1));
    for k = 2:size(n)+1
        xBool(k) = xmarvel - x0(k);
        yBool(k) = ymarvel - y0(k);
        if ((xBool(k) * xBool(k-1)) < margin/10 ) && ((yBool(k) * yBool(k-1)) < margin/10 )
            track = n(k-1);
            rfid = track;
        end
    end
    
    
    % ----- 9.Algoritma virtual imu ------------------------------ %
    switch track
        case {1, 2, 4, 5}
            theta = atand( (yCenter(track)-ymarvel) / (xCenter(track)-xmarvel) ) - 90;
        case {7, 8, 10, 11}
            theta = 90 + atand( (yCenter(track)-ymarvel) / (xCenter(track)-xmarvel) );
        case {3, 6, 9, 12}
            theta = (track*30)+180;
    end
    
    
    % ----- 10.Kalman filter posisi ----------------------------- %
    Q = sigma_q^2 * time * eye(2);          % initial noise state / prediksi
    x = xKalman(i,2*j-1:2*j)';
    v = [speed*cosd(theta)*time; speed*sind(theta)*time];
    z = [xmarvel;ymarvel];
    P = cell2mat(pKalman(j));
    xPred(i+1,2*j-1:2*j) = [xPred(i,2*j-1)+v(1),xPred(i,2*j)+v(2)];
    [x, P, ~] = ukf(f,x,v,theta,P,h,z,Q,R);
    xKalman(i+1,2*j-1:2*j) = x';
    pKalman(j) = mat2cell(P,size(P,1),size(P,1));
    
    
    % ----- 11.Map matching posisi kereta ------------------------ %
    switch track
        case {1, 5, 7, 11}
            xnew = xKalman(i+1,2*j-1);
            ynew = yCenter(track) + plusmin(track) * sqrt(abs(Rad^2 - (xnew - xCenter(track))^2));
        case {2, 4, 8, 10}
            ynew = xKalman(i+1,2*j);
            xnew = xCenter(track) + plusmin(track) * sqrt(abs(Rad^2 - (ynew - yCenter(track))^2));
        case {3, 9}
            ynew = xKalman(i+1,2*j);
            xnew = xCenter(track);
        case {6, 12}
            xnew = xKalman(i+1,2*j-1);
            ynew = yCenter(track);
        otherwise
            xnew = xKalman(i+1,2*j-1);
            ynew = xKalman(i+1,2*j);
    end
    
    
    % ----- 12.Konversi 2D ke 1D tracking ------------------------ %
    lnew = l0(track) + sqrt( (xnew-x0(track))^2 + (ynew-y0(track))^2 );
    
    
    % ----- 13.Kontrol Takagi ------------------------------------ %
    b.x(j) = lnew;                              % position
    b.v(j) = vCtrl(i,j);                        % velocity
    
    % ------ 15.Simpan hasil di variabel buffer ------------------ %
        tBuff(i,j) = track;
        rBuff(i,j) = rfid;
        xBuff(i,j) = xnew;
        yBuff(i,j) = ynew;
        lBuff(i,j) = lnew;
        vBuff(i,j) = speed;
        thBuff(i,j) = theta;
        xmBuff(i,j) = xmarvel;
        ymBuff(i,j) = ymarvel;
        dtBuff(i,j) = dtBuff(i,j) + time;
        arduBuff(i,j) = ardu_PWM;
end


for i = 1:300                       % for every seconds
    for j = 1:b.n                   % for every train
        
        % ----- 7.Transfer data dari nodemcu ------------------------- %
        pause(0.04);
        dat = read(eval(['mySub' num2str(j)])); % membaca data
        datkon = str2num(dat);   % konversi tipe data string menjadi numerik
        xmarvel = datkon(1)/1000;   % data posisi x
        ymarvel = datkon(2)/1000;   % data posisi y dari sensor Marvelmind
        rfid = datkon(3);           % data nomer RFID
        speed = datkon(4);          % data kecepatan rotary encoder
        ardu_PWM = datkon(6);       % data pwm yang diterima nodemcu
        time = 0.04;                % data waktu sampling


        % ----- 8.Menentukan track dimana kereta berada -------------- %
        if rfid == 0, track = rBuff(i,j);
        else, track = rfid;
        end


        % ----- 9.Algoritma virtual imu ------------------------------ %
        switch track
            case {1, 2, 4, 5}
                theta = atand( (yCenter(track)-ymarvel) / (xCenter(track)-xmarvel) ) - 90;
            case {7, 8, 10, 11}
                theta = 90 + atand( (yCenter(track)-ymarvel) / (xCenter(track)-xmarvel) );
            case {3, 6, 9, 12}
                theta = (track*30)+180;
        end

        
        % ----- 10.Kalman filter posisi ----------------------------- %
        Q = sigma_q^2 * time * eye(2);          % initial noise state / prediksi
        x = xKalman(i,2*j-1:2*j)';
        v = [speed*cosd(theta)*time; speed*sind(theta)*time];
        z = [xmarvel;ymarvel];
        P = cell2mat(pKalman(j));
        xPred(i+1,2*j-1:2*j) = [xPred(i,2*j-1)+v(1),xPred(i,2*j)+v(2)];
        if xmarvel < 0 || xmarvel > 2 || ymarvel < 0 || ymarvel > 2 ...
                || ( xmarvel == xBuff(i,j) && ymarvel == yBuff(i,j) )
            x = [xKalman(i,2*j-1)+v(1),xKalman(i,2*j)+v(2)]';
        else
            [x, P, ~] = ukf(f,x,v,theta,P,h,z,Q,R);
        end
        xKalman(i+1,2*j-1:2*j) = x';
        pKalman(j) = mat2cell(P,size(P,1),size(P,1));


        % ----- 11.Map matching posisi kereta ------------------------ %
        switch track
            case {1, 5, 7, 11}
                xnew = xKalman(i+1,2*j-1);
                ynew = yCenter(track) + plusmin(track) * sqrt(abs(Rad^2 - (xnew - xCenter(track))^2));
            case {2, 4, 8, 10}
                ynew = xKalman(i+1,2*j);
                xnew = xCenter(track) + plusmin(track) * sqrt(abs(Rad^2 - (ynew - yCenter(track))^2));
            case {3, 9}
                ynew = xKalman(i+1,2*j);
                xnew = xCenter(track);
            case {6, 12}
                xnew = xKalman(i+1,2*j-1);
                ynew = yCenter(track);
            otherwise
                xnew = xKalman(i+1,2*j-1);
                ynew = xKalman(i+1,2*j);
        end


        % ----- 12.Konversi 2D ke 1D tracking ------------------------ %
        lnew = l0(track) + sqrt( (xnew-x0(track))^2 + (ynew-y0(track))^2 );


        % ----- 13.Kontrol Takagi ------------------------------------ %
        b.t(j) = time;
        b.x(j) = lnew;                              % position
        b.v(j) = vCtrl(i,j);                        % velocity
        if b.v(j) > 0                               % IF train run
            if trn_ahead(b,j)                       %   case 1 : train ahead
                b.a(j) = tkg_sync(b,j,debugg(i,:)); %   activate sync control
                b.strt_ctrl(j) = 0;                 %       release start control switch
                                                    debugg{i+1,j} = 'sync';
            elseif sta_ahead(b,j)                   %   case 2 : station ahead
                b.a(j) = b.kb*b.k*b.aMax;            %       brake at max
                                                    debugg{i+1,j} = 'brak';
            elseif b.v(j) >= b.vMax                 %   case 3 : max speed
                b.a(j) = 0;                         %       stop accelerating
                                                    debugg{i+1,j} = 'maxx';
            elseif b.strt_ctrl(j)                   %   case 4 : start control switch hold
                b.a(j) = tkg_start(b,j);            %       activate start control
                                                    debugg{i+1,j} = 'strt';
            else                                    %   case 5 : else
                b.a(j) = -b.k*b.aMax;               %       accelerate at max
                                                    debugg{i+1,j} = 'accl';
            end
            b.tS(j) = 0;                            %   after : reset dwell timer after run
        elseif b.v(j) < 0                           % IF speed below minimum
            b.a(j) = 0;                             %       stop accelerating
            b.v(j) = 0;                             %       stop running
                                                    debugg{i+1,j} = 'stop';
        else                                        % IF train stop
            if trn_dwell(b,j)                       %   case 1 : train still dwelling
                b.a(j) = 0;                         %       wait
                b.tS(j) = trn_count(b,j,time);      %       count up
                [~,b.sLast(j)] = sta_near(b,j);     %       remember the last station
                                                    debugg{i+1,j} = 'wait';
            elseif trn_ahead(b,j)                   %   case 2 : train ahead
                b.a(j) = tkg_start(b,j);            %       activate start control
                b.strt_ctrl(j) = 1;                 %       hold start control switch
                                                    debugg{i+1,j} = 'strt';
            else                                    %   case 3 : else
                b.a(j) = -b.k*b.aMax;               %       accelerate at max
                                                    debugg{i+1,j} = 'accl';
            end
        end
        vCtrl(i+1,j) = b.v(j) - b.a(j)*b.t(j);
        
        
        % ------ 14.Kontrol pwm motor -------------------------------- %
        Kpwm = 250;
        if pwmMotor(i,j) < pwmTreshold(j), pwmMotor(i,j) = pwmTreshold(j); end          % ...(a)
        eSpeed(i+1,j) = vCtrl(i+1,j) - vCtrl(i,j);                                      % ...(b)
        pwmMotor(i+1,j) = pwmMotor(i,j) + (Kpwm * ( eSpeed(i+1,j) ) );                  % ...(c)
        if pwmMotor(i+1,j)<pwmTreshold(j)+7 && pwmMotor(i+1,j)==pwmMotor(i,j), pwmMotor(i+1,j)=0; end
        if pwmMotor(i+1,j) > 1000, pwmMotor(i+1,j) = 1000; end                          % ...(e)
        publish(myMQTT, eval(['topic' num2str(j)]), num2str(pwmMotor(i+1,j)));          % ...(f)
        % a.jika pwm sebelumnya dibawah treshold, maka pwm_last menjadi treshold
        % b.error antara setpoint dan output
        % c.pwm_desired = pwm_last + Kp(e + SUM(e)/Ti)
        % d.jika pwm_desired dibawah treshold, maka pwm_desired menjadi 0 untuk menghindari stalling
        % e.jika pwm_desired diatas pwm maksimum, maka pwm_desired = 1023


        % ------ 15.Simpan hasil di variabel buffer ------------------ %
        tBuff(i+1,j) = track;
        rBuff(i+1,j) = track;
        xBuff(i+1,j) = xnew;
        yBuff(i+1,j) = ynew;
        lBuff(i+1,j) = lnew;
        vBuff(i+1,j) = speed;
        thBuff(i+1,j) = theta;
        xmBuff(i+1,j) = xmarvel;
        ymBuff(i+1,j) = ymarvel;
        dtBuff(i+1,j) = dtBuff(i,j) + time;
        arduBuff(i+1,j) = ardu_PWM;
        
        
    end                             % end for every train
    % ------ 16.Plot grafik ------------------------------------------ %
    plot(xBuff(i+1,1),yBuff(i+1,1),'.r', ...
        x0(track),y0(track),'xk');
    drawnow
    
end                                 % end for every seconds
hold off; toc

for j = 1:b.n
    publish(myMQTT, eval(['topic' num2str(j)]), '0');   % stop all trains
end