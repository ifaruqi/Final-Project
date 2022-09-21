close all; clear; delete (instrfindall); clc; fclose('all');


% ----- 1.Komunikasi MQTT nodemcu matlab ------------------------------ %
myMQTT = mqtt('tcp://192.168.43.248','ClientID','0001','Port',1882);
topic1 = 'SinyalKontrol1'; mySub1 = subscribe(myMQTT,'datasensor1', 'QoS', 0);
topic2 = 'SinyalKontrol2'; mySub2 = subscribe(myMQTT,'datasensor2', 'QoS', 0);


% ----- 2.Set parameter kontrol Takeuchi ------------------------------ %
c = takeuchi_function;
c.n = 2;                            % jumlah kereta
c.vMax = 0.6;                       % kecepatan maksimum (m/s)
c.aMax = 0.1;                       % perlambatan maksimum (m/s^2)
sp_data = [0, c.vMax * sqrt(1-2/3), c.vMax * sqrt(1-1/3), c.vMax];
bl = block_length(c);

c.tS = zeros(1,c.n);                % incremental timer waktu kereta berhenti di stasiun
c.maxS = 3;                         % lama waktu maksimum kereta berhenti di stasiun (s)
s1 = [floor(1/bl) ; 0.3];           % stasiun kecil 1
s2 = [floor(2.5/bl) ; 0.3];         % stasiun kecil 2
s3 = [floor(4/bl) ; 0.3];           % stasiun kecil 3
s4 = [floor(5.5/bl) ; 1];           % stasiun besar
c.s = [s4, s3;                      % stasiun tempat kereta 1 berhenti
    s4, s2;                         % stasiun tempat kereta 2 berhenti
    s4, s1];                        % stasiun tempat kereta 3 berhenti
c.sLast = zeros(1,c.n);             % stasiun terakhir berhenti


% ----- 3.Inisiasi data awal ------------------------------------------ %
[rBuff, vCtrl, pwmMotor, pwmTreshold, arduBuff, lastSpeed, sb, tb, sIndex] = deal(zeros(1,c.n));
[pwmMotor(1,1), pwmTreshold(1)] = deal(500);      % set pwm treshold
[pwmMotor(1,2), pwmTreshold(2)] = deal(650);      % set pwm treshold
rBuff(1,1) = 2;
rBuff(1,2) = 3;
time = 0.04;



for i = 1:800                       % for every seconds
    for j = 1:c.n                   % for every train
        
        % ----- 4.Transfer data dari nodemcu ------------------------- %
        pause(0.04);
        dat = read(eval(['mySub' num2str(j)])); % membaca data
        datkon = str2num(dat);   % konversi tipe data string menjadi numerik
        rfid = datkon(1);           % data nomer RFID
        ardu_PWM = datkon(2);       % data pwm yang diterima nodemcu


        % ----- 5.Menentukan track dimana kereta berada -------------- %
        if rfid == 0, rfid = rBuff(i,j); end


        % ----- 6.Kontrol Takeuchi ----------------------------------- %
        c.RFID = rBuff(i,:);
        [sb(i+1,j),sIndex(j)] = station_block(c,j);
        tb(i+1,j) = train_block(c,j);
        if  tb(i+1,j) <= sb(i+1,j)
            vCtrl(i+1,j) = speed_calc(c,tb(i+1,j));
        else
            vCtrl(i+1,j) = station_calc(c,sb(i+1,j));
        end
        if check_in_station(c,j)
            c.tS(j) = c.tS(j) + time;
            if c.tS(j) >= c.maxS
                c.tS(j) = 0;
                [~,c.sLast(j)] = station_block(c,j);
            end
        end
        lastSpeed(j) = vCtrl(i+1,j);
        
        
        % ------ 7.Kontrol pwm motor --------------------------------- %
        if vCtrl(i+1,j) == sp_data(1), pwmMotor(i+1,j) = 0;
        elseif vCtrl(i+1,j) == sp_data(2), pwmMotor(i+1,j) = (1 + 1/3/3) * pwmTreshold(j);
        elseif vCtrl(i+1,j) == sp_data(3), pwmMotor(i+1,j) = (1 + 2/3/3) * pwmTreshold(j);
        elseif vCtrl(i+1,j) == sp_data(4), pwmMotor(i+1,j) = (1 + 3/3/3) * pwmTreshold(j);
        else, pwmMotor(i+1,j) = 0;
        end
        publish(myMQTT, eval(['topic' num2str(j)]), num2str(pwmMotor(i+1,j)));          % ...(f)


        % ------ 8.Simpan hasil di variabel buffer ------------------- %
        rBuff(i+1,j) = rfid;
        arduBuff(i+1,j) = ardu_PWM;
        
    end                             % end for every train
end                                 % end for every seconds

for j = 1:c.n
    publish(myMQTT, eval(['topic' num2str(j)]), '0');   % stop all trains
end