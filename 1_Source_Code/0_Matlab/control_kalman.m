close all; clear all; delete (instrfindall); clc; fclose('all'); dbclear if error;


% ----- 1.Communication between NodeMCU and Matlab (PC) using MQTT Protocol ------------------------------ %
myMQTT = mqtt('tcp://192.168.43.248','ClientID','0001','Port',1882);
topic1 = 'SinyalKontrol1'; mySub1 = subscribe(myMQTT,'datasensor1', 'QoS', 0);
topic2 = 'SinyalKontrol2'; mySub2 = subscribe(myMQTT,'datasensor2', 'QoS', 0);

% ----- 2.Reading Track Characteristic Data from Database -------------------------------------- %
n       = csvread('mapDatabase.csv',0,0,[0 0 11 0]);  % Track Number
x0      = csvread('mapDatabase.csv',0,1,[0 1 11 1]);  % Coordinate of Track Segment (x)
y0      = csvread('mapDatabase.csv',0,2,[0 2 11 2]);  % Coordinate of Track Segment (y)
l0      = csvread('mapDatabase.csv',0,3,[0 3 11 3]);  % Position of Track in 1D Coordinate
plusmin = csvread('mapDatabase.csv',0,4,[0 4 11 4]);  % Map Matching Database
xCenter = csvread('mapDatabase.csv',0,5,[0 5 11 5]);  % Center of Curvature (x)
yCenter = csvread('mapDatabase.csv',0,6,[0 6 11 6]);  % Center of Curvature (y)
x0(13) = x0(1); y0(13) = y0(1);




% ----- 4.Setting Takagi Control Parameter ------------------------------- %
b = takagi_function;
b.n = 1;                            % Number of Trains
b.l = 0.27 * ones(1,b.n);           % Length of Train (27 cm)
b.vMax = 0.6;                       % Maximum Velocity (m/s)
b.aMax = 0.3;                       % Maximum Decceleration (m/s^2)
b.dSafe = 0.27;                     % Minimum Headway (m)
b.k = 1;                            % Decceleration Constant

b.tS = zeros(1,b.n);                % Incremental Train Stop Time in Station
b.maxS = 2;                         % Duration of Train Stop in Station (s)
s1 = [0.5 ; 0.5];                     % Small Station 1
s2 = [2 ; 0.5];                   % Small Station 2
s3 = [3.5 ; 0.5];                     % Small Station 3
s4 = [5.3 ; 0.5];                   % Main Station
b.s = [s4, s3;                      % Stop Station 1
    s4, s2;                         % Stop Station 2
    s4, s1];                        % Stop Station 3
b.sLast = zeros(1,b.n);             % Terminus Station
b.strt_ctrl = zeros(1,b.n);         % Initial Condition Start Control
debugg = {'init','init','init'};    % State (Acceleration, Brake, Init)


% ----- 3.Set Kalman Matrix and Parameter ----------------------------- %
f = @(x, v)[ ...
    (x(1)+v(1)) ;
    (x(2)+v(2)) ];                  % (Position) Transition State Matrix
h = @(x, v)[x(1);x(2)];             % (Position) Output Matrix
P = eye(2);                         % (Position) Covariance
sigma_q = 10;                       % (Position) Level of Confidence Prediction
sigma_r = 1;                        % (Position) Level of Confidence Measurement
R = sigma_r^2;                      % (Position) Measurement Noise


% ----- 5.Initialization ------------------------------------------ %
%Kconv = [3, 2, 3]; Kspan = zeros(3,b.n);
[pwmMotor, pwmTreshold] = deal(zeros(1,b.n));
[xKalman, xPred] = deal(zeros(1,2*b.n));
[tBuff, xBuff, yBuff, rBuff, lBuff, vBuff, thBuff, dtBuff, xmBuff, ymBuff, xBool, yBool, vCtrl, eSpeed] ...
    = deal(zeros(1,b.n));
for j=1:b.n
    pause(0.11); dat = read(eval(['mySub' num2str(j)])); datkon = str2num(dat);
    [xKalman(1,2*j-1:2*j), xPred(1,2*j-1:2*j)] = deal([datkon(1)/1000, datkon(2)/1000]);
    vCtrl(1,j) = datkon(4); [pwmMotor(1,j), pwmTreshold(j)] = deal(800);
    %Kspan(:,j) = (pwmTreshold(j) + (1023-pwmTreshold(j))*[6, 10, 15]/15)';
end


% ----- 6.Creating Ideal Track (for Plotting Purposes) ------------------------------------- %
ll = 2; cc = 0.49; rr = 0.165;      % marvelmind distance; curve center; track from point zero
Rad = cc-rr; margin = 0.05;         % curved track radius; marvelmind error margin
figure, rectangle('Position',[rr  rr (ll-2*rr) (ll-2*rr)],'Curvature',(Rad));
axis ([0 ll 0 ll]); hold on



for i = 1:300                      % for every seconds
    for j = 1:b.n                   % for every train
        
        % ----- 7.Data Transfer from NodeMCU ------------------------- %
        pause(0.04);
        dat = read(eval(['mySub' num2str(j)])); %  Reading Data
        datkon = str2num(dat);      % Converting Data from String to Numeric
        xmarvel = datkon(1)/1000;   % Position Data (x) from Indoor GPS Marvelmind Sensor
        ymarvel = datkon(2)/1000;   % Position Data (y) from Indoor GPS Marvelmind Sensor
        rfid = datkon(3);           % Position Data (x,y) from RFID Sensor
        speed = datkon(4);          % Speed Data from Rotary Encoder Data
        time = datkon(5);           % Sampling Time
        Ardu_PWM = datkon(6); disp(Ardu_PWM);


        % ----- 8.Determining Track in Which Train is Located -------------- %
        [ xBool(1), xBool(13)] = deal(xmarvel - x0(1));
        [ yBool(1), yBool(13)] = deal(ymarvel - y0(1));
        for k = 2:size(n)+1
            xBool(k) = xmarvel - x0(k);
            yBool(k) = ymarvel - y0(k);
            if ((xBool(k) * xBool(k-1)) < margin/10 ) && ((yBool(k) * yBool(k-1)) < margin/10 )
                track = n(k-1);
            end
        end


        % ----- 9.Virtual Inertial Measurement Unit (IMU) Algorithm ------------------------------ %
        switch track
            case {1, 2, 4, 5}
                theta = atand( (yCenter(track)-ymarvel) / (xCenter(track)-xmarvel) ) - 90;
            case {7, 8, 10, 11}
                theta = 90 + atand( (yCenter(track)-ymarvel) / (xCenter(track)-xmarvel) );
            case {3, 6, 9, 12}
                theta = (track*30)+180;
        end


        % ----- 11.Kalman Filter Algorithm to Determine Position ------------------------------------- %
        Q = sigma_q^2 * time * eye(2);          % initial noise state / prediksi
        x = xKalman(i,2*j-1:2*j)';
        v = [speed*cosd(theta)*time; speed*sind(theta)*time];
        z = [xmarvel;ymarvel];
        xPred(i+1,2*j-1:2*j) = [xPred(i,2*j-1)+v(1),xPred(i,2*j)+v(2)];
        [x, P, ~] = ekf(f,x,v,P,h,z,Q,R);
        xKalman(i+1,2*j-1:2*j) = x';


        % ----- 12.Map Matching Algorithm to Determine Train Position ------------------------ %
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


        % ----- 13.Conversion from 2D to 1D Position Tracking ------------------------ %
        lnew = l0(track) + sqrt( (xnew-x0(track))^2 + (ynew-y0(track))^2 );


        % ----- 14.Takagi Synchronization Control Algorithm ------------------------------------ %
        b.x(j) = lnew;                          % position
        b.v(j) = vCtrl(i,j);                    % velocity
        if b.v(j) > 0                           % IF train run
            if trn_ahead(b,j)                   %   case 1 : train ahead
                b.a(j) = tkg_sync(b,j);         %       activate sync control
                b.strt_ctrl(j) = 0;             %       release start control switch
                                                debugg{i+1,j} = 'sync';
            elseif sta_ahead(b,j)               %   case 2 : station ahead
                b.a(j) = b.k*b.aMax;            %       brake at max
                                                debugg{i+1,j} = 'brak';
            elseif b.v(j) >= b.vMax      %   case 3 : max speed
                b.a(j) = 0;                     %       stop accelerating
                                                debugg{i+1,j} = 'maxx';
            elseif b.strt_ctrl(j)               %   case 4 : start control switch hold
                b.a(j) = tkg_start(b,j);        %       activate start control
                                                debugg{i+1,j} = 'strt';
            else                                %   case 5 : else
                b.a(j) = -b.k*b.aMax;           %       accelerate at max
                                                debugg{i+1,j} = 'accl';
            end
            b.tS(j) = 0;                        %   after : reset dwell timer after run
        elseif b.v(j) < 0                       % IF speed below minimum
            b.a(j) = 0;                         %       stop accelerating
            b.v(j) = 0;                         %       stop running
                                                debugg{i+1,j} = 'stop';
        else                                    % IF train stop
            if trn_dwell(b,j)                   %   case 1 : train still dwelling
                b.a(j) = 0;                     %       wait
                b.tS(j) = trn_count(b,j,time);  %       count up
                [~,b.sLast(j)] = sta_near(b,j); %       remember the last station
                                                debugg{i+1,j} = 'wait';
            elseif trn_ahead(b,j)               %   case 2 : train ahead
                b.a(j) = tkg_start(b,j);        %       activate start control
                b.strt_ctrl(j) = 1;             %       hold start control switch
                                                debugg{i+1,j} = 'strt';
            else                                %   case 3 : else
                b.a(j) = -b.k*b.aMax;           %       accelerate at max
                                                debugg{i+1,j} = 'accl';
            end
        end
        vCtrl(i+1,j) = b.v(j) - b.a(j)*time;
        
        
        % ------ 15.Pulse Width Modulation (PWM) Motor Control -------------------------------- %
        if pwmMotor(i,j) < pwmTreshold(j), pwmMotor(i,j) = pwmTreshold(j); end          % ...(a)
        %for k=1:3,if pwmMotor(i,j) <= Kspan(k,j), Kpwm = Kconv(k); break; end, end      % ...(b)
        eSpeed(i+1,j) = vCtrl(i+1,j) - vCtrl(i,j);
        pwmMotor(i+1,j) = pwmMotor(i,j) + (1*7.5 * ( eSpeed(i+1,j) ))*time/0.5;        % ...(c)
        if pwmMotor(i+1,j) <= pwmTreshold(j), pwmMotor(i+1,j) = 0; end                  % ...(d)
        if pwmMotor(i+1,j) > 1023, pwmMotor(i+1,j) = 1023; end                          % ...(e)
        publish(myMQTT, eval(['topic' num2str(j)]), num2str(pwmMotor(i+1,j)));          % ...(f)
        % a.if the previous pwm is below the threshold, then pwm_last becomes the threshold
        % b.find the conversion factor(K) between speed and pwm
        % c.pwm_desired = pwm_last + K(speed_desired - speed_kalman)
        % d.if pwm_desired is below the threshold, then pwm_desired becomes 0 to avoid stalling
        % e.if pwm_desired is above the maximum pwm, then pwm_desired = 1023


        % ------ 16.Store The Result in Buffer Variables ------------------ %
        tBuff(i+1,j) = track;
        rBuff(i+1,j) = rfid;
        xBuff(i+1,j) = xnew;
        yBuff(i+1,j) = ynew;
        lBuff(i+1,j) = lnew;
        vBuff(i+1,j) = speed;
        thBuff(i+1,j) = theta;
        dtBuff(i+1,j) = dtBuff(i,j)+time;
        xmBuff(i+1,j) = xmarvel;
        ymBuff(i+1,j) = ymarvel;
        
        
    end                             % end for every train
    % ------ 17.Plot grafik ------------------------------------------ %
    plot(xBuff(i+1,1),yBuff(i+1,1),'.r', ...
        x0(rfid),y0(rfid),'xk');
    drawnow
    pause(0.15);
    
end                                 % end for every seconds
publish(myMQTT, eval(['topic' num2str(j)]), '0');    
hold off;
