classdef takeuchi_function
    properties
        n               % number of train
        vMax            % maximum speed
        aMax            % max deceleration of train as assumed by the FBSS
        RFID            % block the train are in
        s               % station locations and peron length
        sLast           % last station
        tS              % timer how long the train stops in a station
        maxS            % maximum waiting time in a station
    end
    methods
        
        % Define block length
        function bl = block_length(c)
            bl = c.vMax^2/(2*c.aMax)/3;             % ... (2)
        end
        
        % Determine the remaining block from train in front
        function tb = train_block(c,j)
            tr_bl = zeros(2,1);
            for h = 1:c.n
                if j ~= h
                tr_bl(:,h) = [c.RFID(h) - c.RFID(j) + 1;
                    10 + c.RFID(h) - c.RFID(j) + 1];
                else, tr_bl(:,h) = [-1, -1];
                end
            end
            tb = min(tr_bl(tr_bl>0));
        end
        
        % Calculate speed of the train at remaining block
        function sp = speed_calc(c,tb)
            sp_data = [0, 0, 0, c.vMax * sqrt(1-2/3), c.vMax * sqrt(1-1/3), c.vMax];  % ... (3)
            if tb > 6, tb = 6; end
            sp = sp_data(tb);
        end
        
        % Determine the remaining block from station
        function [sb,sIndex] = station_block(c,j)
            st_bl = zeros(2,1);
            for h = 1:size(c.s,2)
                st_bl(:,h) = [c.s(2*j-1,h) - c.RFID(j) + 1;
                    10 + c.s(2*j-1,h) - c.RFID(j) + 1];
            end
            [sb,~] = min(st_bl(st_bl>0));
            [~,sIndex] = find(st_bl == min(st_bl(st_bl>0)));
            if sIndex == c.sLast(j), [sb,~] = min(st_bl(st_bl>1)); end
        end
        
        % Calculate speed to station
        function sp = station_calc(c,sb)
            sp_data = [0, c.vMax * sqrt(1-2/3), c.vMax * sqrt(1-1/3), c.vMax];
            if sb > 4, sb = 4; end
            sp = sp_data(sb);
        end
        
        % Waiting time
        function chk = check_in_station(c,j)
            chk = 0;
            for h = 1:size(c.s,2)
                if c.RFID(j) == c.s(2*j-1,h)
                    chk = 1;
                end
            end
        end
                
    end
end