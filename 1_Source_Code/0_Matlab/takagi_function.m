classdef takagi_function
    properties
        n               % number of train
        t               % time sampling arduino
        x               % nose position of train A,B,C
        v               % velocity of train A,B,C
        vMax            % maximum speed
        a               % deceleration of train A,B,C
        aMax            % max deceleration of train as assumed by the MBSS
        dSafe           % safety margin length under pure MBSS
        k               % a constant
        kb              % braking constant
        bg              % braking compensation
        l               % train length
        s               % station locations and peron length
        sLast           % last station
        tS              % timer how long the train stops in a station
        maxS            % maximum waiting time in a station
        strt_ctrl       % 1 if start control active, 0 otherwise
    end
    methods
        
        % Braking Curve - Train Minimum Distance to "Brick Wall"
        function dmin = tkg_distance(b,j)
            dBB = b.v(j)^2/2/b.aMax;            % ...(2) dBB = vB^2/2/aMax
            dDelay = b.v(j) * b.t(j);
            dmin = b.dSafe + dBB + dDelay;      % ...(1) dmin = dSafe + dBB
        end
        
        % Synchronisation Control of Trains
        function bbeta = tkg_sync(b,j,debugg)
            if j==1
                lastSpeed=b.v(b.n);
                lastDebugg=debugg(b.n);
            else
                lastSpeed=b.v(j-1);
                lastDebugg=debugg(j-1);
            end
            if strcmp(lastDebugg,'brak'), f = b.kb; else, f = 1; end
            bbeta = f * b.aMax*(b.v(j)-lastSpeed)/b.v(j);
            % ...(9) betaB = betaS*(vB-vA)/vB
            % ...(20) betaC = betaS*(vC-vB)/vC
        end
        
        % Simultaneous Starting of Multiple Trains
        function bbeta = tkg_start(b,j)
            if j == 1
                bbeta = -b.k*b.aMax;
            elseif j == 2
                bbeta = -b.aMax*(-1+sqrt(1+4*b.k))/2;
                % ...(18) betaB = -betaS*(-1+sqrt(1+4*k))/2
            elseif j == 3
                bbeta = -b.aMax*(-1+sqrt(-1+2*sqrt(1+4*b.k)))/2;
                % ...(19) betaC = -betaS*(-1+sqrt(-1+2*sqrt(1+4*k)))/2
            end
        end
        
        % Check if there is a train ahead
        function train = trn_ahead(b,j)
            train = 0;
            for h = 1:b.n
                % end to end problem, first calculation is normal distance
                % second case treats point 0 next to 6.12 as continuation
                % find the distance (only take the positive minimum value)
                nose = [ b.x(h) - b.l(h) - b.x(j);
                    6.12 + b.x(h) - b.l(h) - b.x(j) ];
                [nose2tail,~] = min(nose(nose>-0.05));
                % if not this train & { train ahead < min distance
                % || (after start control & train ahead reached max speed) }
                % & station is further than train ahead
                if h~=j && ( nose2tail <= tkg_distance(b,j) + b.bg ...
                        || ( b.strt_ctrl(j) == 1 && b.v(h) >= b.vMax - b.aMax ) ) ...
                        && sta_near(b,j) > nose2tail        % ------- MUNGKIN ADA PROBLEM
                     train = 1;
                end
            end
        end
        
        % Check if there is a station ahead
        function station = sta_ahead(b,j)
            [nearest,n_index] = sta_near(b,j);
            if nearest <= tkg_distance(b,j) + b.bg...
                    && n_index ~= b.sLast(j)
                 station = 1;
            else, station = 0;
            end
        end
        
        % Calculate distance to the nearest station
        function [nearest,n_index] = sta_near(b,j)
            near = zeros(2,1);
            for h=1:size(b.s,2)
                % end to end problem, first calculation is normal distance
                % second case treats point 0 next to 6.12 as continuation
                near(:,h) = [ b.s(2*j-1,h) - b.x(j) + b.dSafe;
                    (6.12 + b.s(2*j-1,h)) - b.x(j) + b.dSafe ];
            end
            % get only the value (distance) and the index (which station)
            % only take the positive minimum value
            [nearest,~] = min(near(near>0));
            [~,n_index] = find(near == min(near(near>0)));
        end
        
        % Check if train in a station
        function dwell = trn_dwell(b,j)
            [nearest,n_index] = sta_near(b,j);
            % if timer still counts & train is behind the peron's end
            % & train is ahead of the peron's start
            if b.tS(j) < b.maxS && nearest >= 0 ...
                    && nearest - b.dSafe < b.s(2*j,n_index)
                 dwell = 1;
            else, dwell = 0;
            end
        end
        
        % Count dwell time in a station
        function cnt = trn_count(b,j,time)
            cnt = b.tS(j) + time;
        end
        
    end
end