function [Scan,rotS,trans] = accurateMatch(GNSS, Scan, tOff, rotS, trans, iter, bound, sub, del)
% Function performs an accurate trajectory match for the provided GNSS and
% Scan trajectory and returns the transformed trajectory as well as the
% transformation parameters
% -------------------------------------------------------------------------
% Autor: SIMP-Project Team
% -------------------------------------------------------------------------
% Trajectories have to have matched times! The matching is performed
% iteratively to get the best solution.
% -------------------------------------------------------------------------
% Input:    GNSS    = GNSS trajectory [time [s], X [m], Y [m], Z [m]]
%           Scan    = Scan trajectory [time [s], X [m], Y [m], Z [m] ...]
%           tOff    = Time offset between GNSS and Scan trajectory [s]
%           rotS    = Prior rotation and scale of the scan trajectory to 
%                     concatenate the final rotation and scale
%           trans   = Prior offset of the Scan trajectory to concatenate
%                     the final offset
%           iter    = Number of iterations for ICP
%           bound   = Size of threshold to check for closest points
%           sub     = Subsampling ratio (2 equals to every second point...)
%           del     = percentage of worst points to delete each iteration
% Output:   Scan    = Scan trajectory [time [s], X [m], Y [m], Z [m] ...]
%           rotS    = Combined rotation and scale of coarse transformation
%           trans   = Translation of coarse transformation [m]
% -------------------------------------------------------------------------

% figure;

% Get mean time step size
ScanTSS = mean(diff(Scan(:,1)));

% Find iterative closest points
match = zeros(size(GNSS,1),3);
for t = 1:iter 
    % Find closest Scan point for each point in GNSS trajectory
    for i = 1:length(GNSS)
        % Get approximate Scan index
        idx = round((GNSS(i,1)-tOff)/ScanTSS)+1;
        if idx > length(Scan)
            idx = length(Scan);
        end

        % Estimate lower and upper bound for threshold
        if idx-round(bound/2) < 1
            idxLB = 1;
        else
            idxLB = idx-round(bound/2);
        end

        if idx+round(bound/2) > length(Scan)
            idxUB = length(Scan);
        else
            idxUB = idx+round(bound/2);
        end

        % Find closest Scan point in threshold and save index and distance
        [~, matchIDX] = min(vecnorm(Scan(idxLB:idxUB,2:4)-GNSS(i,2:4),2,2));
        dist = norm(Scan(idxLB + matchIDX - 1,2:4)-GNSS(i,2:4));
        match(i,:) = [i, idxLB + matchIDX - 1, dist];   % order: [GNSSIDX, ScanIDX, distance]
    end
    
    % Delete the worst del [%] matches (biggest difference)
    [~, sortIDX] = sort(match(:,3));        % sort by distance
    delElem = round(del/100*length(match)); % determine number of elements to be deleted
    sortIDX = sortIDX(1:end-delElem);       % delete elements
    scanIDX = sort(sortIDX);                % remaining GNSS indices
    match = match(scanIDX,:);               % update match list
    
    % Subsample match list
    match = match(1:sub:end,:);
    
    % Estimate transformation
    trafoParam = Est9Trafo3D(Scan(match(:,2),2:4),GNSS(match(:,1),2:4),[1 1 1 0 0 0 0 0 0]',1e-1^t);

    % Rotate scanner trajectory
    Scan(:,2:4) = Trafo9(Scan(:,2:4),trafoParam);
    
    % Delete bad GNSS points
    GNSS = GNSS(scanIDX,:);
    
    % Update complete transformation
    rotScaleNew = [trafoParam(1) 0 0; 0 trafoParam(2) 0; 0 0 trafoParam(3)]*...
                  [cos(trafoParam(6)) -sin(trafoParam(6)) 0; sin(trafoParam(6)) cos(trafoParam(6)) 0; 0 0 1]*...
                  [cos(trafoParam(5)) 0 sin(trafoParam(5)); 0 1 0; -sin(trafoParam(5)) 0 cos(trafoParam(5))]*...
                  [1 0 0; 0 cos(trafoParam(4)) -sin(trafoParam(4)); 0 sin(trafoParam(4)) cos(trafoParam(4))];          
    trans = trafoParam(7:9) + rotScaleNew*trans;
    rotS = rotScaleNew*rotS;
    
    % Reset match
    match = [];
end
end