function [ region, status ] = region_grow( LLA, P, tau, status )
% REGION_GROW 
%   input: level-line field LLA:    MxN   ----- angles at each pixel
%          seed pixel P:            2x1   ----- pixel to grow from in rc
%          space
%          angle tolerance tau:     R     ----- scalar hyper-parameter
%          status:                  MxN   ----- binary mask
%   output: region:                 2xVar ----- coordinate of pixels in
%   this region rc space

% e.g. [160, 291]

    USED = 1;
    NOT_USED = 0;
    [M, N] = size(LLA);
    region = zeros(2,1);
    added = zeros(2,1);
    region(:,1) = P;
    added(:,1) = P;
    r = P(1);
    c = P(2);
    status(r, c) = USED;
    theta_region = LLA(r, c);
    Sx = cos(theta_region);
    Sy = sin(theta_region);
    
    offset = [[-1;-1],[-1;0],[-1;1],[0;-1],[0;1],[1;-1],[1;0],[1;1]];
    % BFS for region grow
    has_new_pixel = 1;
    while has_new_pixel
        new_added = zeros(2,1);
        for i = 1:length(added(1,:))
            has_new_pixel = 0;
            pixel = added(:,i); % [r; c]
            for j = 1:length(offset)
                neighbor = pixel + offset(:,j);
                if isValidPixel(neighbor, M, N) && status(neighbor(1), neighbor(2)) == NOT_USED
                    if angle_diff(theta_region, LLA(neighbor(1), neighbor(2))) < tau
                        region(:, end+1) = neighbor;
                        status(neighbor(1), neighbor(2)) = USED;
                        Sx = Sx + cos(LLA(neighbor(1), neighbor(2)));
                        Sy = Sy + sin(LLA(neighbor(1), neighbor(2)));
                        theta_region = atan2(Sy,Sx);
                        
                        % maintain search table
                        if length(new_added(1,:)) == 1
                            new_added(:,1) = neighbor;
                            has_new_pixel = 1;
                        else
                            new_added(:,end+1) = neighbor;
                        end
                    end
                end
            end
        end
        added = new_added;
    end
end

