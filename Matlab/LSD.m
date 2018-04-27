clear;
clc;
close all;

%% Step 0: Read input image
im  = rgb2gray(imread('../samples/zebra11.jpg'));
im = imresize(im, [300, 400]);
iim = im;   % for visualization
im = 255 - im;
% black pixels are 0 and white pixels are up to 255

tic;
%% Step 1: Image scaling
% Hyper-parameters
S = 0.8;
E = 0.6;

im = imgaussfilt(im, E/S);
im = imsharpen(im);
im = imadjust(im);
% imshow(iim);

%% Step 2: Gradient computation
% compute the gradient angle and magnitude of all pixels
% -1  1  flip  1  -1
% -1  1  ===>  1  -1
Dx_mask = [1 -1; 1 -1];
% -1  -1  flip   1   1 
%  1   1  ===>  -1  -1
Dy_mask = [1 1; -1 -1];

[M, N] = size(im);
zero_padded_img = zeros(M+1, N+1);
zero_padded_img(1:M,1:N) = im;

gx = conv2(zero_padded_img, Dx_mask, 'valid')./2;
gy = conv2(zero_padded_img, Dy_mask, 'valid')./2;

angle = atan2(-gy, gx);
G = sqrt(gx.^2 + gy.^2);
% figure;
% imshow(gx);
% figure;
% imshow(gy);

%% Step 3: Gradient pseudo ordering (and thresholding)
% Hyper-parameters:
tau = pi/8; % 22.5 deg
q = 2;
% gradient magnitude threshold
threshold = q/sin(tau);

max_gradient = max(max(G));

% 1024 bins [2, n, 1024]
bins = zeros(2,10240,1024);
bin_ends = ones(1024,1);
bin_centers = max_gradient:-max_gradient/1024:max_gradient/1024;

for c = 2:N-1
    for r = 2:M-1
        if G(r,c) > threshold
            dist = abs(bin_centers - G(r,c));
            [~, id] = min(dist);
            bins(:, bin_ends(id), id) = [r; c];
            bin_ends(id) = bin_ends(id) + 1;
        end
    end
end

%% Step 4: Region grow
% output for next step
output_regions = {};
output_rectangles = {};
count = 0;

% Hyper-parameters:
tau = pi/8; % 22.5 deg

% Constants
USED = 1;
NOT_USED = 0;

status = ones(M,N); % 1: USED ---- 0: NOT USED
for c = 2:N
    for r = 2:M
        if G(r,c) > threshold && abs(gy(r,c)) > threshold/2
            status(r, c) = 0;
        end
    end
end

% what bin is active now, starting from 1
iter = 1;

% visualization 1: show original image as background
figure;
imshow(flipud(iim));
set(gca, 'ydir', 'normal');
hold on;

while iter <= 1024
    for i = 1:bin_ends(iter)-1
        pixel = bins(:,i,iter);
        r = pixel(1);
        c = pixel(2);
        if status(r,c) == NOT_USED
            [region, status] = region_grow(angle, pixel, tau, status);
            if length(region(1,:)) < 20
                continue;
            end
            [rect_center, rect_angle, W, L] = rectangle_self(region, G, im);
            if abs(abs(rect_angle) - pi/2) > pi/8
                continue;
            end
            p = 1/8;
            count = count + 1;
            output_regions{count} = region;
            output_rectangles{count} = [rect_center; rect_angle; W; L]; % 1:2 center 3,4,5 angle,W,L
        end
    end
    iter = iter + 1;
end

%% Step 5: Grouping line segments
% create line semgents
%   [rho; theta; xmin; xmax; polarity]
% Note:  theta in [0, pi]

segment_functions = {};
for i = 1:count
    p0 = [output_rectangles{i}(1); output_rectangles{i}(2)];
    n_angle = output_rectangles{i}(3);
    L = output_rectangles{i}(5);
    n = [cos(n_angle); sin(n_angle)];
    rho = (p0 - [1;1])' * n;
    if rho > 0
        theta = n_angle;
    else
        theta = n_angle + pi;
        rho = -rho;
    end
    xmin = p0(1) - abs(L/2*sin(theta));
    xmax = p0(1) + abs(L/2*sin(theta));
    polar = polarity(output_regions{i}, gy);
    segment_functions{i} = [rho; theta; xmin; xmax; polar];
    % visualization 2: plot raw line segments detected
%     x = xmin:0.1:xmax;
%     y = (rho - x.*cos(theta))./ sin(theta);
%     if polar > 0
%         plot(x, y, 'g-', 'linewidth', 0.1);
%     else
%         plot(x, y, 'r-', 'linewidth', 0.1);
%     end
end

% Hyper-parameters:
eps_theta = pi/32;
eps_rho_base = 5;
eps_dist = 5;

% whether a segment is already combined to a final segment
segment_usage = zeros(count,1);
combinations = {};

xdiff = zeros(length(segment_functions),1);
for i = 1:length(segment_functions)
    xdiff(i) = segment_functions{i}(4)-segment_functions{i}(3);
end

[~,sort_idx] = sort(xdiff);

for ii = 1 : count
    i = sort_idx(ii);
    if segment_usage(i) == USED
        continue;
    end
    seed = segment_functions{i};
    wait_list = zeros(6,1);
    wait_list(:,1) = [seed; i];
    segment_usage(i) = USED;
    for jj = ii+1:count
        j = sort_idx(jj);
        if segment_usage(j) == USED
            continue;
        end
        if line_similarity(segment_functions{i}, segment_functions{j}, eps_theta, eps_dist) == 1
            wait_list(:,end+1) = [segment_functions{j}; j];
            segment_usage(j) = USED;
        end
    end
    % WTF -------------------------------------------- %
    wait_list = sortrows(wait_list' ,3)';
    wait_list_size = size(wait_list,2);
    cur_comb = [];    
    cur_xmax = 0;
    tolerance = 10;
    cur_comb(1) = wait_list(6,1);
    cur_xmax = wait_list(4,1);
    for j = 2:wait_list_size
        if cur_xmax + tolerance > wait_list(3,j)
            cur_comb(end+1) = wait_list(6,j);
            cur_xmax = wait_list(4,j);
        else
            combinations{end+1} = cur_comb;
            cur_comb = [];
            cur_comb(1) = wait_list(6,j);
            cur_xmax = wait_list(4,j);
        end    
    end
    combinations{end+1} = cur_comb;
end

% Do the combination
% Hyper-parameter
G_thresh = threshold * 3;

pos_segments = {};
neg_segments = {};
for i=1:length(combinations)
    polar = segment_functions{combinations{i}(1)}(5);
    xmin = segment_functions{combinations{i}(1)}(3);
    xmax = segment_functions{combinations{i}(1)}(4);
    BW = zeros(M,N);
    for j = 1:length(combinations{i})
        idx = combinations{i}(j);
        new_x_min = segment_functions{idx}(3);
        new_x_max = segment_functions{idx}(4);
        if new_x_min < xmin
            xmin = new_x_min;
        end
        if new_x_max > xmax
            xmax = new_x_max;
        end
        for k=1:length(output_regions{idx}(1,:))
            if G(output_regions{idx}(1,k),output_regions{idx}(2,k))>G_thresh
                BW(output_regions{idx}(1,k),output_regions{idx}(2,k))=1;
            end
        end
    end
    BW = flipud(BW);
    [H,T,R] = hough(BW);
    P  = houghpeaks(H,1,'threshold',ceil(0.3*max(H(:))));
    lines = houghlines(BW,T,R,P);
    if length(lines)<1
        continue;
    end
    rho = lines(1).rho;
    theta = lines(1).theta/180*pi;
    
    % remove small segments
    if xmax - xmin < 20
        continue;
    end
    if polar > 0
        pos_segments{end+1} = [rho; theta; xmin; xmax; polar];
    else
        neg_segments{end+1} = [rho; theta; xmin; xmax; polar];
    end
        
    x = xmin:0.1:xmax;
    y = (rho - x.*cos(theta))./ sin(theta);
    
    % visualization 3: plot the grouped line segments with polarity
    if polar > 0
        % plot(x, y, 'g-', 'linewidth', 0.1);
        %text(mean(x), mean(y), num2str(theta), 'color', 'y');
    else
        % plot(x, y, 'r-', 'linewidth', 0.1);
        %text(mean(x), mean(y), num2str(theta), 'color', 'y');
    end
end


%% Step 6: Matching line segments
K1 = length(pos_segments);
K2 = length(neg_segments);
score_matrix = zeros(K1, K2);
% pos->neg

for i = 1:K2
    for j = 1:K1
        score = proximity(pos_segments{j}, neg_segments{i});
        score_matrix(j,i) = score;
    end
end

order = zeros(K1, K2);
for i = 1:K2
    [~, order(:,i)] = sort(score_matrix(:,i));
    order(:,i) = flipud(order(:,i));
end

pos_matched = zeros(K1,1);
pos_left = K1;
neg_matched = zeros(K2,1);
neg_left = K2;

pairs = []; %   [pos_id; neg_id]

[n1, n2] = size(score_matrix);
for j=1:n1
    for i=1:n2
        if score_matrix(j,i) > 0
            if check_color(pos_segments{j}, neg_segments{i}, im)
                pairs(:,end+1) = [j;i];
            end
            % visualization 4: Shade the area of candidates of stripelets
            % shade_area(pos_segments{j}, neg_segments{i}, im);
        end
    end
end

%% Step 7: Construct stripelets using matched line segments
% compute stripe representation of each stripe
stripes = [];
for i = 1:length(pairs(1,:))
    stripes(:,end+1) = stripe(pos_segments{pairs(1,i)}, neg_segments{pairs(2,i)});
end

%% Step 8: Compute connectivity of the stripelets

connectivity_matrix = zeros(length(stripes(1,:)));
for i = 1:length(stripes(1,:))
    for j = i+1:length(stripes(1,:))
        stripe_up = [];
        stripe_down = [];
        if stripes(2,i) < stripes(2,j)
            stripe_up = stripes(:,j);
            stripe_down = stripes(:,i);
        else
            stripe_up = stripes(:,i);
            stripe_down = stripes(:,j);
        end
        connectivity_matrix(i,j) = connectivity(stripe_up, stripe_down);
        connectivity_matrix(j,i) = connectivity_matrix(i,j);
    end
end

for i = 1:length(stripes(1,:))
    for j = i+1:length(stripes(1,:))
        if connectivity_matrix(i,j)
            x = [stripes(1,i),stripes(1,j)];
            y = [stripes(2,i),stripes(2,j)];
            % visualization 5: plot the connected graph of stripelets
            % plot(x,y, 'b-', 'linewidth', 0.2);
        end
    end
end

%% Step 9: Compute envelope
% visualization 6: plot the y-w points of stripelets
% figure;
% hold on;
% for i = 1:length(stripes(1,:))
%     plot(stripes(2,i), stripes(3,i), 'x');
% end

stripe_points = []; % Nx3: id, distance used for sorting
for i = 1:length(stripes(1,:))
    stripe_points(end+1,:) = [i, stripes(2,i), sqrt(stripes(2,i)^2+stripes(3,i)^2)];
end

stripe_points = sortrows(stripe_points, 2);

y0 = stripes(2,stripe_points(1,1));
w0 = stripes(3,stripe_points(1,1));
stripe_points = sortrows(stripe_points, 3);

num = 7;
k = 0;
denom = 0;
for i=1:length(stripe_points(:,1))
    yi = stripes(2,stripe_points(i,1));
    wi = stripes(3,stripe_points(i,1));
    if wi > w0
        continue;
    end
    denom = denom + (y0 - yi)^2;
    k = k + (wi-w0)*(yi-y0);
    num = num - 1;
    if num == 0
        break;
    end
    % visualization 7: plot the points used to estimate the envelope in blue
    %plot(yi, wi, 'b*');
end
k = k/denom;
b = w0 - k*y0;
envelop = [k; -1; b];

% visualization 8: plot the estimated envelop
% y = y0:0.2:170;
% w = -(envelop(3) + y*envelop(1))./envelop(2);
% plot(y,w,'b-');
% envelop is characterized as y*envelop(1)+w*envelop(2)+envelop(3)=0

%% Step 10: Compute unary potential and binary potential
binary_potential = 10/3*exp(-10*connectivity_matrix);
mask = connectivity_matrix>0;
binary_potential = binary_potential.*mask;

unary_potential = zeros(length(stripes(1,:)),1);
for i=1:length(unary_potential)
    point = [stripes(2,i); stripes(3,i); 1];
    E = abs(point'*envelop)/sqrt(k^2+1);
    w_hat = k*stripes(2,i) + b;
    if w_hat < 0
        unary_potential(i) = 0;
    else
        unary_potential(i) = 0.1*min(1, max(0,stripes(10,i)*(1-E/w_hat)));
    end

    % visualization 9: label parameters of each stripelet
    % text(stripes(1,i), stripes(2,i), num2str(i), 'color', 'y');
    % text(stripes(1,i), stripes(2,i), num2str(w_hat), 'color', 'y');
    % text(stripes(1,i), stripes(2,i), num2str(E), 'color', 'y');
    % text(stripes(1,i), stripes(2,i), num2str(point(1)), 'color', 'y');
    % text(stripes(1,i), stripes(2,i), num2str(point(2)), 'color', 'y');
end

%% Step 11: Belief propatation: message update
N = length(stripes(1,:));
old_message_0 = ones(N)/N;
old_message_1 = ones(N)/N;
new_message_0 = zeros(N);
new_message_1 = zeros(N);

unary_potential = unary_potential * 20;
binary_potential = binary_potential * 30;

for iter = 1:20
    for i=1:N
        for j = 1:N
            if i==j
                continue;
            end
            if connectivity_matrix(i,j) == 0
                continue;
            end
            % update for xj = 1
            update_0 = 1;
            update_1 = 1;
            for k = 1:N
                if k==j || k==i
                    continue;
                end
                if connectivity_matrix(k,i) > 0 && old_message_0(k,i) > 0
                    update_0 = update_0 * old_message_0(k,i);
                end
                if connectivity_matrix(k,i) > 0 && old_message_1(k,i) > 0
                    update_1 = update_1 * old_message_1(k,i);
                end
            end
            update_0 = update_0 * 1 * 1;
            update_1 = update_1 * binary_potential(i,j) * unary_potential(i);
            new_message_1(i,j) = update_0 + update_1;
            
            % update for xj = 0
            update_0 = 1;
            update_1 = 1;
            for k = 1:N
                if k==j || k==i
                    continue;
                end
                if connectivity_matrix(k,i) > 0 && old_message_0(k,i) > 0
                    update_0 = update_0 * old_message_0(k,i);
                end
                if connectivity_matrix(k,i) > 0 && old_message_1(k,i) > 0
                    update_1 = update_1 * old_message_1(k,i);
                end
            end
            update_0 = update_0 * 1 * 1;
            update_1 = update_1 * 1 * unary_potential(i);
            new_message_0(i,j) = update_0 + update_1;
        end
    end
    old_message_0 = new_message_0;
    old_message_1 = new_message_1;
    sum = old_message_0 + old_message_1;
    mask = (sum > 0);
    old_message_0(mask) = old_message_0(mask) ./ sum(mask);
    old_message_1(mask) = old_message_1(mask) ./ sum(mask);
end

belief_0 = zeros(N,1);
belief_1 = zeros(N,1);

for i = 1:N
    belief_0(i) = 1;
    for k = 1:N
        if k==i || connectivity_matrix(k,i) == 0 || old_message_0(k,i) == 0
            continue;
        end
        belief_0(i) = belief_0(i) * old_message_0(k,i);
    end
    belief_1(i) = unary_potential(i);
    for k = 1:N
        if k==i || connectivity_matrix(k,i) == 0 || old_message_1(k,i) == 0
            continue;
        end
        belief_1(i) = belief_1(i) * old_message_1(k,i);
    end
end

belief = (belief_1 >= belief_0);

toc;

% visualization 10: shade areas of the finalized true zebra crosswalk stripelets
for i = 1:length(belief)
    if belief(i) > 0
        pos_seg = pos_segments{pairs(1,i)};
        neg_seg = neg_segments{pairs(2,i)};
        shade_area(pos_seg, neg_seg, im);
    end
end