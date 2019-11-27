function [arm_properties] = arm_segmenter(i1, i2, stereoParams)
    close all;
    I1 = imread(i1);
    I2 = imread(i2);
    
    I1_bg = imread('data/out_screen_FINAL/0/cam_0.png');
    I2_bg = imread('data/out_screen_FINAL/0/cam_1.png');
    
    figure; imshow(I1_bg);
    figure; imshow(I2_bg);
    
    [I1_r, I2_r] = rectifyStereoImages(I1, I2, stereoParams, 'OutputView','full');
    
    % RGB segmentation----
    [I1_bg_r, ~] = rectifyStereoImages(I1_bg, I2_bg, stereoParams, 'OutputView','full');
    diff1 = I1_bg_r - I1_r;
    th = 5;
    diff_mask = ((diff1(:,:,1) > th) & (diff1(:,:,2) > th) & (diff1(:,:,3)) > th);
    figure; imshow(diff_mask);
    title('Diff mask');
    % RGB segmentation end----
    
    A = stereoAnaglyph(I1_r,I2_r);
    figure;imshow(A)
    title('Red-Cyan composite view of the rectified stereo pair image');

    disparityRange = [-80, 0];
    ut = 10;
    disparityMap = disparitySGM(rgb2gray(I1_r),rgb2gray(I2_r),'DisparityRange',disparityRange,'UniquenessThreshold', ut);
    
    figure;
    imshow(disparityMap,disparityRange)
    colormap(gca,jet) 
    colorbar
    title('Disparity Map');
    colormap jet;
    colorbar;
    dd = disparityMap;
    
    dd(~isfinite(dd)) = 0;
    dd = (dd - min(min(dd)));
    dd = dd / max(max(dd));
    [L,Centers] = imsegkmeans(dd,3);
    [~, I] = sort(Centers);
    furthest_claster = I(1);
    B = labeloverlay(dd,L);
    
    mask = L == furthest_claster;
    
    
    xyzPoints = reconstructScene(disparityMap ,stereoParams);

    [y, x] = ndgrid(1:size(mask, 1), 1:size(mask, 2));
    centroid = mean([x(mask), y(mask)]);
     
%     left = imdilate(mask, strel('line', 15, 45));
%     right = imdilate(mask, strel('line', 15, -45));
%     mask = left | right;
    imshow(uint8(regiongrowing(mask, round(centroid(2)), round(centroid(1)), 0.15)) .* I1_r);
    
    mask = regiongrowing(mask, round(centroid(2)), round(centroid(1)), 0.15);
    
    se = strel('disk', 1);
    afteropen1 = imopen(mask, se);
    
    figure;
    imshow(afteropen1);
    title("opening");
    afteropen = (diff_mask == 1 & 1 == afteropen1);
    %afteropen = afteropen1;
    figure;
    imshow(afteropen);
    title("break")
%     afteropen = imfill(afteropen,'holes');
%     se = strel('disk',5);
%     afteropen = imclose(afteropen,se);
%     figure;
%     imshow(afteropen);
%     title("fill hole")
    
    width = size(afteropen, 2);
    
     afteropen_l = afteropen(:, 1:round(width/2));
     afteropen_r = afteropen(:, round(width/2):end);
    
     afteropen_sum_x = sum(afteropen, 1);
     cut_at = [0,0];
     if sum(afteropen_l, 'all') > sum(afteropen_r, 'all')
         diff_x = diff(fliplr(afteropen_sum_x));
         [~, i] = max(diff_x);
         
         cut_at = [1, width - i];
     else
         diff_x = diff(afteropen_sum_x);
         [~, i] = max(diff_x);
         cut_at = [i, width-1];
     end
     
     x_filtered = afteropen;
     
     x_filtered(:, cut_at(1):cut_at(2)) = 0;
    
    figure;
    imshow(x_filtered);
   
    [rr, cc, ~] = find(x_filtered > 0);
    
    pts_image = [rr, cc];
    
    n = size(pts_image, 1);
    
    points_3d = zeros(n, 3);
    
    for i = 1:n
       coordinates = pts_image(i, :);
       points_3d(i, :) = xyzPoints(coordinates(1),coordinates(2),:);
    end
    
    %figure,plot(pts_image(1,:),pts_image(2,:),'.'),hold on
    
    % NEW RANSAC
    
%     scatter3(points_3d(:,1),points_3d(:,2), points_3d(:,3), 'o');
%     hold on
    
%     sampleSize = 2; % number of points to sample per trial
%     maxDistance = 2; % max allowable distance for inliers
% 
%     %fitLineFcn = @(points) polyfit(points(:,1),points(:,2),points(:,3), 1); % fit function using polyfit
%     fitLineFcn = @(points) my_fit_func(points(:,1),points(:,2),points(:,3), 1); % fit function using polyfit
%     evalLineFcn = ...   % distance evaluation function
%         @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);
% 
%     [modelRANSAC, inlierIdx] = ransac(points_3d, fitLineFcn, evalLineFcn, sampleSize,maxDistance);
% 
%     modelInliers = polyfit(points(inlierIdx,1),points(inlierIdx,2),1);
    
    
    
    [N, ~] = size(points_3d(:,1));
    % Find line of best fit (in least-squares sense) through X
    % -------------------------------------------------------------------------
    X_ave=mean(points_3d,1);            % mean; line of best fit will pass through this point  
    dX=bsxfun(@minus,points_3d,X_ave);  % residuals
    C=(dX'*dX)/(N-1);           % variance-covariance matrix of X
    [R,D]=svd(C,0);             % singular value decomposition of C; C=R*D*R'
    % NOTES:
    % 1) Direction of best fit line corresponds to R(:,1)
    % 2) R(:,1) is the direction of maximum variances of dX 
    % 3) D(1,1) is the variance of dX after projection on R(:,1)
    % 4) Parametric equation of best fit line: L(t)=X_ave+t*R(:,1)', where t is a real number
    % 5) Total variance of X = trace(D)
    % Coefficient of determineation; R^2 = (explained variance)/(total variance)
    D=diag(D);
    R2=D(1)/sum(D);
    % Visualize X and line of best fit
    % -------------------------------------------------------------------------
    % End-points of a best-fit line (segment); used for visualization only 
    x=dX*R(:,1);    % project residuals on R(:,1) 
    x_min=min(x);
    x_max=max(x);
    dx=x_max-x_min;
    Xa=(x_min-0.05*dx)*R(:,1)' + X_ave;
    Xb=(x_max+0.05*dx)*R(:,1)' + X_ave;
    X_end=[Xa;Xb];
    figure('color','w')
    axis equal 
    hold on
    plot3(X_end(:,1),X_end(:,2),X_end(:,3),'-r','LineWidth',3) % best fit line 
    plot3(points_3d(:,1),points_3d(:,2),points_3d(:,3),'.k','MarkerSize',13)           % simulated noisy data
    set(get(gca,'Title'),'String',sprintf('R^2 = %.3f',R2),'FontSize',25,'FontWeight','normal')
    xlabel('X','FontSize',20,'Color','k')
    ylabel('Y','FontSize',20,'Color','k')
    zlabel('Z','FontSize',20,'Color','k')
    view([20 20])
    drawnow
    
    
    
%     % RANSAC
%     iterNum = 300;
%     thDist = 2;
%     thInlrRatio = .1;
%     [t,r] = ransac(pts_image,iterNum,thDist,thInlrRatio);
%     k1 = -tan(t);
%     b1 = r/cos(t);
%     X = 0:600;
%     imshow(I1_r);hold on;
%     plot(X, k1*X+b1,'r')
%     
%     
%     
%     figure;
%     imshow(afteropen);
%     
%     
%     a = [168,350]; b = [199,313];
%     [b2, b1, ~] = find(afteropen == max(max(afteropen)));
%     b(1) = b1(1);
%     b(2) = b2(1);
%     
%     mask = regiongrowing(afteropen, b(2), b(1), 300);
%     asd = afteropen .* double(mask);
%     asd(asd == 0) = nan;
%     [a2, a1, ~] = find(afteropen == min(min(asd)));
%     a(1) = a1(1);
%     a(2) = a2(1);
%     
%     line_y1 = a(2);
%     line_x1 = a(1);
%     line_y2 = b(2);
%     line_x2 = b(1);
%     
%     line_z1 = Z(line_y1, line_x1);
%     line_z2 = Z(line_y2, line_x2);
%     
     %arm_properties = [line_x1 line_y1 line_z1; line_x2 line_y2 line_z2];
end