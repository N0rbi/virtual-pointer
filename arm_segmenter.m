function [armA, armB] = arm_segmenter(i1, i2, stereoParams)
    I1 = imread(i1);
    I2 = imread(i2);
    
    [I1_r,I2_r] = rectifyStereoImages(I1, I2, stereoParams, 'OutputView','full');
    A = stereoAnaglyph(I1_r,I2_r);
    imshow(A)

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
    afteropen = imopen(mask, se);
    
    figure;
    imshow(afteropen);
    title("opening");
    
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
    
    imshow(x_filtered);
   
    [rr, cc, ~] = find(x_filtered > 0);
    
    pts_image = [rr, cc];
    
    n = size(pts_image, 1);
    
    points_3d = zeros(n, 3);
    
    for i = 1:n
       coordinates = pts_image(i, :);
       points_3d(i, :) = xyzPoints(coordinates(1),coordinates(2),:);
    end
    
    figure,plot(pts_image(1,:),pts_image(2,:),'.'),hold on
    

    % RANSAC
    iterNum = 300;
    thDist = 2;
    thInlrRatio = .1;
    [t,r] = ransac(pts_image,iterNum,thDist,thInlrRatio);
    k1 = -tan(t);
    b1 = r/cos(t);
    X = 0:600;
    imshow(I1_r);hold on;
    plot(X, k1*X+b1,'r')
    
    
    
    figure;
    imshow(afteropen);
    
    
    a = [168,350]; b = [199,313];
    [b2, b1, ~] = find(afteropen == max(max(afteropen)));
    b(1) = b1(1);
    b(2) = b2(1);
    
    mask = regiongrowing(afteropen, b(2), b(1), 300);
    asd = afteropen .* double(mask);
    asd(asd == 0) = nan;
    [a2, a1, ~] = find(afteropen == min(min(asd)));
    a(1) = a1(1);
    a(2) = a2(1);
    
    line_y1 = a(2);
    line_x1 = a(1);
    line_y2 = b(2);
    line_x2 = b(1);
    
    line_z1 = Z(line_y1, line_x1);
    line_z2 = Z(line_y2, line_x2);
    
    arm_properties = [line_x1 line_y1 line_z1; line_x2 line_y2 line_z2];
end