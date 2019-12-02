function [result] = arm_segmenter(base_dir, stereoParams, number_of_img)
close all;
number_of_img_str = int2str(number_of_img);
i1 = strcat(base_dir, '/', number_of_img_str, '/cam_0.png');
i2 = strcat(base_dir, '/', number_of_img_str, '/cam_1.png');

I1 = imread(i1);
I2 = imread(i2);

I1_bg = imread('data/out_screen_FINAL/0/cam_0.png');
I2_bg = imread('data/out_screen_FINAL/0/cam_1.png');

% figure; imshow(I1_bg);
% figure; imshow(I2_bg);

[I1_r, I2_r] = rectifyStereoImages(I1, I2, stereoParams, 'OutputView','full');

% RGB segmentation----
[I1_bg_r, ~] = rectifyStereoImages(I1_bg, I2_bg, stereoParams, 'OutputView','full');
diff1 = I1_bg_r - I1_r;
th = 5;
diff_mask = ((diff1(:,:,1) > th) & (diff1(:,:,2) > th) & (diff1(:,:,3)) > th);
figure; imshow(diff_mask);
diff_mask = imfill(diff_mask,'holes');
title('Bg and human diff mask');
%mkdir('out_test_FINAL_arm_results', number_of_img_str);
%saveas(gcf, strcat('out_test_FINAL_arm_results/', number_of_img_str, '/diff_mask_', number_of_img_str, '.png'));
% RGB segmentation end----

A = stereoAnaglyph(I1_r,I2_r);
figure;imshow(A)
title('Red-Cyan composite view of the rectified stereo pair image');
%saveas(gcf, strcat('out_test_FINAL_arm_results/', number_of_img_str, '/rect_images_', number_of_img_str, '.png'));

disparityRange = [-80, 0];
ut = 10;
disparityMap = disparitySGM(rgb2gray(I1_r),rgb2gray(I2_r),'DisparityRange',disparityRange,'UniquenessThreshold', ut);

figure;
imshow(disparityMap,disparityRange);
title('Disparity map');
%saveas(gcf, strcat('out_test_FINAL_arm_results/', number_of_img_str, '/disp_map_', number_of_img_str, '.png'));
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

%% Emil pcfit

%show the 3d plots
debugfig = 1;

xPoints = xyzPoints(:,:,1);
yPoints = xyzPoints(:,:,2);
zPoints = xyzPoints(:,:,3);

xyz = [xPoints(:),yPoints(:),zPoints(:) ]';
xyz_t = xyz';
xyz_t(any(isnan(xyz_t), 2), :) = [];
%pointcloud from the reconstructed scene
ptCloud = pointCloud(xyz_t);

if debugfig
    figure;
    
    pcshow(ptCloud);
    xlabel('X(m)')
    ylabel('Y(m)')
    zlabel('Z(m)')
    %roi = [100, 500, 100, 500, 5000, 5003];
    %indices = findPointsInROI(ptCloud,roi);
    %ptCloudB = select(ptCloud,indices);
    %plane_model = pcfitplane(ptCloudB, 0.1, 'MaxNumTrials', 100000);
    %pcshow(ptCloud); hold on; plot(plane_model);
    % [ 273.5830  153.9863] 433.1823  244.1616
    
    %for test
    %BB_coordinates = [221, 105, 496, 365];
    BB_coordinates = [270, 70, 500, 360]; %nagyobb bbvel tesztelve
    BB_coordinates = [219, 130, 483, 281];
    tl_x = BB_coordinates(1, 1);
    tl_y = BB_coordinates(1,2);
    br_x_end = BB_coordinates(1,3);
    br_y_end = BB_coordinates(1,4);
    
    
    %get the XY coordinates of the BB
    x = tl_x:5:br_x_end;
    y = tl_y:5:br_y_end;
    
    [XX,YY] = meshgrid(x,y);
    pts = [XX(:),YY(:)];
    BB_point_2d = (pts)';
    
    
    for s=1:length(pts)
        z = zPoints(pts(s, 1), pts(s,2));
        testdataforplane(s,:) = [pts(s, 1), pts(s,2),z];
    end
    testdataforplane(any(isnan(testdataforplane), 2), :) = [];
    ptCloudROI = pointCloud(testdataforplane);
    med = median(testdataforplane(:,3));
    
    ttt = testdataforplane(any(testdataforplane(:,3) < (med + 3), 3), :);
    tttt = ttt(any(ttt(:,3) > (med - 3), 3), :);
    
    planePT = pointCloud(tttt);
    
    %selected = select(ptCloudROI, ind);
    %plane_model = pcfitplane(planePT, 0.01, 'MaxNumTrials', 100000, 'Confidence', 99.99);
    load('plane_model.mat');
    figure; pcshow(xyz_t); hold on; plot(plane_model);
end





%%

[y, x] = ndgrid(1:size(mask, 1), 1:size(mask, 2));
centroid = mean([x(mask), y(mask)]);

%     left = imdilate(mask, strel('line', 15, 45));
%     right = imdilate(mask, strel('line', 15, -45));
%     mask = left | right;
figure;
imshow(uint8(regiongrowing(mask, round(centroid(2)), round(centroid(1)), 0.15)) .* I1_r);
title('Regiongrowing')
%saveas(gcf, strcat('out_test_FINAL_arm_results/', number_of_img_str, '/reggrow_', number_of_img_str, '.png'));

mask = regiongrowing(mask, round(centroid(2)), round(centroid(1)), 0.15);

se = strel('disk', 1);
afteropen1 = imopen(mask, se);

figure;
imshow(afteropen1);
afteropen1 = imfill(afteropen1,'holes');
title("Opening to reduse the noise, and fill hole");
%saveas(gcf, strcat('out_test_FINAL_arm_results/', number_of_img_str, '/open_', number_of_img_str, '.png'));

afteropen = (diff_mask == 1 & 1 == afteropen1);

%afteropen = afteropen1;
se = strel('disk', 5);
afteropenclose = imclose(afteropen, se);
figure;
imshow(afteropenclose);
title("Intersect with bg and human difference and closing to clear the holes")
%saveas(gcf, strcat('out_test_FINAL_arm_results/', number_of_img_str, '/inter_and_close_', number_of_img_str, '.png'));
%     afteropen = imfill(afteropen,'holes');
%     se = strel('disk',5);
%     afteropen = imclose(afteropen,se);
%     figure;
%     imshow(afteropen);
%     title("fill hole")

width = size(afteropenclose, 2);

afteropen_l = afteropenclose(:, 1:round(width/2));
afteropen_r = afteropenclose(:, round(width/2):end);

afteropen_sum_x = sum(afteropenclose, 1);
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
title('Arms cutted down, calculated on the closed image but applyed on the opened image');
%saveas(gcf, strcat('out_test_FINAL_arm_results/', number_of_img_str, '/cuted_', number_of_img_str, '.png'));

[rr, cc, ~] = find(x_filtered > 0);

pts_image = [rr, cc];

n = size(pts_image, 1);

points_3d = zeros(n, 3);

for i = 1:n
    coordinates = pts_image(i, :);
    if ~isnan(xyzPoints(coordinates(1),coordinates(2),:))
        points_3d(i, :) = xyzPoints(coordinates(1),coordinates(2),:);
        
    end
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

[model, CuboidParameters, inlierIndices, outlierIndices] = CuboidRANSAC( points_3d );

[size_inliers, ~] = size(inlierIndices);
[size_outliers, ~] = size(outlierIndices);

points_3d_ran = double.empty();
points_3d_out = double.empty();
for i = 1:size_inliers
    points_3d_ran = [points_3d_ran; points_3d(inlierIndices(i),:)];
end

for i = 1:size_outliers
    points_3d_out = [points_3d_out; points_3d(outlierIndices(i),:)];
end

[N, ~] = size(points_3d(:,1));
% Find line of best fit (in least-squares sense) through X
% -------------------------------------------------------------------------
X_ave=mean(points_3d_ran,1);            % mean; line of best fit will pass through this point
dX=bsxfun(@minus,points_3d_ran,X_ave);  % residuals
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
plot3(X_end(:,1),X_end(:,2),X_end(:,3),'-g','LineWidth',3) % best fit line
plot3(points_3d_ran(:,1),points_3d_ran(:,2),points_3d_ran(:,3),'.k','MarkerSize',13); hold on;     % simulated noisy data
plot3(points_3d_out(:,1),points_3d_out(:,2),points_3d_out(:,3),'.b','MarkerSize',13)
title('Blue - out; Black - in, Red - line')
%saveas(gcf, strcat('out_test_FINAL_arm_results/', number_of_img_str, '/line_', number_of_img_str, '.png'));
%set(get(gca,'Title'),'String',sprintf('R^2 = %.3f',R2),'FontSize',25,'FontWeight','normal')

xlabel('X','FontSize',20,'Color','k')
ylabel('Y','FontSize',20,'Color','k')
zlabel('Z','FontSize',20,'Color','k')
view([20 20])
drawnow

%% Emil
id = find(testdataforplane(:,3) == med);
%P1 = testdataforplane(id, :);
load('P1.mat');
[I,check] = plane_line_intersect(plane_model.Normal,P1, X_end(1,:), X_end(2,:));
figure;
pcshow(xyz_t); hold on;
plot(plane_model);
v = [X_end; I];
plot3(v(:,1),v(:,2),v(:,3),'-g','LineWidth',3)
plot3(X_end(:,1),X_end(:,2),X_end(:,3),'-b','LineWidth',2)
plot3(I(1),I(2),I(3),'-b*', 'Markersize',20);

xy = (stereoParams.CameraParameters1.IntrinsicMatrix' * I');
xy = xy';
xy(:,1) = xy(:,1)./xy(:,3);
xy(:,2) = xy(:,2)./xy(:,3);
xy(:,3) = xy(:,3)./xy(:,3);

figure; imshow(I1); hold on;  hold on; plot(xy(1), xy(2),'-k*', 'MarkerSize',20);

proj_tl_pixel = [223,133];
proj_tr_pixel = [482, 140];
proj_bl_pixel = [223, 279];

point = xy(1:2);

projWidth = 1920;
projHeight = 1080;

A_x_ratio = proj_tr_pixel(1) - proj_tl_pixel(1);
A_y_ratio = proj_bl_pixel(2) - proj_tl_pixel(2);

A_x_ratio = A_x_ratio / projWidth;
A_y_ratio = A_y_ratio / projHeight;

pointXScale = point(1) - proj_tl_pixel(1);
pointX = pointXScale/A_x_ratio;

pointYScale = point(2) - proj_tl_pixel(2);
pointY = pointYScale/A_y_ratio;
result = [pointX, pointY];
%plotCamera('Location', translationVector, 'Orientation', rotationMatrix, 'Size', 100);
%hold on; plot3(camera_position(1),camera_position(2),camera_position(3),'-y*', 'MarkerSize',100);
%camera_position=rotationMatrix(1:3,1:3)' * translationVector';
%camera_position= GT_pose(1:3,4); % Edit: Robi
%GT_pose = [rotationMatrix, translationVector'; 0 0 0 1];
%PointProjectionToPlane(200, 300, stereoParams.CameraParameters1.IntrinsicMatrix', GT_pose, plane_model.Normal, plane_model.Parameters(4));
%figure;
%imshow(I1); hold on;
%plot(I(1), -I(2),'-r*', 'Markersize',20)
%%
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