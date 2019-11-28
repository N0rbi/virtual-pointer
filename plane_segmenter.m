function [top_left, top_right, bottom_left, bottom_right] = plane_segmenter(i1, i2, marked_i1, stereoParams)
marked_l = imread(marked_i1);
% marked_r = imread(marked_i2);
left= imread(i1);
right= imread(i2);


[left_r,right_r] = rectifyStereoImages(left, right, stereoParams, 'OutputView','full');

[left_marked_r,~] = rectifyStereoImages(marked_l, right, stereoParams, 'OutputView','full');

A = stereoAnaglyph(left_r,right_r);
imshow(A)

disparityRange = [-80, 0];
ut = 10;
disparityMap = disparitySGM(rgb2gray(left_r),rgb2gray(right_r),'DisparityRange',disparityRange,'UniquenessThreshold', ut);
    
xyzPoints = reconstructScene(disparityMap ,stereoParams);


[h,s,v] = rgb2hsv(left_marked_r);

edges_l = h < 0.1 & s > 0.4 & v > 0.01 & rgb2gray(left_marked_r) > 70; 

[cc, rr, ~] = find(edges_l);

top_left_target = [1,1];
top_right_target = [681, 1];
bottom_left_target = [1, 507];
bottom_right_target = [681, 507];


points = [rr cc];

finite_points = isfinite(xyzPoints);
finite_points = finite_points(:, :, 1) & finite_points(:, :, 2) & finite_points(:, :, 3); 

[cc, rr, ~] = find(finite_points);

valid_points = [rr cc];

top_left = points(closest_index(top_left_target, points), :);
top_right = points(closest_index(top_right_target, points), :);
bottom_left = points(closest_index(bottom_left_target, points), :);
bottom_right = points(closest_index(bottom_right_target, points), :);

top_left = valid_points(closest_index(top_left, valid_points), :);
top_right = valid_points(closest_index(top_right, valid_points), :);
bottom_left = valid_points(closest_index(bottom_left, valid_points), :);
bottom_right = valid_points(closest_index(bottom_right, valid_points), :);

get_coords = @(index) xyzPoints(index(2), index(1), :);

top_left = get_coords(top_left);
top_right = get_coords(top_right);
bottom_left = get_coords(bottom_left);
bottom_right = get_coords(bottom_right);

end