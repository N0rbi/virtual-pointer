load('new_calib.mat');
resultList = [];
targetList = [];


[plane_top_left, plane_top_right, plane_bottom_left, plane_bottom_right] = plane_segmenter('data/out_screen_FINAL/0/cam_0.png', 'data/out_screen_FINAL/0/cam_1.png', 'data/out_screen_FINAL/0/cam_0_FINAL.png', stereoParams);

for i=1:2
    close all;
    label = importdata(['data/out_test_FINAL/', num2str(i-1) , '/label.txt']);
    string = label(1);
    ss = split(string{1}(2:end-1), ',');
    dim1 = str2num(ss{1});
    dim2 = str2num(ss{2});
    
    target = [dim1, dim2];
    
     try
        [result] = calculate_intersection(['data/out_test_FINAL/', num2str(i), '/cam_0.png'], ['data/out_test_FINAL/', num2str(i), '/cam_1.png'], stereoParams, plane_top_left, plane_top_right, plane_bottom_left);
        resultList = [resultList; result];
        targetList = [targetList; target];
     catch ME
         resultList = [resultList; [nan, nan]];
         targetList = [targetList; target];
         disp(ME)
         continue
     end
    mean_diff = sum((targetList-resultList).^2, 2);
    errors = sum(isnan(mean_diff));
    mean_error = nanmean(mean_diff);
end
