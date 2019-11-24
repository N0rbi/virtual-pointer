load('new_calib.mat');
resultList = [];
targetList = [];


[plane_properties] = plane_segmenter('data/out_screen_FINAL/0/cam_0.png', 'data/out_screen_FINAL/0/cam_0.png', 'data/patterns/pattern2.png', stereoParams);

for i=1:30
    close all;
    label = importdata(['data/out_test_FINAL/', num2str(i-1) , '/label.txt']);
    string = label(1);
    ss = split(string{1}(2:end-1), ',');
    dim1 = str2num(ss{1});
    dim2 = str2num(ss{2});
    
    target = [dim1, dim2];
    
     try
        [result] = calculate_intersection(['data/out_test_FINAL/', num2str(i), '/cam_0.png'], ['data/out_test_FINAL/', num2str(i), '/cam_1.png'], stereoParams, plane_properties);
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
