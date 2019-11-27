function [outputArg1,outputArg2] = my_fit_func(inputArg1)
    [N, ~] = size(inputArg1(:,1));
    % Find line of best fit (in least-squares sense) through X
    % -------------------------------------------------------------------------
    X_ave=mean(inputArg1,1);            % mean; line of best fit will pass through this point  
    dX=bsxfun(@minus,inputArg1,X_ave);  % residuals
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
end

