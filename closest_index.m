function [index] = closest_index(target, list)
[~, index] = min(sqrt(sum((target' - list') .^ 2)));
end