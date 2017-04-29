% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu =  [149.7198  144.5757   60.8763];
sig = [180.8987  128.4632  339.5755];
thre = 0.0000001;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
[nrows, ncols, dim] = size(I);
mask = zeros(nrows,ncols);
for i = 1:nrows
    for j = 1:ncols
        dens = mvnpdf(double([I(i,j,1) I(i,j,2) I(i,j,3)]), mu, sig);
        if dens > thre
            mask(i,j) = 1;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
segI = false(nrows,ncols);
CC = bwconncomp(mask);
numPixels = cellfun(@numel,CC.PixelIdxList);
[b,idx] = max(numPixels);
segI(CC.PixelIdxList{idx}) = true; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;


% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
