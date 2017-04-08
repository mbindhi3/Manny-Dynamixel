load('preCyclePart1Alphas.mat')
load('preCyclePart2Alphas.mat')
load('preCyclePart3Alphas.mat')
load('leftCyclePart1Alphas.mat')
load('leftCyclePart2Alphas.mat')
load('leftCyclePart3Alphas.mat')
%preCycleAlphas = [preCyclePart1Alphas , preCyclePart2Alphas, preCyclePart3Alphas];
leftCycleAlphas = [leftCyclePart1Alphas, leftCyclePart2Alphas, leftCyclePart3Alphas];
rightCycleAlphas = circshift(leftCycleAlphas,5);
rightCycleAphas(1,:) = -1 * 
% alphas = [leftCycleAlphas,rightCycleAlphas];
% save('alphas.mat','alphas')