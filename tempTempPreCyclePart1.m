addpath('Optragen/src');
addpath('Snopt/mex');
addpath('Snopt/matlab');
clear all;
close all
global nlp;
xCoMEquation = '(3*1263704114^(1/2)*cos(a14 + a15 + atan(34817/7175)))/350680 - (81*91663321^(1/2)*cos(a14 - atan(9480/1339)))/1753400 - (483284377^(1/2)*cos(a14 + a15 + a16 + atan(19684/9789)))/1753400 + 1057433/1753400';
xRightFootBaseEquation = '(90274^(1/2)*cos(a14 + a15 + atan(295/57)))/100 - (3*9965^(1/2)*cos(a14 - atan(98/19)))/100 - (127*sin(a14 + a15 + a16))/100';
zRightFootBaseEquation = '(179/25 - (90274^(1/2)*cos(a14 + a15 - atan(57/295)))/100 - (3*9965^(1/2)*cos(a14 + atan(19/98)))/100 - (127*cos(a14 + a15 + a16))/100)';
zRightFootFrontEquation = '179/25 - (52610^(1/2)*cos(a14 + a15 + a16 + atan(191/127)))/100 - (3*9965^(1/2)*cos(a14 + atan(19/98)))/100 - (90274^(1/2)*cos(a14 + a15 - atan(57/295)))/100';
zRightFootBackEquation = '179/25 - (52610^(1/2)*cos(a14 + a15 + a16 + atan(191/127)))/100 - (3*9965^(1/2)*cos(a14 + atan(19/98)))/100 - (90274^(1/2)*cos(a14 + a15 - atan(57/295)))/100';
ninterv = 4;
hl = 1.0;
a14 = traj(ninterv,2,5);
a15 = traj(ninterv,2,5);
a16 = traj(ninterv,2,5);
a14d = deriv(a14);
a15d = deriv(a15);
a16d = deriv(a16);
ParamList = {};
a14InitialConstraint = constraint(0,'a14',0,'initial');
a15InitialConstraint = constraint(0,'a15',0,'initial');
a16InitialConstraint = constraint(0,'a16',0,'initial');
a15TrajectoryConstraint = constraint(0,'a15',pi,'trajectory');
xRightFootBasePositionFinalConstraint = constraint(1,xRightFootBaseEquation,1,'final');
zRightFootBasePositionFinalConstraint = constraint(1,zRightFootBaseEquation,1,'final');
zRightFootBasePositionTrajectoryConstraint = constraint(0,zRightFootBaseEquation,1,'trajectory');
xRightFootBasePositionTrajectoryConstraint = constraint(0,xRightFootBaseEquation,1,'trajectory');
zRightFootFrontPositionTrajectoryConstraint = constraint(0,zRightFootFrontEquation,inf,'trajectory');
zRightFootBackPositionTrajectoryConstraint = constraint(0,zRightFootBackEquation,inf,'trajectory');
orientationTrajectoryConstraint = constraint(0,'a14 + a15 + a16',0,'trajectory');
initialConstraints = a14InitialConstraint + a15InitialConstraint + a16InitialConstraint;
trajectoryConstraints =  xRightFootBasePositionTrajectoryConstraint + zRightFootBasePositionTrajectoryConstraint;
finalConstraints = xRightFootBasePositionFinalConstraint + zRightFootBasePositionFinalConstraint;
allConstraints = initialConstraints  + trajectoryConstraints + finalConstraints;
Cost = cost('a14d^2+a15d^2+a16d^2','trajectory');
breaks = linspace(0,hl,ninterv+1);
gauss = [-1 1]*sqrt(1/3)/2;
temp = ((breaks(2:ninterv+1)+breaks(1:ninterv))/2);
temp = temp(ones(1,length(gauss)),:) + gauss'*diff(breaks);
colpnts = temp(:).';
HL = linspace(0,hl,20);
pathName = './';
probName = 'planarMotionLeg';
TrajList = trajList(a14, a14d, a15, a15d, a16, a16d);
nlp = ocp2nlp(TrajList,Cost,allConstraints,HL,ParamList,pathName,probName);
snset('Minimize');
xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);
Time = linspace(0,1,100);
a14val = linspace(0,-.9825,100);
a15val = [zeros(1,50) linspace(0,1.559,50)];
a16val = -a14val - a15val;
a14sp = createGuess(a14,Time,a14val);
a15sp = createGuess(a15,Time,a15val);
a16sp = createGuess(a16,Time,a16val);
init = [a14sp.coefs a15sp.coefs a16sp.coefs]';% + 0.001*rand(nlp.nIC,1);
tic;
if(0)
[x,F,inform] = snopt(init, xlow, xupp, [], [], ...
                     [0;nlp.LinCon.lb;nlp.nlb], [Inf;nlp.LinCon.ub;nlp.nub],...
                     [], [], 'ocp2nlp_cost_and_constraint');
F(1)
end
toc;
x =init;
sp = getTrajSplines(nlp,x);
a14SP = sp{1};
a15SP = sp{2};
a16SP = sp{3};
refinedTimeGrid = linspace(min(HL),max(HL),100);
a14 = fnval(a14SP,refinedTimeGrid);
a14d = fnval(fnder(a14SP),refinedTimeGrid);
a15 = fnval(a15SP,refinedTimeGrid);
a15d = fnval(fnder(a15SP),refinedTimeGrid);
a16 = fnval(a16SP,refinedTimeGrid);
a16d = fnval(fnder(a16SP),refinedTimeGrid);
delete('planarMotionLeg_fcf.m' , 'planarMotionLeg_icf.m', 'planarMotionLeg_nlfcf.m' , 'planarMotionLeg_nlgcf.m','planarMotionLeg_nlicf.m','planarMotionLeg_nltcf.m','planarMotionLeg_tcf.m')
t=1;
preCyclePart1Alphas = [a14;a15;a16];
save('preCyclePart1Alphas','preCyclePart1Alphas')
manny = Manny(zeros(1,22));
alphas = preCyclePart1Alphas;
xCOMS = zeros(1,100);
zRightFootBasePosition = zeros(1,100);
xRightFootBasePosition = zeros(1,100);
zRightFootFrontPosition = zeros(1,100);
zRightFootBackPosition = zeros(1,100);
for i = 1:100
 tempAlphas = zeros(1,22);
 tempAlphas(14) = alphas(1,i);
 tempAlphas(15) = alphas(2,i);
 tempAlphas(16) = alphas(3,i);
 manny = manny.setGTransforms(tempAlphas);
 xCOMS(i) = manny.CoM(1);
[RightFootBase, fLeftFootBase, LeftFootFront, LeftFootBack, RightFootFront, RightFootBack] = footPositions(manny);
 zRightFootBasePosition(i) = RightFootBase(3);
 xRightFootBasePosition(i) = RightFootBase(1);
 zRightFootFrontPosition(i) = RightFootFront(3);
 zRightFootBackPosition(i) = RightFootBack(3);
end
figure(1)
plot(xCOMS);
title('Center of Mass: X')
figure(2)
plot(zRightFootBasePosition,'m')
hold on
plot(xRightFootBasePosition,'c')
%plot(zRightFootBackPosition,'b')
%plot(zRightFootFrontPosition,'r')
title('Foot Position Base')
legend('Z  base position ','x  base position','z back position','z front position')
hold off
figure(3)
plot(alphas(1,:),'r')
hold on
plot(alphas(2,:),'g')
plot(alphas(3,:),'b')
legend('a14(hip)','a15(knee)','a16(foot)')
title('Right Leg Y Plane Angles')
hold off
figure(4)
hold on
alphas = zeros(1,22);
manny = Manny(alphas);
for index = 1:5:length(a16)
alphas(14) = a14(index);
alphas(15) = a15(index);
alphas(16) = a16(index);
manny = manny.setGTransforms(alphas);
visualization(manny,1)
pause(.05)
end
hold off