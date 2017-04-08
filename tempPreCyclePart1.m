addpath('Optragen/src');
addpath('Snopt/mex');
addpath('Snopt/matlab');
clear all;
close all
global nlp;
xCoMEquation = '(3*2226736708433^(1/2)*cos(a4 + a5 + atan(1462372/296993)))/1753400 + (3*1263704114^(1/2)*cos(a3 + a4 + a5 - a14 - a15 - atan(34817/7175)))/350680 - (483284377^(1/2)*cos(a14 - a4 - a5 - a3 + a15 + a16 + atan(19684/9789)))/1753400 + (4528831224905^(1/2)*cos(a3 + a4 + a5 + atan(2060011/534028)))/876700 - (22420388002594^(1/2)*cos(a5 - atan(4650275/891813)))/1753400 - (81*91663321^(1/2)*cos(a3 + a4 + a5 - a14 + atan(9480/1339)))/1753400 - 9789/1753400';
xRightFootBaseEquation = '(127*sin(a3 + a4 + a5 - a14 - a15 - a16))/100 - (90274^(1/2)*cos(a5 - atan(295/57)))/100 + (3*9965^(1/2)*cos(a4 + a5 + atan(98/19)))/100 - (3*9965^(1/2)*cos(a3 + a4 + a5 - a14 + atan(98/19)))/100 + (90274^(1/2)*cos(a3 + a4 + a5 - a14 - a15 - atan(295/57)))/100';
zRightFootBaseEquation = '(3*9965^(1/2)*cos(a4 + a5 - atan(19/98)))/100 - (127*cos(a3 + a4 + a5 - a14 - a15 - a16))/100 - (3*9965^(1/2)*cos(a3 + a4 + a5 - a14 - atan(19/98)))/100 + (90274^(1/2)*cos(a5 + atan(57/295)))/100 - (90274^(1/2)*cos(a3 + a4 + a5 - a14 - a15 + atan(57/295)))/100 + 127/100'; 
ninterv = 2;
hl = 1.0;
a3 = traj(ninterv,2,5); 
a4 = traj(ninterv,2,5);
a5 = traj(ninterv,2,5);
a14 = traj(ninterv,2,5);
a15 = traj(ninterv,2,5);
a16 = traj(ninterv,2,5);
a3d = deriv(a3);
a4d = deriv(a4);
a5d = deriv(a5);
a14d = deriv(a14);
a15d = deriv(a15);
a16d = deriv(a16);
ParamList = {};
a14InitialConstraint = constraint(0,'a14',0,'initial');
a15InitialConstraint = constraint(0,'a15',0,'initial');
a16InitialConstraint = constraint(0,'a16',0,'initial');
a3InitialConstraint = constraint(0,'a3',0,'initial');
a4InitialConstraint = constraint(0,'a4',0,'initial');
a5InitialConstraint = constraint(0,'a5',0,'initial');
a3TrajectoryConstraint = constraint(-pi/4,'a3',pi/6,'trajectory');
a4TrajectoryConstraint = constraint(0,'a4',pi/3,'trajectory');
a5TrajectoryConstraint = constraint(-pi/4,'a5',0,'trajectory');
a14TrajectoryConstraint = constraint(-3*pi/4,'a14',0,'trajectory');
a15TrajectoryConstraint = constraint(0,'a15',pi/2,'trajectory');
a16TrajectoryConstraint = constraint(-pi/2,'a16',pi/3,'trajectory');
orientationTrajectoryConstraint = constraint(0,'a14 + a15+ a16', 0,'trajectory');
orientationFinalConstraint = constraint(0,'a14 + a15 + a16',0,'final');
orientationTrajectoryConstraintRight = constraint(0,'a3 + a4+ a5', 0,'trajectory');
xRightFootBasePositionFinalConstraint = constraint(2,xRightFootBaseEquation,2,'final');
xRightFootBasePositionTrajConstraint = constraint(0,xRightFootBaseEquation,2.5,'trajectory');
zRightFootBasePositionFinalConstraint = constraint(0,zRightFootBaseEquation,0,'final');
zRightFootBasePositionTrajConstraint = constraint(0,zRightFootBaseEquation,1.5,'trajectory');
xCoMTrajectoryConstraint = constraint(0.5,xCoMEquation,1.2,'trajectory');
initialConstraints = a14InitialConstraint + a15InitialConstraint + a16InitialConstraint + a3InitialConstraint + a4InitialConstraint + a5InitialConstraint;
trajectoryConstraints = xRightFootBasePositionTrajConstraint + orientationTrajectoryConstraintRight + orientationFinalConstraint + a3TrajectoryConstraint + a4TrajectoryConstraint + a5TrajectoryConstraint + a14TrajectoryConstraint + a15TrajectoryConstraint + a16TrajectoryConstraint + orientationTrajectoryConstraint + xCoMTrajectoryConstraint + zRightFootBasePositionTrajConstraint;
finalConstraints = xRightFootBasePositionFinalConstraint + zRightFootBasePositionFinalConstraint;
allConstraints = initialConstraints  + trajectoryConstraints + finalConstraints;
Cost = cost(['a3d^2+a4d^2+a5d^2+a14d^2+a15d^2+a16d^2+30*(6 -(' zRightFootBaseEquation '))'] ,'trajectory');
breaks = linspace(0,hl,ninterv+1);
gauss = [-1 1]*sqrt(1/3)/2;
temp = ((breaks(2:ninterv+1)+breaks(1:ninterv))/2);
temp = temp(ones(1,length(gauss)),:) + gauss'*diff(breaks);
colpnts = temp(:).';
HL = linspace(0,hl,20);
pathName = './';
probName = 'planarMotionLeg';
TrajList = trajList(a3,a3d,a4,a4d,a5,a5d,a14, a14d, a15, a15d, a16, a16d);
nlp = ocp2nlp(TrajList,Cost,allConstraints,HL,ParamList,pathName,probName);
snset('Minimize');
xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);
Time = linspace(0,1,100);
a3val = linspace(0,-pi/6,100);
a4val = linspace(0,pi/3,100);
a5val = linspace(0,-pi/6,100);
a14val = linspace(0,-pi/4,100);
a15val = linspace(0,pi/2,100);
a16val = linspace(pi/16,-pi/5,100);
a3sp = createGuess(a3,Time,a3val);
a4sp = createGuess(a4,Time,a4val);
a5sp = createGuess(a5,Time,a5val);
a14sp = createGuess(a14,Time,a14val);
a15sp = createGuess(a15,Time,a15val);
a16sp = createGuess(a16,Time,a16val);
init = [a3sp.coefs a4sp.coefs a5sp.coefs a14sp.coefs a15sp.coefs a16sp.coefs]';% + 0.001*rand(nlp.nIC,1);
tic;
if(1)
[x,F,inform] = snopt(init, xlow, xupp, [], [], ...
                     [0;nlp.LinCon.lb;nlp.nlb], [Inf;nlp.LinCon.ub;nlp.nub],...
                     [], [], 'ocp2nlp_cost_and_constraint');
toc;
F(1)
end
sp = getTrajSplines(nlp,x);
a1SP = sp{1};
a2SP = sp{2};
a3SP = sp{3};
a4SP = sp{4};
a5SP = sp{5};
a6SP = sp{6};
refinedTimeGrid = linspace(min(HL),max(HL),100);
a3 = fnval(a1SP,refinedTimeGrid);
a3d = fnval(fnder(a1SP),refinedTimeGrid);
a4 = fnval(a2SP,refinedTimeGrid);
a4d = fnval(fnder(a2SP),refinedTimeGrid);
a5 = fnval(a3SP,refinedTimeGrid);
a5d = fnval(fnder(a3SP),refinedTimeGrid);
a14 = fnval(a4SP,refinedTimeGrid);
a14d = fnval(fnder(a4SP),refinedTimeGrid);
a15 = fnval(a5SP,refinedTimeGrid);
a15d = fnval(fnder(a5SP),refinedTimeGrid);
a16 = fnval(a6SP,refinedTimeGrid);
a16d = fnval(fnder(a6SP),refinedTimeGrid);
delete('planarMotionLeg_fcf.m' , 'planarMotionLeg_icf.m', 'planarMotionLeg_nlfcf.m' , 'planarMotionLeg_nlgcf.m','planarMotionLeg_nlicf.m','planarMotionLeg_nltcf.m','planarMotionLeg_tcf.m')
t=1;
alphas = zeros(1,22);
manny = Manny(alphas);
for index = 1:length(a16)
alphas(3) = a3(index);
alphas(4) = a4(index);
alphas(5) = a5(index);
alphas(14) = a14(index);
alphas(15) = a15(index);
alphas(16) = a16(index);
manny = manny.setGTransforms(alphas);
visualization(manny,1)
pause(.05)
end
[xCoM,yCoM,zCoM] = manny.CoM(1)
[xRightFootBase,yRightFootBase,zRightFootBase] = footPositions(manny)
preCyclePart1Alphas = [a3;a4;a5;a14;a15;a16];
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