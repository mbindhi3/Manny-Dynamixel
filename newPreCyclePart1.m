addpath('Optragen/src');
addpath('Snopt/mex');
addpath('Snopt/matlab');
clear all;
close all
global nlp;
xCoMEquation = '(3*2226736708433^(1/2)*cos(a15 + a16 + atan(1462372/296993)))/1753400 + (3*1263704114^(1/2)*cos(a3 + a4 - a14 - a15 - a16 + atan(34817/7175)))/350680 + (4528831224905^(1/2)*cos(a14 + a15 + a16 + atan(2060011/534028)))/876700 - (22420388002594^(1/2)*cos(a16 - atan(4650275/891813)))/1753400 - (81*91663321^(1/2)*cos(a14 - a3 + a15 + a16 + atan(9480/1339)))/1753400 - (2492477989^(1/2)*cos(a3 + a4 + a5 - a14 - a15 - a16 + atan(9842/48945)))/876700 - 9789/175340';
xLeftFootBaseEquation = '(3*9965^(1/2)*cos(a15 + a16 + atan(98/19)))/100 - (90274^(1/2)*cos(a16 - atan(295/57)))/100 - (127*sin(a3 + a4 + a5 - a14 - a15 - a16))/100 - (3*9965^(1/2)*cos(a14 - a3 + a15 + a16 + atan(98/19)))/100 + (90274^(1/2)*cos(a3 + a4 - a14 - a15 - a16 + atan(295/57)))/100';
zLeftFootBaseEquation = '(3*9965^(1/2)*cos(a15 + a16 - atan(19/98)))/100 - (127*cos(a3 + a4 + a5 - a14 - a15 - a16))/100 - (3*9965^(1/2)*cos(a14 - a3 + a15 + a16 - atan(19/98)))/100 + (90274^(1/2)*cos(a16 + atan(57/295)))/100 - (90274^(1/2)*cos(a14 - a4 - a3 + a15 + a16 + atan(57/295)))/100 + 127/100';
ninterv = 4;
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
a3InitialConstraint = constraint(0,'a3',0,'initial');
a4InitialConstraint = constraint(0,'a4',0,'initial');
a5InitialConstraint = constraint(0,'a5',0,'initial');
a14InitialConstraint = constraint(0,'a14',0,'trajectory');
a15InitialConstraint = constraint(0,'a15',0,'trajectory');
a16InitialConstraint = constraint(0,'a16',0,'trajectory');
a4TrajectoryConstraint = constraint(0,'a4',pi,'trajectory');
xLeftFootBasePositionFinalConstraint = constraint(0,xLeftFootBaseEquation,0,'final');
zLeftFootBasePositionFinalConstraint = constraint(1,zLeftFootBaseEquation,1,'final');
zLeftFootBasePositionTrajectoryConstraint = constraint(0,zLeftFootBaseEquation,1,'trajectory');
xLeftFootBasePositionTrajectoryConstraint = constraint(0,xLeftFootBaseEquation,1,'trajectory');
%xCoMTrajectoryConstraint = constraint(-0.2,xCoMEquation,1.2,'trajectory');
orientationTrajectoryConstraint = constraint(0,'a3 + a4 + a5',0,'trajectory');
initialConstraints = a3InitialConstraint + a4InitialConstraint + a5InitialConstraint + a14InitialConstraint + a15InitialConstraint + a16InitialConstraint;
trajectoryConstraints = xLeftFootBasePositionTrajectoryConstraint+ zLeftFootBasePositionTrajectoryConstraint +a4TrajectoryConstraint ;%+ xLeftFootBasePositionTrajectoryConstraint + zLeftFootBasePositionTrajectoryConstraint;
finalConstraints = xLeftFootBasePositionFinalConstraint + zLeftFootBasePositionFinalConstraint;
allConstraints = initialConstraints  + trajectoryConstraints + finalConstraints;
Cost = cost('a14d^2+a15d^2+a16d^2+a3d^2+a4d^2+a5d^2','trajectory');
breaks = linspace(0,hl,ninterv+1);
gauss = [-1 1]*sqrt(1/3)/2;
temp = ((breaks(2:ninterv+1)+breaks(1:ninterv))/2);
temp = temp(ones(1,length(gauss)),:) + gauss'*diff(breaks);
colpnts = temp(:).';
HL = linspace(0,hl,20);
pathName = './';
probName = 'planarMotionLeg';
TrajList = trajList(a3, a3d, a4, a4d, a5, a5d, a14, a14d, a15, a15d, a16, a16d);
nlp = ocp2nlp(TrajList,Cost,allConstraints,HL,ParamList,pathName,probName);
snset('Minimize');
xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);
Time = linspace(0,1,100);
a3val = linspace(0,-0.5790,100);
a4val = linspace(0,1.5587,100);
a5val = linspace(0,-0.9797,100);
a14val = 0*linspace(0,0,100);
a15val = 0*linspace(0,0,100);
a16val = 0*linspace(0,0,100);
a3sp = createGuess(a3,Time,a3val);
a4sp = createGuess(a4,Time,a4val);
a5sp = createGuess(a5,Time,a5val);
a14sp = createGuess(a14,Time,a14val);
a15sp = createGuess(a15,Time,a15val);
a16sp = createGuess(a16,Time,a16val);
init = [a3sp.coefs a4sp.coefs a5sp.coefs a14sp.coefs a15sp.coefs a16sp.coefs]';% + 0.001*rand(nlp.nIC,1);
tic;
[x,F,inform] = snopt(init, xlow, xupp, [], [], ...
                     [0;nlp.LinCon.lb;nlp.nlb], [Inf;nlp.LinCon.ub;nlp.nub],...
                     [], [], 'ocp2nlp_cost_and_constraint');
F(1)

toc;
%x =init;
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
preCyclePart1Alphas = [a3;a4;a5;a14;a15;a16];
save('preCyclePart1Alphas','preCyclePart1Alphas')
manny = Manny(zeros(1,22));
alphas = preCyclePart1Alphas;
xCOMS = zeros(1,100);
zLeftFootBasePosition = zeros(1,100);
xLeftFootBasePosition = zeros(1,100);
for i = 1:100
 tempAlphas = zeros(1,22);
 tempAlphas(3) = alphas(1,i);
 tempAlphas(4) = alphas(2,i);
 tempAlphas(5) = alphas(3,i);
 tempAlphas(14) = alphas(4,i);
 tempAlphas(15) = alphas(5,i);
 tempAlphas(16) = alphas(6,i);
 manny = manny.setGTransforms(tempAlphas);
 xCOMS(i) = manny.CoM(2);
[RightFootBase, LeftFootBase, LeftFootFront, LeftFootBack, RightFootFront, RightFootBack] = footPositions(manny);
 zLeftFootBasePosition(i) = LeftFootBase(3);
 xLeftFootBasePosition(i) = LeftFootBase(1);
end
figure(1)
plot(xCOMS);
title('Center of Mass: X')
figure(2)
plot(zLeftFootBasePosition,'m')
hold on
plot(xLeftFootBasePosition,'c')
title('Foot Position Base')
legend('Z  base position ','x  base position')
hold off
figure(3)
plot(alphas(1,:),'r')
hold on
plot(alphas(2,:),'g')
plot(alphas(3,:),'b')
legend('a3(hip)','a4(knee)','a5(foot)')
title('Left Leg Y Plane Angles')
hold off
figure(4)
hold on
alphas = zeros(1,22);
manny = Manny(alphas);
for index = 1:5:length(a16)
alphas(3) = a3(index);
alphas(4) = a4(index);
alphas(5) = a5(index);
alphas(14) = a14(index);
alphas(15) = a15(index);
alphas(16) = a16(index);
manny = manny.setGTransforms(alphas);
visualization(manny,2)
pause(.05)
end
hold off