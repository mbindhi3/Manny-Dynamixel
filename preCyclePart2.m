addpath('Optragen/src');
addpath('Snopt/mex');
addpath('Snopt/matlab');
clear all;
global nlp;
intialXPosition = 0;
finalYPosition = 0;
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
a3InitialConstraint = constraint(0,'a3',0,'initial');
a4InitialConstraint = constraint(0,'a4',0,'initial');
a5InitialConstraint = constraint(0,'a5',0,'initial');
a14InitialConstraint = constraint(-0.9825,'a14',-0.9825,'initial');
a15InitialConstraint = constraint(1.5587,'a15',1.5587,'initial');
a16InitialConstraint = constraint(-.5762,'a16',-0.5762,'initial');
a3TrajectoryConstraint = constraint(-3*pi/4,'a3',0,'trajectory');
a4TrajectoryConstraint = constraint(0,'a4',pi/2,'trajectory');
a5TrajectoryConstraint = constraint(-pi/3,'a5',0,'trajectory');
a14TrajectoryConstraint = constraint(-3*pi/4,'a14',0,'trajectory');
a15TrajectoryConstraint = constraint(0,'a15',pi/2,'trajectory');
a16TrajectoryConstraint = constraint(-pi/3,'a16',0,'trajectory');
orientationFinalConstraint = constraint(0,'a14 + a15 + a16',0,'final');
xRightFootBasePositionFinalConstraint = constraint(2,'(17218^(1/2)*cos(a14 - a4 - a5 - a3 + a15 + a16 + atan(127/33)))/100 - (90274^(1/2)*cos(a5 - atan(295/57)))/100 + (3*9965^(1/2)*cos(a4 + a5 + atan(98/19)))/100 - (3*9965^(1/2)*cos(a3 + a4 + a5 - a14 + atan(98/19)))/100 + (90274^(1/2)*cos(a3 + a4 + a5 - a14 - a15 - atan(295/57)))/100 - 33/100',2,'final');
zRightFootBasePositionFinalConstraint = constraint(0,'(3*9965^(1/2)*cos(a4 + a5 - atan(19/98)))/100 - (3*9965^(1/2)*cos(a3 + a4 + a5 - a14 - atan(19/98)))/100 + (90274^(1/2)*cos(a5 + atan(57/295)))/100 - (17218^(1/2)*cos(a3 + a4 + a5 - a14 - a15 - a16 + atan(33/127)))/100 - (90274^(1/2)*cos(a3 + a4 + a5 - a14 - a15 + atan(57/295)))/100 + 127/100',0,'final');
xCOMTrajectoryConstraint = constraint(-1.2,'(3*89089168577^(1/2)*cos(a4 + a5 + atan(1462372/297821)))/350680 - (22348350232498^(1/2)*cos(a5 - atan(4642523/891813)))/1753400 - (483284377^(1/2)*cos(a14 - a4 - a5 - a3 + a15 + a16 + atan(19684/9789)))/1753400 + (4528831224905^(1/2)*cos(a3 + a4 + a5 + atan(2060011/534028)))/876700 - (27*32969569^(1/2)*cos(a3 + a4 + a5 - a14 + atan(5688/785)))/350680 + (123*19333106^(1/2)*cos(a3 + a4 + a5 - a14 - a15 - atan(4309/875)))/1753400 - 588411/1753400',0.6,'trajectory');
orientationTrajectoryConstraint = constraint( 0, 'a3+a4+a5', 0 ,'final');
initialConstraints = a3InitialConstraint + a4InitialConstraint + a5InitialConstraint + a14InitialConstraint + a15InitialConstraint + a16InitialConstraint;
trajectoryConstraints = a3TrajectoryConstraint + a4TrajectoryConstraint + a5TrajectoryConstraint + a14TrajectoryConstraint + a15TrajectoryConstraint + a16TrajectoryConstraint + orientationTrajectoryConstraint + xCOMTrajectoryConstraint;
finalConstraints = xRightFootBasePositionFinalConstraint + zRightFootBasePositionFinalConstraint + orientationFinalConstraint;
allConstraints = initialConstraints  + trajectoryConstraints + finalConstraints;
Cost = cost('a3d^2+a4d^2+a5d^2+a14d^2+a15d^2+a16d^2','trajectory');
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
a3val = linspace(-pi/2,pi/2,100);
a4val = linspace(0,pi,100);
a5val = linspace(-pi/4,pi/4,100);
a14val = linspace(-3*pi/4,pi,100);
a15val = linspace(-pi,0,100);
a16val = linspace(-pi/2,pi/3,100);
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
toc;
F(1)
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
[xCoM,yCoM,zCoM] = manny.CoM(1);
xCoM
pause(.05)
end
[xCoM,yCoM,zCoM] = manny.CoM(1)
[xRightFootBase,yRightFootBase,zRightFootBase] = footPositions(manny)
preCyclePart2Alphas = [a3;a4;a5;a14;a15;a16];
save('preCyclePart2Alphas','preCyclePart2Alphas')