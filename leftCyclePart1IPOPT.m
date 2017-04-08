addpath('C:/Users/MALU/Documents/GitHub/control')
addpath('C:/Users/MALU/Documents/GitHub/control/optimal')
addpath('C:/Users/MALU/Documents/GitHub/Manny-master')
addpath('C:/Users/MALU/Documents/GitHub')
MATLIBS = {'control/snopt','control/snopt/matlab/matlab', ...
            'control/optimal/Optragen/src', 'control/optimal/Optragen/', ...
            'control/optimal/'};
ivalab.loadLibraries(MATLIBS);
addpath('autogen');
close all
clear all;
load('withRespectToRightFoot.mat')
xLeftFootBaseEquation = char(positionOfLeftFootBase(1));
yLeftFootBaseEquation = char(positionOfLeftFootBase(2));
zLeftFootBaseEquation = char(positionOfLeftFootBase(3));
xCoMwrtRightFootEquation = char(xCoMwrtRight);
yCoMwrtRightFootEquation = char(yCoMwrtRight);
noDraggingEquation = char(noDragging);
alphaSymbols = {'a1';'a3';'a4';'a5';'a6';'a12';'a14';'a15';'a16';'a17'};
global nlp;
ninterv = 2;
hl = 1.0;
a1 = traj('a1',ninterv,2,5); 
a3 = traj('a3',ninterv,2,5); 
a4 = traj('a4',ninterv,2,5);
a5 = traj('a5',ninterv,2,5);
a6 = traj('a6',ninterv,2,5); 
a12 = traj('a12',ninterv,2,5); 
a14 = traj('a14',ninterv,2,5); 
a15 = traj('a15',ninterv,2,5); 
a16 = traj('a16',ninterv,2,5); 
a17 = traj('a17',ninterv,2,5); 
a1d = a1.deriv('a1d');
a3d = a3.deriv('a3d');
a4d = a4.deriv('a4d');
a5d = a5.deriv('a5d');
a6d = a6.deriv('a6d');
a12d = a12.deriv('a12d');
a14d = a14.deriv('a14d');
a15d = a15.deriv('a15d');
a16d = a16.deriv('a16d');
a17d = a17.deriv('a17d');
ParamList = {};
a1InitialConstraint = constraint(.1612,'a1', .1612,'initial','a1');
a3InitialConstraint = constraint(-0.1197,'a3', -0.1197 ,'initial','a3');
a4InitialConstraint = constraint(0.6958,'a4', 0.6958,'initial','a4');
a5InitialConstraint = constraint( -0.5760 ,'a5', -0.5760 ,'initial','a5');
a6InitialConstraint = constraint(-0.1612,'a6',   -0.1612,'initial','a6');
a12InitialConstraint = constraint( 0.1640,'a12', 0.1640,'trajectory','a12');
a14InitialConstraint = constraint( -0.7369,'a14', -0.7369 + .0001,'trajectory','a14');
a15InitialConstraint = constraint(1.2310  ,'a15',1.2310 + .0001,'trajectory','a15');
a16InitialConstraint = constraint(-0.4941 ,'a16',-0.4941+ .0001,'trajectory','a16');
a17InitialConstraint = constraint(-0.1640,'a17', -0.1640+ .0001,'trajectory','a17');
initialConstraints = a1InitialConstraint + a3InitialConstraint + a4InitialConstraint + a5InitialConstraint + a6InitialConstraint + a12InitialConstraint + a14InitialConstraint + a15InitialConstraint + a16InitialConstraint + a17InitialConstraint;
rollOrientationLeftLegTrajectoryConstraint = constraint(-0.001,'a1 + a6', 0.001,'trajectory',alphaSymbols([1 5]));
pitchOrientationLeftLegTrajectoryConstraint = constraint(-0.001,'a3 + a4 + a5', 0.001,'trajectory',alphaSymbols(2:4));
rollOrientationRightLegTrajectoryConstraint = constraint(-0.001,'a12 + a17', 0.001,'trajectory',alphaSymbols([6 10]));
pitchOrientationRightLegTrajectoryConstraint = constraint(-0.001,'a16 + a14 + a15', 0.001,'trajectory',alphaSymbols(7:9));
yLeftFootBaseTrajectoryConstraint = constraint(4.00,yLeftFootBaseEquation,4.04,'trajectory',alphaSymbols);
yCoMTrajectoryConstraint = constraint(2.75,yCoMwrtRightFootEquation,3.7,'trajectory',alphaSymbols);
noDraggingConstraint = constraint( .5^2 + .001,noDraggingEquation,inf,'trajectory',alphaSymbols);
trajectoryConstraints = rollOrientationLeftLegTrajectoryConstraint + pitchOrientationLeftLegTrajectoryConstraint + yLeftFootBaseTrajectoryConstraint;
zLeftFootBaseFinalConstraint = constraint(1,zLeftFootBaseEquation,1,'final',alphaSymbols);
xLeftFootBaseFinalConstraint = constraint(0,xLeftFootBaseEquation,0,'final',alphaSymbols);
finalConstraints = zLeftFootBaseFinalConstraint + xLeftFootBaseFinalConstraint;
allConstraints = initialConstraints + finalConstraints + trajectoryConstraints;
Cost = cost('a1d^2+a3d^2+a4d^2+a5d^2+a6d^2+a12d^2+a14d^2+a15d^2+a16d^2+a17d^2','trajectory');
breaks = linspace(0,hl,ninterv+1);
gauss = [-1 1]*sqrt(1/3)/2;
temp = ((breaks(2:ninterv+1)+breaks(1:ninterv))/2);
temp = temp(ones(1,length(gauss)),:) + gauss'*diff(breaks);
colpnts = temp(:).';
HL = linspace(0,hl,20);
pathName = './';
probName = 'planarMotionLeg';
TrajList = traj.trajList(a1,a1d,a3,a3d,a4,a4d,a5,a5d,a6,a6d,a12,a12d,a14,a14d,a15,a15d,a16,a16d,a17,a17d);
nlp = ocp2nlp(TrajList,Cost,allConstraints,HL,ParamList,pathName,probName);
snset('Minimize');
xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);
Time = linspace(0,1,100);
a1val = 0.1612 * ones(1,100);
a3val = linspace( -0.1197,-0.9263,100);
a4val = linspace(-0.5760,1.7683,100);
a5val = linspace(-0.1612,-0.8420,100);
a6val = 0.1640 * ones(1,100);
a12val = -0.7369 * ones(1,100);
a14val = 1.2310 * ones(1,100);
a15val = 1.3728 * ones(1,100);
a16val = -0.4941 * ones(1,100);
a17val = -0.1640 * ones(1,100);
a1sp = createGuess(a1,Time,a1val);
a3sp = createGuess(a3,Time,a3val);
a4sp = createGuess(a4,Time,a4val);
a5sp = createGuess(a5,Time,a5val);
a6sp = createGuess(a6,Time,a6val);
a12sp = createGuess(a12,Time,a12val);
a14sp = createGuess(a14,Time,a14val);
a15sp = createGuess(a15,Time,a15val);
a16sp = createGuess(a16,Time,a16val);
a17sp = createGuess(a17,Time,a17val);
init = [a1sp.coefs a3sp.coefs a4sp.coefs a5sp.coefs a6sp.coefs a12sp.coefs a14sp.coefs a15sp.coefs a16sp.coefs a17sp.coefs];
tic;
ghSnopt = ipoptFunction(nlp);
toc;
[~,~,nobj, nlinConstr, nnlConstr]=ghSnopt(init');
nFreeVar=length(init);
nConstraint = nlinConstr+nnlConstr;
profile on;
tic;
[x, info]=optragen_ipopt(ghSnopt,nobj,nFreeVar,nConstraint,init,xlow,xupp,[nlp.LinCon.lb;nlp.nlb],[nlp.LinCon.ub;nlp.nub]);
toc;
sp = getTrajSplines(nlp,x);
a1SP = sp{1};
a3SP = sp{2};
a4SP = sp{3};
a5SP = sp{4};
a6SP = sp{5};
a12SP = sp{6};
a14SP = sp{7};
a15SP = sp{8};
a16SP = sp{9};
a17SP = sp{10};
refinedTimeGrid = linspace(min(HL),max(HL),100);
a1 = fnval(a1SP,refinedTimeGrid);
a3 = fnval(a3SP,refinedTimeGrid);
a4 = fnval(a4SP,refinedTimeGrid);
a5 = fnval(a5SP,refinedTimeGrid);
a6 = fnval(a6SP,refinedTimeGrid);
a12 = fnval(a12SP,refinedTimeGrid);
a14 = fnval(a14SP,refinedTimeGrid);
a15 = fnval(a15SP,refinedTimeGrid);
a16 = fnval(a16SP,refinedTimeGrid);
a17 = fnval(a17SP,refinedTimeGrid);
t=1;
manny = Manny(zeros(1,22));
xCOMS = zeros(1,100);
yCOMS = zeros(1,100);
zLeftFootBasePosition = zeros(1,100);
yLeftFootBasePosition = zeros(1,100);
xLeftFootBasePosition = zeros(1,100);
leftCyclePart1Alphas = [a1;a3;a4;a5;a6;a12;a14;a15;a16;a17];
save('leftCyclePart1Alphas','leftCyclePart1Alphas')
tempAlphas = zeros(1,22);
for i = 1:100
 tempAlphas(1) = a1(i);
 tempAlphas(3) = a3(i);
 tempAlphas(4) = a4(i);
 tempAlphas(5) = a5(i);
 tempAlphas(6) = a6(i);
 tempAlphas(12) = a12(i);
 tempAlphas(14) = a14(i);
 tempAlphas(15) = a15(i);
 tempAlphas(16) = a16(i);
 tempAlphas(17) = a17(i);
 manny = manny.setGTransforms(tempAlphas);
 [xCoM,yCoM] = manny.CoM(2);
 xCOMS(i) = xCoM;
 yCOMS(i) = yCoM;
 [~,leftFootBase] = footPositions(manny);
 zLeftFootBasePosition(i) = leftFootBase(3);
 yLeftFootBasePosition(i) = leftFootBase(2);
 xLeftFootBasePosition(i) = leftFootBase(1); 
end
figure
plot(xCOMS);
hold on
plot(yCOMS);
title('Center of Mass')
legend('Center of Mass:x','Center of Mass:y')
figure
plot(zLeftFootBasePosition,'m')
hold on
plot(xLeftFootBasePosition,'c')
hold on
plot(yLeftFootBasePosition,'b')
title('Foot Position Base')
legend('z right foot base position ','x right goot base position','y right foot base postion')
figure
plot(a1,'k')
hold on
plot(a3,'c')
hold on
plot(a4,'m')
hold on
plot(a5,'r')
hold on
plot(a6,'g')
legend('a1(hip-x)','a3(hip-y)','a4(knee-y)','a5(foot-y)','a6(foot-x)')
title('Left Foot Angles')
figure
plot(a12,'k')
hold on
plot(a14,'c')
hold on
plot(a15,'m')
hold on
plot(a16,'r')
hold on
plot(a17,'g')
legend('a12(hip-x)','a14(hip-y)','a15(knee-y)','a16(foot-y)','a17(foot-x)')
title('Right Foot Angles')
figure
alphas = zeros(1,22);
manny = Manny(alphas);
for index = 1:1:length(a3)
alphas(1) = a1(index);
alphas(3) = a3(index);
alphas(4) = a4(index);
alphas(5) = a5(index);
alphas(6) = a6(index);
alphas(12) = a12(index);
alphas(14) = a14(index);
alphas(15) = a15(index);
alphas(16) = a16(index);
alphas(17) = a17(index);
manny = manny.setGTransforms(alphas);
visualization(manny,2)
pause(.05)
end
figure
plot(xLeftFootBasePosition,zLeftFootBasePosition)
hold on
x = -1.4;
y = -.2;
r = .5;
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
axis equal