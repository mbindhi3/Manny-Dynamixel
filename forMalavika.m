addpath('/Users/siddharthmehta/Documents/Georgia Tech/Research/Manny-master')
close all
manny = Manny(zeros(1,22));
xCOMS = zeros(1,100);
yCOMS = zeros(1,100);
zLeftFootBasePosition = zeros(1,100);
yLeftFootBasePosition = zeros(1,100);
xLeftFootBasePosition = zeros(1,100);
load('leftCyclePart1Alphas.mat')
tempAlphas = zeros(1,22);
a1 = leftCyclePart1Alphas(1,:);
a3 = leftCyclePart1Alphas(2,:);
a4 = leftCyclePart1Alphas(3,:);
a5 = leftCyclePart1Alphas(4,:);
a6 = leftCyclePart1Alphas(5,:);
a12 = leftCyclePart1Alphas(6,:);
a14 = leftCyclePart1Alphas(7,:);
a15 = leftCyclePart1Alphas(8,:);
a16 = leftCyclePart1Alphas(9,:);
a17 = leftCyclePart1Alphas(10,:);
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