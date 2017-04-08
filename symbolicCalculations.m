syms  a1 a3 a4 a5 a6 a12 a14 a15 a16 a17 
alphasSymbolic = [a1 0 a3 a4 a5 a6 0 0 0 0 0  a12 0 a14 a15 a16 a17 0 0 0 0 0];
% alphasSymbolic = [0 0 -0.0637 0.7513 -0.6876 0 0 0 0 0 0  0 0 -0.5853 1.1022 -0.5169 0 0 0 0 0 0];
MannySymbolic = Manny(alphasSymbolic);
[xCoMwrtRight,yCoMwrtRight] = MannySymbolic.CoM(2);
[~,positionOfLeftFootBase] = MannySymbolic.footPositions;
noDragging1 = (positionOfLeftFootBase(1) + 1.4)^2 + (positionOfLeftFootBase(3) + .2)^2;