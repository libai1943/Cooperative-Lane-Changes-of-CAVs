function [nA1,nA2,nA3] = random_division(NCNC)
% Randomly divide NCNC into 3 numbers (each number is larger or equal to 1)
nA1 = 0;
nA2 = 0;
nA3 = 0;

while ((nA3+nA2+nA1)~= NCNC)
    nA1 = round(rand*(NCNC-1)) + 1;
    nA2 = round(rand*(NCNC-1)) + 1;
    nA3 = round(rand*(NCNC-1)) + 1;
end