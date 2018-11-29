function flag = RECcheck(A1,B1,C1,D1,A2,B2,C2,D2)

f1 = Pcheck(A2,A1,B1,C1,D1);
f2 = Pcheck(B2,A1,B1,C1,D1);
f3 = Pcheck(C2,A1,B1,C1,D1);
f4 = Pcheck(D2,A1,B1,C1,D1);

f5 = Pcheck(A1,A2,B2,C2,D2);
f6 = Pcheck(B1,A2,B2,C2,D2);
f7 = Pcheck(C1,A2,B2,C2,D2);
f8 = Pcheck(D1,A2,B2,C2,D2);


if (max([f1,f2,f3,f4,f5,f6,f7,f8]) == 1)
    flag = 1;
else
    flag = 0;
end