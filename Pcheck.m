function flag = Pcheck(Q,A,B,C,D)

QX = Q(1);
QY = Q(2);

AX = A(1);
AY = A(2);

BX = B(1);
BY = B(2);

CX = C(1);
CY = C(2);

DX = D(1);
DY = D(2);

AQ = [QX-AX, QY-AY];
AB = [BX-AX, BY-AY];
AD = [DX-AX, DY-AY];

if (AQ*(AB') > 0.0000000001) && (AQ*(AB') < AB*(AB') - 0.0000000001) && (AQ*(AD') < AD*(AD') - 0.0000000001) && (AQ*(AD') > 0.0000000001)
    flag = 1;
else
    flag = 0;
end