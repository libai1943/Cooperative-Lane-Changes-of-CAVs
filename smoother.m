function xx = smoother(x,number_of_frame,tf,NE)
[~,n] = size(x);
number_of_frame = round(number_of_frame./NE)*NE;
xx = zeros(number_of_frame+1,n);
for ii = 1:n
    basic = x(:,ii);
    xx(:,ii) = print_lagrange(basic,number_of_frame,tf,NE);
end