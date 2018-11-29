% ==============================================================================
% MATLAB Source Codes for "Balancing Computation Speed and Quality: A Decentralized
% Motion Planning Method for Cooperative Lane Changes of Connected and Automated Vehicles".

% ==============================================================================

%   Copyright (C) 2018 Bai Li
%   Useers must cite the following article when utilizing these source codes to produce new contributions.
%   Bai Li et al., "Balancing Computation Speed and Quality: A Decentralized Motion Planning Method for Cooperative
%   Lane Changes of Connected and Automated Vehicles",  IEEE TRANSACTIONS ON INTELLIGENT VEHICLES, accepted.

% ==============================================================================

% If there are inquiries, feel free to contact libai@zju.edu.cn or directly,
% libaioutstanding@163.com

% 2018.02.06
% ==============================================================================
close all

car_n = 0.96; % NEVER try to change this setting.
car_l = 2.8; % NEVER try to change this setting.
car_m = 0.929; % NEVER try to change this setting.
car_b = 1.942/2; % NEVER try to change this setting.

Gross_CPU_time = 0; % records the CPU time
tf1 = 0; % records the formation reconfiguration time at Stage 1
tf2 = 0; % records the simultaneous lane changes time at Stage 2

ds = 0.5; % ¡÷s >0 defined as the sampling gap in the paper
NCNC = 20; % Number of CAVs - Feel free to set NCNC in the range of [3,200]

global gap_max
gap_max = 5; % This variable is defined as the maximum (possible) gap between each adjacent vehicles in the same lane

% NOTE that this code allows only 3 lanes, but you will find that the proposed MVMP can handle
% any finite lanes when you read the paper.
% [a,b,c] = random_division(NCNC) is the function that divide NCNC as three
% numbers randomly such that NCNC = a + b + c, and a,b,c >= 1;

[nA1,nA2,nA3] = random_division(NCNC); % nA1,nA2,nA3 define the number of CAVs in each lane at the terminal moment
% nA1=6;nA2=7;nA3=7;
[oA1,oA2,oA3] = random_division(NCNC); % oA1,oA2,oA3 define the number of CAVs in each lane at the initial moment
% It would be better if the users can specify [nA1,nA2,nA3], e.g., you may
% set [nA1,nA2,nA3] = [6,7,7] when NC = 20; You can use:
% %  nA1=6;nA2=7;nA3=7; to replace [nA1,nA2,nA3] = random_division(NCNC);

delete('OLD1');
fid = fopen('OLD1', 'w');
fprintf(fid,'1  %g ', oA1);
fclose(fid);
delete('OLD2');
fid = fopen('OLD2', 'w');
fprintf(fid,'1  %g ', oA2);
fclose(fid);
delete('OLD3');
fid = fopen('OLD3', 'w');
fprintf(fid,'1  %g ', oA3);
fclose(fid);

% PXPY generates a file named PXPY which records the initial (x,y) location
% of each CAV. Vehicles 1,2,...,nA1 are naturally set to be in lane 1,
% vehicles (nA1+1),...,(nA1+nA2) are in lane 2, and so on.
PXPY_generater(oA1,oA2,oA3);

% A1,A2,A3 are index sets which record the terminal lanes of all the
% vehicles. For example, if A1 = {3,12}, then vehicles 3 and 12 should
% reach lane 1 at the TERMINAL moment. Note that A1,A2,A3 are randomly
% determined in this code, the users can have their own ways to set these
% three index sets.
temp = randperm(NCNC);
A1 = temp(1,1:nA1);
A2 = temp(1,(nA1+1):(nA1+nA2));
A3 = temp(1,(nA1+nA2+1):(nA1+nA2+nA3));

delete('A1');
fid = fopen('A1', 'w');
for ii = 1:length(A1)
    fprintf(fid,'%g ', A1(ii));
end
fclose(fid);
delete('A2');
fid = fopen('A2', 'w');
for ii = 1:length(A2)
    fprintf(fid,'%g ', A2(ii));
end
fclose(fid);
delete('A3');
fid = fopen('A3', 'w');
for ii = 1:length(A3)
    fprintf(fid,'%g ', A3(ii));
end
fclose(fid);


% Computation of the simultaneous lane changes MVMP subproblem begins
tic
NLP_SOLVING(1);

Gross_CPU_time = Gross_CPU_time + toc;
% Computation ends
load tf.txt
tf2 = tf;
load NE.txt

% Execution of Algorithm 1 begins
tic

load x.txt
load y.txt
load t.txt

x = reshape(x,length(x)./NCNC,NCNC);
y = reshape(y,length(y)./NCNC,NCNC);
theta = reshape(t,length(t)./NCNC,NCNC);

old_x = x(1,:);
temp = sort(x(1,1:NCNC));
for ii = 1:NCNC
    list(1,ii) = find(x(1,1:NCNC) == temp(1,(NCNC+1-ii)));
end

ds1 = 0;
ds2 = 0;
ds3 = 0;

for ix = 2 : NCNC
    ii = list(1,ix);
    flag = 1;
    for checker = 1 : (ix-1)
        if (x(1,list(1,ix)) > x(1,list(1,checker)))
            ddd = x(1,list(1,ix)) - x(1,list(1,ix-1));
            x(:,ii) = x(:,ii) - ddd - car_n - car_l - car_m;
        end
    end
    while (flag == 1)
        x(:,ii) = x(:,ii) - ds;
        tt = 1;
        flag = 0;
        while (tt <= NE*4)
            for iy = 1 : (ix-1)
                jj = list(1,iy); 
                [AA1,BB1,CC1,DD1] = ABCDgeneratoer(x(tt,ii), y(tt,ii), theta(tt,ii));
                [AA2,BB2,CC2,DD2] = ABCDgeneratoer(x(tt,jj), y(tt,jj), theta(tt,jj));
                if (RECcheck(AA1,BB1,CC1,DD1,AA2,BB2,CC2,DD2) == 1)
                    tt = NE*4;
                    flag = 1;
                end
            end
            tt = tt + 1;
        end
    end
end
new_x = x(1,:);
delete('sequenceddd');
fid = fopen('sequenceddd', 'w');
for ii = 1 : NCNC
    fprintf(fid,'%g   %g    \r\n', ii, (new_x(1,1) - new_x(1,ii)));
end
fclose(fid);
% Execution of Algorithm 1 ends, the output pf Algorithm 1 is the sequenceddd file, which
% is {d_i,k} in the paper.
Gross_CPU_time = Gross_CPU_time + toc;
new_x = x(1,:);

% Record the related state profiles for video generation later.
x_stage2 = x;
y_stage2 = y;
theta_stage2 = theta;
time_stage2 = tf2;
NE_stage2 = NE;

% Computation of the formation reconfiguration MVMP subproblem begins
tic
NLP_SOLVING(2);
load tf.txt
tf1 = tf;

load x.txt
load y.txt
load t.txt
load NE.txt
x = reshape(x,length(x)./NCNC,NCNC);
y = reshape(y,length(y)./NCNC,NCNC);
theta = reshape(t,length(t)./NCNC,NCNC);

ind = list(1);
dxdx = x(NE*4,ind) - x(1,ind);
x = x - dxdx;
% Computation ends
Gross_CPU_time = Gross_CPU_time + toc;

% Record the related state profiles for video generation later.
x_stage1 = x;
y_stage1 = y;
theta_stage1 = theta;
time_stage1 = tf1;
NE_stage1 = NE;

% Report the CPU time and the lane change completion time

fprintf('\r\nThe gross CPU time is %g sec.\r\n',Gross_CPU_time);
fprintf('The gross lane change completion time is %g sec.\r\n',tf1 + tf2);

% Video generation process begins.
black_box_function;
% <END> The generated simulation file has been recorded as "SIM.avi".