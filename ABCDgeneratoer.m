function [APP,BPP,CPP,DPP] = ABCDgeneratoer(x,y,theta)

R = 2.5376;

        x00 = x;
        y00 = y;
        t00 = theta;
        APP = [x00 + R*cos(t00+0.3927), y00 + R*sin(t00+0.3927)];
        CPP = [x00 - R*cos(t00+0.3927), y00 - R*sin(t00+0.3927)];
        BPP = [x00 + R*cos(-t00+0.3927), y00 - R*sin(-t00+0.3927)];
        DPP = [x00 - R*cos(-t00+0.3927), y00 + R*sin(-t00+0.3927)];