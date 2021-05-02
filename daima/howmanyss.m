function dif=howmanyss(c1,c2)
% 如果出现VFH+算法得出了多于1个最佳方向的时候 所用的取舍函数
n = 72; % 扇区数目
dif = min([abs(c1-c2), abs(c1-c2-n), abs(c1-c2+n)]);
%dif=abs(c1-c2);