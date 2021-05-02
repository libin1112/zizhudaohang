function dif=dif(c1,c2)
% 如果取舍函数的代价还是一样，就用这个函数
%c1 方向 c2 目标方向
n = 72; % 扇区数目
%dif = min([abs(c1-c2), abs(c1-c2-n), abs(c1-c2+n)]);
dif=abs(c1-c2);