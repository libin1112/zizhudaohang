function cost=howmuch(c1,c2,c3,c4)
% 这是计算阈值和其最优方向的综合值的函数
omaga=10;
dirt=abs(c2-c3);
cost=omaga*(c4-c1)+dirt;