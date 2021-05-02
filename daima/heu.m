function cost=heu(a,b,c,d)
% 这是VFH*算法投影节点的启发值函数
% a:目标方向 b:当前运动方向 c:上次选择方向 d:投影深度
n = 72; % 扇区数目
lmd=0.8;% 减少系数
u2=1;% 系数，与当前运动方向的差距
u3=1;% 系数，与上一次选择的方向的差距
dirt2 = min([abs(a-b), abs(a-b-n), abs(a-b+n)]);
dirt3 = min([abs(a-c), abs(a-c-n), abs(a-c+n)]);
cost = lmd^d*(u2*dirt2+u3*dirt3);