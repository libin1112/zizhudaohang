function cost=howmanys(c1,c2,c3,c4,a,b,c,d)
% 这是VFH*算法投影节点的方向判定代价函数
% c1:备选方向 c2:目标方向 c3:当前运动方向 c4:上一次选择方向 a:备选方向的极障碍物密度 b:备选左密度 c:备选右密度 d:投影深度
n = 72; % 扇区数目
lmd=0.8;% 减少系数
u1=5;% 系数，与目标方向的差距
u2=1;% 系数，与当前运动方向的差距
u3=1;% 系数，与上一次选择的方向的差距
u4=1;% 系数，改进代价函数，备选扇区及其左右扇区的极障碍物密度
dirt1 = min([abs(c1-c2), abs(c1-c2-n), abs(c1-c2+n)]);
dirt2 = min([abs(c1-c3), abs(c1-c3-n), abs(c1-c3+n)]);
dirt3 = min([abs(c1-c4), abs(c1-c4-n), abs(c1-c4+n)]);
if b~=inf
   if c~=inf 
   cost = lmd^d*(u1*dirt1+u2*dirt2+u3*dirt3+u4*[a+b+c]/3*(dirt1+dirt2+dirt3));
   else
   cost = lmd^d*(u1*dirt1+u2*dirt2+u3*dirt3+u4*[a+b]/2*(dirt1+dirt2+dirt3));    
   end
else
   cost = lmd^d*(u1*dirt1+u2*dirt2+u3*dirt3+u4*[a+c]/2*(dirt1+dirt2+dirt3)); 
end