function cost=howmuchs(c1,c2,c3,c4,c5) 
% 这是判定一组最佳方向的阈值代价函数
% c1:当前阈值距离 c2:最大阈值距离 c3:这组备选方向的个数 c4:一组备选方向 c5:目标方向
omaga=10;
sum=0;
for i=1:c3
    sum=sum+abs(c4(i)-c5);
end 
cost=omaga*(c2-c1)+sum;