function [ valueij ] = Iij(d)
%UNTITLED3 活动单元的Iij值
%   使用了VFH*TDT的求模公式
rsafe=0.6; %主程序里是0.3 ？？
dmax=1.8;
    if (d<=rsafe)
        valueij=1;
    else if (rsafe<d<=dmax)
        valueij=(dmax^2-d^2)/(dmax^2-rsafe^2); 
    else   
        valueij=0;
    end
end

