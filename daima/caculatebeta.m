%计算角度
%得到的角度(弧度制) 都是360°以内的正角度
function beta = caculatebeta(s,e)
dy = e(2) - s(2);
dx = e(1) - s(1);
if dx==0
    beta=pi/2;
else
    beta = atan(dy/dx); 
    if(dx < 0)
        if(dy > 0)
            beta = pi - abs(beta);
        else
            beta = pi + abs(beta);
        end
    else
        if(dy < 0)
            beta = 2*pi- abs(beta);
        end
    end
end


