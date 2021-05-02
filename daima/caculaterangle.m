%º∆À„Ω«∂»

function rangle = caculaterangle(s,e)
dy = e(2) - s(2);
dx = e(1) - s(1);
if dx==0
    rangle=pi/2;
else
    rangle = asin(dy/dx); 
    if(dx < 0)
        if(dy > 0)
            rangle = pi - abs(rangle);
        else
            rangle = pi + abs(rangle);
        end
    else
        if(dy < 0)
           rangle = 2*pi- abs(rangle);
        end
    end
end
