% 图像2值化处理 并生成障碍物坐标
a=imread('cave3.png');
b=graythresh(a);
c=im2bw(a,b);
%c=imbinarize(a,b);
c=double(c);
d=imshow(c);
saveas(d,'cave3_1','png'); %存图片 2值化格式
%求障碍物坐标坐标 并保存
[row,flow]=size(c);
n=1;
ob=zeros(2000,2);

for i=1:row
    for j=1:flow
       if c(i,j)==0
            ob(n,1:2)=[j i];
            n=n+1; 
%        elseif i==1||i==500||j==1||j==500
%             obstacle(n,1:2)=[j i];
%             n=n+1;
        end
    end
end
           ob=ob/50;
save obstacle1 ob;