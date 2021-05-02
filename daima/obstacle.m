%load ob ob
ob=imread('cave.png');
s=ob;
s=double(s);
[row,flow]=size(ob);
n=1;
ob=zeros(2000,2);

for i=1:row
    for j=1:flow
       if s(i,j)==0
            ob(n,1:2)=[j i];
            n=n+1; 
%        elseif i==1||i==500||j==1||j==500
%             obstacle(n,1:2)=[j i];
%             n=n+1;
        end
    end
end
           ob=ob/50;  %×ø±êËõÐ¡50±¶
            
            
      %      save obstacle obstacle
