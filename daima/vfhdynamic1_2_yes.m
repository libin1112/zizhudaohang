%动态环境（动态障碍物）VFH路径规划尝试
%构建动态障碍物：一个圆
load obstacle 'obstacle';
plot(obstacle(:,1),obstacle(:,2),'.k');
hold on
x0=1; y0=2.5;  %圆心起始坐标
r=0.15; %圆的半径
time=0.1; %时间间隔

v_car=0.5; %小车速度大小
load startpoint 'startpoint'; %载入起始点
load endpoint 'endpoint';   %载入起始点
plot(startpoint(1),startpoint(2),'.b');
hold on
plot(endpoint(1),endpoint(2),'.r')
hold on
title('VFH动态路径规划');
%参数设置 
%基本参数
f=5;                                      %角分辨率,单位：度
dmax = 1;                             %视野 活动窗格半径 最远距离  dmax也可以取很多：1.8  1.7  1.9  2.0  1.0  0.5  0.45  0.35 0.15
smax = 18;                                %大于18为宽波谷
C=15;                                     %cv值，原始值15
alpha = deg2rad(f);                       %分辨率由角度转为弧度 单位：弧度
n=360/f;                                  %分为72个扇区, n=72
rsafe=0.3;                                %扩大半径和安全距离0.6 障碍物点膨化处理 0.3
% 动态阈值的参数
D=0.125;                                  %刹车距离
lmda=1.5;                                 %安全放大系数
Dthmax=0.5;                                 %最大阈值距离 可取3 2 0.5
Dthmin=round(lmda*(D+rsafe));             %最小阈值距离
thresholdlow=C^2*Iij(Dthmin);             %最小阈值
dirtD=0.05;                                %阈值遍历步长 可以取0.2 0.1 0.05
blcs=round(Dthmax/dirtD);                 %阈值遍历次数
% 初始参数
robot=startpoint;                         %机器人位于起始点位置
ref=robot;                                %参考位置：机器人一开始还没走
radius=0.1;                              %机器人最小转向半径 可以取0.35  0.15 0.05 0.1
%VFH*算法的参量
%搜索树深度设为 ng=2
%ds=step;  %投影步长 
step=0.29 %以前是步长 现在是误差 步长已经被速度×时间代替了

kt=round(caculatebeta(robot,endpoint)/alpha);    %先定义目标方向
if(kt==0)
    kt=n;
end

for i=0:time:13.5
    
    if i<=4 %第一阶段（时间段）
        v=1.2; phiv=(5/18)*pi; %速度大小及方向
        %画出圆心起始位置
        plot(x0,y0,'.b');
        hold on
        x0=x0+v*cos(phiv)*time;  %圆心运动方程   %圆的方程
        y0=y0+v*sin(phiv)*time;  %圆心运动方程   方程
        %画出圆心实时位置
        scatter(x0,y0,'.r');
        drawnow;
        %plot(x0,y0,'r');
        phi=2*pi/3600:2*pi/3600:2*pi; %圆上的取点间隔
        x=x0+r*cos(phi);y=y0+r*sin(phi); %圆的方程
        xy=[];
        xy=[xy;x;y];
        xy=xy'; %转置后 得到圆上各点的坐标 
        plot(x,y,'.g');
        hold on
        mid=xy;
        obstacle=cat(1,obstacle,mid);  %动态障碍物加入静态障碍物坐标 cat为矩阵拼接函数 1竖着拼 2横着拼
        
        %%% 现在开始避障路径规划
        if(norm(robot-endpoint))>step          % 机器人位置和终点位置差距大于0.1时
        else
            break
        end
        %首先建立障碍物的极直方图
        i=1;mag = zeros(n,1);his=zeros(n,1);
        % 下面一段程序得到机器人360度范围视野内的障碍物分布值 72个扇区的极障碍物密度
        while (i<=length(obstacle)) 
           d = norm(obstacle(i,:) - robot); % 障碍物栅格与机器人之间距离
            if (d<dmax)
                beta = caculatebeta(robot,obstacle(i,:));  % 障碍物栅格向量的方向
                rangle=asin(rsafe/d);        % 扩大的角度
                k = round(beta/alpha);       % 逆时针数，第k个扇区区域
                if(k == 0)
                    k = 1;
                end
                % 更新后的极坐标直方图的h值
                if((5*k>rad2deg(beta)-rad2deg(rangle))&&(5*k<rad2deg(beta)+rad2deg(rangle)))  
                    h(k)=1;
                else
                    h(k)=0;
                end
                i=i+1;
                m = C^2*Iij(d);   % 障碍物栅格的向量幅值，与VFH计算方法不同
                mag(k)=max(mag(k),m.*h(k));   % mag为zeros(n,1)，mag的第k个元素为m
                i=i+1;
            else
                i=i+1;
            end
        end
        %接着应用VFH+算法，考虑运动半径因素 排除更多的扇区
        i4=1;
        if  norm(robot-ref)==0
            km=kt;
        else
            km=dc;
        end
        k1=0;
        k2=0;       
        alpha;
        while (i4<=length(obstacle))
            % 考虑转向半径因素
            dirtr(1)=radius*sin(km*alpha); 
            dirtr(2)=radius*cos(km*alpha);         %右转向中心差量
            centerr(1)=robot(1)+dirtr(1); centerr(2)=robot(2)+dirtr(2); %右转向中心坐标
            dirtl(1)=-radius*sin(km*alpha);  dirtl(2)=-radius*cos(km*alpha);        %左转向中心差量
            centerl(1)=robot(1)+dirtl(1); centerl(2)=robot(2)+dirtl(2); %左转向中心坐标
            %dor=norm(obstacle(i,:) - centerr);                          %障碍物到右转向中心的距离
            %dor=norm(obstacle(i,:) - centerl);                          %障碍物到左转向中心的距离
            dirtor(1)=obstacle(i4)-robot(1); dirtor(2)=obstacle(2)-robot(2); %障碍物到机器人的坐标差
            disor=(dirtr(1)-dirtor(1))^2+(dirtr(2)-dirtor(2))^2; %障碍物到右转向中心距离（平方）
            disol=(dirtl(1)-dirtor(1))^2+(dirtl(2)-dirtor(2))^2; %障碍物到左转向中心距离（平方）
            if 0<=km&&km<36
                k1=k1+1
                phib=km+36; %初始极限角度=运动方向的反方向
                phil=phib;  %初始左极限角度
                phir=phib;  %初始右极限角度
                beta = caculatebeta(robot,obstacle(i4,:));
                k = round(beta/alpha); %障碍物所在的扇区
                if km<=k&&k<phil  %障碍物在左半边区域
                    if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                        phil=k;
                        i5=phil;
                        while (phil<=i5&&i5<=phib)
                            mag(i5)=max(mag);
                            i5=i5+1;
                        end
                    end
                else
                %if (0<=k<km|phir<=k<=n) %障碍物在右半边区域
                    if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                        phir=k;
                        if phir<=k&&k<=n
                            i6=phib;
                            while (phib<=i6&&i6<=phir)
                                mag(i6)=max(mag);
                                i6=i6+1;
                            end
                        else
                        %if 0<=k<=km
                            i7=phib;
                            while (phib<=i7&&i7<=72)
                                mag(i7)=max(mag);
                                i7=i7+1;
                            end
                            i8=1;
                            while (0<=i8&&i8<=phir)
                                mag(i8)=max(mag);
                                i8=i8+1;
                            end
                        end
                    end
                end
            elseif 36<=km&&km<=72
               k2=k2+1
               phib=km-36; %初始极限角度=运动方向的反方向 
               phil=phib;  %初始左极限角度
               phir=phib;  %初始右极限角度
               beta = caculatebeta(robot,obstacle(i4,:));
               k = round(beta/alpha); %障碍物所在的扇区
               if k~=0
               else
                   k=1;
               end
               if phir<=k&&k<km  %障碍物在右半边区域
                  if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                     phir=k;
                     i9=phib;
                     while (phib<=i9&&i9<=phir)
                           mag(i9)=max(mag);
                           i9=i9+1;
                     end
                  end
               else
               %if (km<=k<72|0<=k<=phil) %障碍物在左半边区域
                   if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                      phil=k;
                        if 0<=k&&k<=phib
                            i10=phil;

                            while (phil<=i10&&i10<=phib)
                                mag(i10)=max(mag);
                                i10=i10+1;
                            end
                        else
                        %if km<=k<=72
                            i11=1;
                            while (0<=i11&&i11<=phib)
                                mag(i11)=max(mag);
                                i11=i11+1;
                            end
                            i12=1;
                            while (phil<=i12&&i12<=72)
                                mag(i12)=max(mag);
                                i12=i12+1;
                            end
                        end
                   end
               end  
            end
            i4=i4+1;
        end
        his=mag;      %现在 his 是一个含72个元素的向量--各扇区极障碍物密度
        %现在利用自适应阈值求一组备选方向（一组里有若干个备选方向）
        i1=1; %自适应阈值的循环次数
        kb=cell(1,blcs);
        howth=[];
        while (i1<=blcs)   % 自适应阈值的while循环变量：i1 i1取9的时候 会在某一点停下 i1取15的时候 完成避障！说明自适应阈值生效！！！
            %kb2=zeros(9,1);
            %howth2=zeros(9,1);
            Dt=norm(robot-endpoint);
            Dth(i1)=Dthmax-i1*dirtD;
            c=[];
            if  Dth(i1)<Dt
                threshold(i1)=C^2*Iij(Dth(i1));
                j=1;q=1;
                
                while (q<=n)       
                    %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                    if(his(q)< threshold(i1))
                        kr=q;                        % 找到了波谷的左端
                        while(q<=n && his(q)< threshold(i1))   %这一小段找到了波谷的右端
                            kl=q;
                            q=q+1;
                        end

                        if(kl-kr > smax)                  % 宽波谷
                            c   =  [c round(kl - smax/2)];  % 朝向左侧
                            c   =  [c round(kr + smax/2)];  % 朝向右侧
                            %j=j+1;
                            if(kt >= kr && kt <= kl)
                                c  = [c kt];                % straight at look ahead
                                %j=j+1;
                            end
                         elseif(kl-kr > smax/5)           % 窄波谷
                            c   =  [c round((kr+kl)/2-2.5)];
                            %j=j+1;
                         end

                    else
                        q=q+1;                            % his(q)不为0，直接下一个

                    end                                   % 退出if选择，再次进入while条件循环
                end                                       % 退出while循环
                % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                numb=length(c);
                temp2=howmuchs(Dth(i1),Dthmax,numb,c,kt);
            end 
            kb{1,i1}=c;
            howth=[howth temp2];       %存储阈值综合代价    
            i1=i1+1;
        end
        ftth=find(howth==min(howth));
        kbbest=kb{1,ftth(1)};   %此时，获得一个最佳阈值下的若干备选方向
        %现在，要判断是否采用VFH*算法进行路径规划
        %判断这组备选方向能否分成左右组：先确定当前运动方向
        %情况一：
        %if  (0<=km&&km<=round((ds/radius)/alpha))||(72-round((ds/radius)/alpha)<=km&&km<=72) 
        if 0<=km&&km<=round((ds/radius)/alpha)  %锐角情况
           lml=km+round((ds/radius)/alpha); %左边界
           lmr=km-round((ds/radius)/alpha)+72; %右边界
           numb1=length(kbbest); 
           phib=km+36; %当前运动反方向
           tempkl=[];
           tempkr=[];
           for j1=1:numb1
               if km<=kbbest(j1)&&kbbest(j1)<=lml  %左边方向可达
                  tempkl=[tempkl kbbest(j1)];
               %end
               elseif lml<=kbbest(j1)&&kbbest(j1)<=phib %左边方向不可达
                  tempkl=[tempkl lml];
               %end
               elseif phib<=kbbest(j1)&&kbbest(j1)<=lmr %右边方向不可达
                  tempkr=[tempkr lmr];
               %end
               elseif lmr<=kbbest(j1)&&kbbest(j1)<=72  %右边方向可达 状态一
                  tempkr=[tempkr kbbest(j1)];
               %end
               %if 0<=kbbest(j1)<=km  %右边方向可达 状态二 
               else
                  tempkr=[tempkr kbbest(j1)]; 
               end
           end         
        %else
        elseif 72-round((ds/radius)/alpha)<=km&&km<=72  %钝角情况
           lml=km+round((ds/radius)/alpha)-72;  %左边界
           if lml~=0
           else
              lml=1; 
           end
           lmr=km-round((ds/radius)/alpha);  %右边界
           numb1=length(kbbest);
           phib=km-36;  %当前运动反方向
           tempkl=[];
           tempkr=[];
           for j2=1:numb1
               if phib<=kbbest(j2)&&kbbest(j2)<=lmr  %右边方向不可达
                  tempkr=[tempkr lmr]; 
               %end
               elseif lmr<=kbbest(j2)&&kbbest(j2)<=km  %右边方向可达
                  tempkr=[tempkr kbbest(j2)]; 
               %end
               elseif km<=kbbest(j2)&&kbbest(j2)<=72  %左边方向可达 状态一
                  tempkl=[tempkl kbbest(j2)];
               %end
               elseif 0<=kbbest(j2)&&kbbest(j2)<=lml  %左边方向可达 状态二
                  tempkl=[tempkl kbbest(j2)]; 
               %end
               else
               %if lml<=kbbest(j2)<=phib  %左边方向不可达
                  tempkl=[tempkl lml]; 
               end
           end
        
        %end
        %情况二
        elseif  round((ds/radius)/alpha)<=km&&km<=36 
            lml=km+round((ds/radius)/alpha); %左边界
            lmr=km-round((ds/radius)/alpha); %右边界
            numb1=length(kbbest); 
            phib=km+36; %当前运动反方向
            tempkl=[];
            tempkr=[];
            for j3=1:numb1
                if km<=kbbest(j3)&&kbbest(j3)<=lml  %左边方向可达
                   tempkl=[tempkl kbbest(j3)]; 
                %end
                elseif lml<=kbbest(j3)&&kbbest(j3)<=phib  %左边方向不可达
                   tempkl=[tempkl lml]; 
                %end
                elseif km<=kbbest(j3)&&kbbest(j3)<=lmr  %右边方向可达
                   tempkr=[tempkr kbbest(j3)]; 
                %end
                else
                tempkr=[tempkr lmr];  %右边方向不可达
                end
            end 
        %end
        else
        %情况三
        %if  36<=km<=72-round((ds/radius)/alpha)
            lml=km+round((ds/radius)/alpha); %左边界
            lmr=km-round((ds/radius)/alpha); %右边界
            numb1=length(kbbest); 
            phib=km-36; %当前运动反方向
            tempkl=[];
            tempkr=[];
            for j4=1:numb1
                if lmr<=kbbest(j4)&&kbbest(j4)<=km  %右边方向可达
                   tempkr=[tempkr kbbest(j4)]; 
                %end
                elseif phib<=kbbest(j4)&&kbbest(j4)<=lmr  %右边方向不可达
                   tempkr=[tempkr lmr]; 
                %end
                elseif km<=kbbest(j4)&&kbbest(j4)<=lml  %左边方向可达
                   tempkl=[tempkl kbbest(j4)]; 
                %end
                else
                tempkl=[tempkl lml];  %左边方向不可达
                end
            end
        end
        % 判定是否进入VFH*算法 左右两边是不是都至少有一个方向
        geshul=length(tempkl);
        geshur=length(tempkr);
        if  geshul>=1&&geshur>=1  %符合条件-开始进入VFH*算法
            refp=cell(1,6);  %未来投影位置的坐标
            gc=cell(1,6);  %未来投影位置的代价
            hc=cell(1,6);  %未来投影位置的启发值
            fc=cell(1,6);  %未来投影位置的综合判定值
            pp=cell(1,6);  %未来投影位置的节点
            pfcun=zeros(1,4);  %储存每个节点的最佳方向
            ng=1;  % 开始VFH* 第一次搜索
            %先确定第一个节点的左右方向
            if  norm(robot-ref)==0
                lc=kt;
            else
            %if  norm(robot-ref)~=0
                lc=lc;
            end 
            g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
            for i14=1:geshul
                g1l(i14)=tempkl(i14);
                order1l=g1l(i14);
                ol1l=g1l(i14)-1;
                or1l=g1l(i14)+1;
                if ol1l~=0
                   if or1l~=73
                   how1l(i14,1)=howmany(g1l(i14),kt,km,lc,his(order1l),his(ol1l),his(or1l));
                   else
                   how1l(i14,1)=howmany(g1l(i14),kt,km,lc,his(order1l),his(ol1l),inf);    
                   end
                else
                   how1l(i14,1)=howmany(g1l(i14),kt,km,lc,his(order1l),inf,his(or1l)); 
                end
            end
            ft1l=find(how1l==min(how1l));
            ft1l;
            
            pf1lbest=filt(g1l(ft1l),kt);  %找到左边最佳方向
            g1r=zeros(geshur,1);how1r=zeros(geshur,1);  %右边前进方向
            for i15=1:geshur
                g1r(i15)=tempkr(i15);
                order1r=g1r(i15);
                ol1r=g1r(i15)-1;
                or1r=g1r(i15)+1;
                if ol1r~=0
                   if or1r~=73 
                   how1r(i15)=howmany(g1r(i15),kt,km,lc,his(order1r),his(ol1r),his(or1r));
                   else
                   how1r(i15)=howmany(g1r(i15),kt,km,lc,his(order1r),his(ol1r),inf);    
                   end
                else
                   how1r(i15)=howmany(g1r(i15),kt,km,lc,his(order1r),inf,his(or1r));
                end
            end
            ft1r=find(how1r==min(how1r));
            pf1rbest=filt(g1r(ft1r),kt);  %找到右边最佳方向
            %前进（投影）
            pp{1,1}=robot+[v_car*time*cos(pf1lbest*alpha),v_car*time*sin(pf1lbest*alpha)];
            pp{1,2}=robot+[v_car*time*cos(pf1rbest*alpha),v_car*time*sin(pf1rbest*alpha)];
            ol1l=pf1lbest-1;
            or1l=pf1lbest+1;
            order1l=pf1lbest;
            if ol1l~=0
               if or1l~=73 
               gc{1,1}=howmanys(pf1lbest,kt,km,lc,his(order1l),his(ol1l),his(or1l),ng);
               else
               gc{1,1}=howmanys(pf1lbest,kt,km,lc,his(order1l),his(ol1l),inf,ng);    
               end
            else
               gc{1,1}=howmanys(pf1lbest,kt,km,lc,his(order1l),inf,his(or1l),ng); 
            end
            ol1r=pf1rbest-1;
            or1r=pf1rbest+1;
            order1r=pf1rbest;
            if ol1r~=0
               if or1r~=73  
               gc{1,2}=howmanys(pf1rbest,kt,km,lc,his(order1r),his(ol1r),his(or1r),ng);
               else
               gc{1,2}=howmanys(pf1rbest,kt,km,lc,his(order1r),his(ol1r),inf,ng);    
               end
            else
               gc{1,2}=howmanys(pf1rbest,kt,km,lc,his(order1r),inf,his(or1r),ng); 
            end
            kt1l=round(caculatebeta(pp{1,1},endpoint)/alpha);  %新节点的目标方向 左
            kt1r=round(caculatebeta(pp{1,2},endpoint)/alpha);  %新节点的目标方向 右
            hc{1,1}=heu(kt1l,pf1lbest,pf1lbest,ng);
            hc{1,2}=heu(kt1r,pf1rbest,pf1rbest,ng);
            fc{1,1}=gc{1,1}+hc{1,1};
            fc{1,2}=gc{1,2}+hc{1,2};
            % 开始 VFH* 第二次搜索
            ng=ng+1;
            % 建立第一个投影点的极直方图
            i2=1;mag1l = zeros(n,1);his1l=zeros(n,1);
            while (i2<=length(obstacle))  
            
            %%%%%%%%%%% 下面一段程序得到投影点一的360度范围视野内的障碍物分布值 72个扇区的极障碍物密度  
            
                d1l = norm(obstacle(i2,:) - pp{1,1}); % 障碍物栅格与机器人之间距离
                if (d1l<dmax)
                    beta1l = caculatebeta(pp{1,1},obstacle(i2,:));  % 障碍物栅格向量的方向
                    rangle1l=asin(rsafe/d1l);        % 扩大的角度
                    k1l = round(beta1l/alpha);       % 逆时针数，第k个扇区区域
                    if(k1l == 0)
                        k1l = 1;
                    end
                    % 更新后的极坐标直方图的h值
                    if((5*k1l>rad2deg(beta1l)-rad2deg(rangle1l))&&(5*k1l<rad2deg(beta1l)+rad2deg(rangle1l)))  
                        h1l(k1l)=1;
                    else
                        h1l(k1l)=0;
                    end
                    i2=i2+1;

                    m1l = C^2*Iij(d1l);   % 障碍物栅格的向量幅值，与VFH计算方法不同
                    mag1l(k1l)=max(mag1l(k1l),m1l.*h1l(k1l));   % mag为zeros(n,1)，mag的第k个元素为m
                    i2=i2+1;
                else
                    i2=i2+1;
                end
            end
            % 第一个投影点：用VFH+排除一些扇区
            i41l=1; %应用VFH+算法，考虑运动半径因素 排除更多的扇区
            if  norm(pp{1,1}-ref)==0
                km=kt;
            else
            %if  norm(pp{1,1}-ref)~=0
                km=pf1lbest;
            end
            while (i41l<=length(obstacle))

                %%%%%%%%%% 考虑转向半径因素
                    km
                    dirtr(1)=radius*sin(km*alpha);   dirtr(2)=radius*cos(km*alpha);         %右转向中心差量
                    centerr(1)=pp{1,1}(1)+dirtr(1); centerr(2)=pp{1,1}(2)+dirtr(2); %右转向中心坐标
                    dirtl(1)=-radius*sin(km*alpha);  dirtl(2)=-radius*cos(km*alpha);        %左转向中心差量
                    centerl(1)=pp{1,1}(1)+dirtl(1); centerl(2)=pp{1,1}(2)+dirtl(2); %左转向中心坐标
                    %dor=norm(obstacle(i,:) - centerr);                          %障碍物到右转向中心的距离
                    %dor=norm(obstacle(i,:) - centerl);                          %障碍物到左转向中心的距离
                    dirtor(1)=obstacle(i41l)-pp{1,1}(1); dirtor(2)=obstacle(2)-pp{1,1}(2); %障碍物到机器人的坐标差
                    disor=(dirtr(1)-dirtor(1))^2+(dirtr(2)-dirtor(2))^2; %障碍物到右转向中心距离（平方）
                    disol=(dirtl(1)-dirtor(1))^2+(dirtl(2)-dirtor(2))^2; %障碍物到左转向中心距离（平方）
                    if 0<=km&&km<36
                        phib=km+36; %初始极限角度=运动方向的反方向
                        phil=phib;  %初始左极限角度
                        phir=phib;  %初始右极限角度
                        beta = caculatebeta(pp{1,1},obstacle(i41l,:));
                        k = round(beta/alpha); %障碍物所在的扇区
                        if km<=k&&km<phil  %障碍物在左半边区域
                            if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                                phil=k;
                                i5=phil;
                                while (phil<=i5&&i5<=phib)
                                    mag1l(i5)=max(mag1l);
                                    i5=i5+1;
                                end
                            end
                        else
                        %if (0<=k<km|phir<=k<=n) %障碍物在右半边区域
                            if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                                phir=k;
                                if phir<=k&&k<=n
                                    i6=phib;
                                    while (phib<=i6&&i6<=phir)
                                        mag1l(i6)=max(mag1l);
                                        i6=i6+1;
                                    end
                                else
                                %if 0<=k<=km
                                    i7=phib;
                                    while (phib<=i7&&i7<=72)
                                        mag1l(i7)=max(mag1l);
                                        i7=i7+1;
                                    end
                                    i8=1;
                                    while (0<=i8&&i8<=phir)
                                        mag1l(i8)=max(mag1l);
                                        i8=i8+1;
                                    end
                                end
                            end
                        end
                    else
                    %if 36<=km<=72
                       phib=km-36; %初始极限角度=运动方向的反方向 
                       phil=phib;  %初始左极限角度
                       phir=phib;  %初始右极限角度
                       beta = caculatebeta(pp{1,1},obstacle(i41l,:));
                       k = round(beta/alpha); %障碍物所在的扇区
                       if k~=0
                       else
                           k=1;
                       end
                       if phir<=k&&k<km  %障碍物在右半边区域
                          if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                             phir=k;
                             i9=phib;
                             while (phib<=i9&&i9<=phir)
                                   mag1l(i9)=max(mag1l);
                                   i9=i9+1;
                             end
                          end
                       else
                       %if (km<=k<72|0<=k<=phil) %障碍物在左半边区域
                           if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                              phil=k;
                                if 0<=k&&k<=phib
                                    i10=phil;
                                    while (phil<=i10&&i10<=phib)
                                        mag1l(i10)=max(mag1l);
                                        i10=i10+1;
                                    end
                                else
                                %if km<=k<=72
                                    i11=1;
                                    while (0<=i11&&i11<=phib)
                                        mag1l(i11)=max(mag1l);
                                        i11=i11+1;
                                    end
                                    i12=1;
                                    while (phil<=i12&&i12<=72)
                                        mag1l(i12)=max(mag1l);
                                        i12=i12+1;
                                    end
                                end
                           end
                       end  
                    end
                    i41l=i41l+1;
            end
        
            his1l=mag1l;      %现在 his 是一个含72个元素的向量--各扇区极障碍物密度
            % 第一个投影点：选取一组最佳备选方向
            i1=1; %自适应阈值的循环次数
            kb=cell(1,blcs);
            howth=[];
            while (i1<=blcs)   % 自适应阈值的while循环变量：i1 i1取9的时候 会在某一点停下 i1取15的时候 完成避障！说明自适应阈值生效！！！
                %kb2=zeros(9,1);
                %howth2=zeros(9,1);
                Dt=norm(pp{1,1}-endpoint);
                Dth(i1)=Dthmax-i1*dirtD;
                c=[];
                if  Dth(i1)<Dt
                    threshold(i1)=C^2*Iij(Dth(i1));
                    j=1;q=1;
                    
                    while (q<=n)       
                        %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                        if(his1l(q)< threshold(i1))
                            kr=q;                        % 找到了波谷的左端
                            while(q<=n && his1l(q)< threshold(i1))   %这一小段找到了波谷的右端
                                kl=q;
                                q=q+1;
                            end

                            if(kl-kr > smax)                  % 宽波谷
                                c   = [c round(kl - smax/2)];  % 朝向左侧
                                c   = [c round(kr + smax/2)];  % 朝向右侧
                                %j=j+2;
                                if(kt1l >= kr && kt1l <= kl)
                                    c  = [c kt1l];                % straight at look ahead
                                    %j=j+1;
                                end
                             elseif(kl-kr > smax/5)           % 窄波谷
                                c  = [c round((kr+kl)/2-2.5)];
                                %j=j+1;
                             end

                        else
                            q=q+1;                            % his(q)不为0，直接下一个

                        end                                   % 退出if选择，再次进入while条件循环
                    end                                       % 退出while循环

                    % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                    numb=length(c);
                    temp2=howmuchs(Dth(i1),Dthmax,numb,c,kt1l);
                end 

                %temp1=howmuch(Dth(i1),fk,kt,Dthmax);   
                kb{1,i1}=c;
                howth=[howth temp2];       %存储阈值综合代价    
                i1=i1+1;
            end
            ftth=find(howth==min(howth));
            kbbest1l=kb{1,ftth(1)};   %此时，获得一个最佳阈值下的若干备选方向
            %第一个投影点：能否分左右
            
            %情况一：
            %if  0<=km<=round((ds/radius)/alpha)||72-round((ds/radius)/alpha)<=km<=72 
            if 0<=km&&km<=round((ds/radius)/alpha)  %锐角情况
               lml=km+round((ds/radius)/alpha); %左边界
               lmr=km-round((ds/radius)/alpha)+72; %右边界
               numb1=length(kbbest1l); 
               phib=km+36; %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j1=1:numb1
                   if km<=kbbest1l(j1)&&kbbest1l(j1)<=lml  %左边方向可达
                      tempkl=[tempkl kbbest1l(j1)];
                   %end
                   elseif lml<=kbbest1l(j1)&&kbbest1l(j1)<=phib %左边方向不可达
                      tempkl=[tempkl lml];
                   %end
                   elseif phib<=kbbest1l(j1)&&kbbest1l(j1)<=lmr %右边方向不可达
                      tempkr=[tempkr lmr];
                   %end
                   elseif lmr<=kbbest1l(j1)&&kbbest1l(j1)<=72  %右边方向可达 状态一
                      tempkr=[tempkr kbbest1l(j1)];
                   %end
                   else
                   %if 0<=kbbest1l(j1)<=km  %右边方向可达 状态二   
                      tempkr=[tempkr kbbest1l(j1)]; 
                   end
               end         
            
            elseif 72-round((ds/radius)/alpha)<=km&&km<=72  %钝角情况
               lml=km+round((ds/radius)/alpha)-72;  %左边界
               if lml~=0
               else
                  lml=1; 
               end
               lmr=km-round((ds/radius)/alpha);  %右边界
               numb1=length(kbbest1l);
               phib=km-36;  %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j2=1:numb1
                   if phib<=kbbest1l(j2)&&kbbest1l(j2)<=lmr  %右边方向不可达
                      tempkr=[tempkr lmr]; 
                   %end
                   elseif lmr<=kbbest1l(j2)&&kbbest1l(j2)<=km  %右边方向可达
                      tempkr=[tempkr kbbest1l(j2)]; 
                   %end
                   elseif km<=kbbest1l(j2)&&kbbest1l(j2)<=72  %左边方向可达 状态一
                      tempkl=[tempkl kbbest1l(j2)];
                   %end
                   elseif 0<=kbbest1l(j2)&&kbbest1l(j2)<=lml  %左边方向可达 状态二
                      tempkl=[tempkl kbbest1l(j2)]; 
                   %end
                   else
                   %if lml<=kbbest1l(j2)<=phib  %左边方向不可达
                      tempkl=[tempkl lml]; 
                   end
               end
            
            %end
            %情况二
            elseif  round((ds/radius)/alpha)<=km&&km<=36 
                lml=km+round((ds/radius)/alpha); %左边界
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1l); 
                phib=km+36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j3=1:numb1
                    if km<=kbbest1l(j3)&&kbbest1l(j3)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1l(j3)]; 
                    %end
                    elseif lml<=kbbest1l(j3)&&kbbest1l(j3)<=phib  %左边方向不可达
                       tempkl=[tempkl lml]; 
                    %end
                    elseif km<=kbbest1l(j3)&&kbbest1l(j3)<=lmr  %右边方向可达
                       tempkr=[tempkr kbbest1l(j3)]; 
                    else
                    tempkr=[tempkr lmr];  %右边方向不可达
                    end
                end 
            %end
            else
            %情况三
            %if  36<=km<=72-round((ds/radius)/alpha)
                lml=km+round((ds/radius)/alpha); %左边界
                if lml~=0
                else
                   lml=1; 
                end
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1l); 
                phib=km-36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j4=1:numb1
                    if lmr<=kbbest1l(j4)&&kbbest1l(j4)<=km  %右边方向可达
                       tempkr=[tempkr kbbest1l(j4)]; 
                    %end
                    elseif phib<=kbbest1l(j4)&&kbbest1l(j4)<=lmr  %右边方向不可达
                       tempkr=[tempkr lmr]; 
                    %end
                    elseif km<=kbbest1l(j4)&&kbbest1l(j4)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1l(j4)]; 
                    else
                    tempkl=[tempkl lml];  %左边方向不可达 
                    end
                end
            end
            
            geshul=length(tempkl);
            geshur=length(tempkr);
            %第一个投影点：能分左右
            if  geshul>=1&&geshur>=1  
                lc=pf1lbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                       if or1l~=73 
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l));
                       else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),inf);   
                       end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),inf,his1l(or1l));
                    end
                end
                ft1ll=find(how1l==min(how1l));
                pf1llbest=filt(g1l(ft1ll),kt1l);  %找到左边最佳方向 
                g1r=zeros(geshur,1);how1r=zeros(geshur,1);  %右边前进方向
                for i15=1:geshur
                    g1r(i15)=tempkr(i15);
                    order1r=g1r(i15);
                    ol1r=g1r(i15)-1;
                    or1r=g1r(i15)+1;
                    if ol1r~=0
                       if or1r~=73
                       how1r(i15)=howmany(g1r(i15),kt1l,km,lc,his1l(order1r),his1l(ol1r),his1l(or1r));
                       else
                       how1r(i15)=howmany(g1r(i15),kt1l,km,lc,his1l(order1r),his1l(ol1r),inf);    
                       end
                    else
                       how1r(i15)=howmany(g1r(i15),kt1l,km,lc,his1l(order1r),inf,his1l(or1r));
                    end
                end
                ft1lr=find(how1r==min(how1r));
                pf1lrbest=filt(g1r(ft1lr),kt1l);  %找到右边最佳方向
                %前进（投影）
                pp{1,3}=pp{1,1}+[v_car*time*cos(pf1llbest*alpha),v_car*time*sin(pf1llbest*alpha)];
                pp{1,4}=pp{1,1}+[v_car*time*cos(pf1lrbest*alpha),v_car*time*sin(pf1lrbest*alpha)];
                ol1l=pf1llbest-1;
                or1l=pf1llbest+1;
                order1l=pf1llbest;
                if ol1l~=0
                   if or1l~=73 
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l),ng)+gc{1,1};
                   else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),inf,ng)+gc{1,1};    
                   end
                else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),inf,his1l(or1l),ng)+gc{1,1}; 
                end
                ol1r=pf1lrbest-1;
                or1r=pf1lrbest+1;
                order1r=pf1lrbest;
                if ol1r~=0
                   if or1r~=73 
                   gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1r),his1l(ol1r),his1l(or1r),ng)+gc{1,1};
                   else
                   gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1r),his1l(ol1r),inf,ng)+gc{1,1};    
                   end
                else
                   gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1r),inf,his1l(or1r),ng)+gc{1,1}; 
                end
                kt1ll=round(caculatebeta(pp{1,3},endpoint)/alpha);  %新节点的目标方向 左
                kt1lr=round(caculatebeta(pp{1,4},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,3}=heu(kt1ll,pf1llbest,pf1llbest,ng);
                hc{1,4}=heu(kt1lr,pf1lrbest,pf1lrbest,ng);
                fc{1,3}=gc{1,3}+hc{1,3};
                fc{1,4}=gc{1,4}+hc{1,4};
                pfcun(1,1)=pf1llbest;
                pfcun(1,2)=pf1lrbest;
            %end
            %第一个投影点：不能分左右 只有一个方向
            elseif  geshul>=1&&geshur==0  %该方向是左
                lc=pf1lbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                       if or1l~=73 
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l));
                       else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),inf);    
                       end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),inf,his1l(or1l)); 
                    end
                end
                ft1ll=find(how1l==min(how1l));
                pf1llbest=filt(g1l(ft1ll),kt1l);  %找到左边最佳方向 
                pp{1,3}=pp{1,1}+[v_car*time*cos(pf1llbest*alpha),v_car*time*sin(pf1llbest*alpha)];
                ol1l=pf1llbest-1;
                or1l=pf1llbest+1;
                order1l=pf1llbest;
                if ol1l~=0
                   if or1l~=73 
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l),ng)+gc{1,1};
                   else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),inf,ng)+gc{1,1};   
                   end
                else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),inf,his1l(or1l),ng)+gc{1,1}; 
                end
                kt1ll=round(caculatebeta(pp{1,3},endpoint)/alpha);  %新节点的目标方向 左
                hc{1,3}=heu(kt1ll,pf1llbest,pf1llbest,ng);
                fc{1,3}=gc{1,3}+hc{1,3};
                fc{1,4}=inf;
                pfcun(1,1)=pf1llbest;
            %end
            elseif  geshur>=1&&geshul==0  %该方向是右
                lc=pf1lbest;
                g1l=zeros(geshur,1);how1l=zeros(geshur,1);  %右边前进方向 
                for i14=1:geshur
                    g1l(i14)=tempkr(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                        if or1l~=73
                        how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l));
                        else
                        how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),inf);    
                        end
                    else
                        how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),inf,his1l(or1l)); 
                    end
                end
                ft1lr=find(how1l==min(how1l));
                pf1lrbest=filt(g1l(ft1lr),kt1l);  %找到右边最佳方向 
                pp{1,4}=pp{1,1}+[v_car*time*cos(pf1lrbest*alpha),v_car*time*sin(pf1lrbest*alpha)];
                ol1l=pf1lrbest-1;
                or1l=pf1lrbest+1;
                order1l=pf1lrbest;
                if ol1l~=0
                    if or1l~=73
                    gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l),ng)+gc{1,1};
                    else
                    gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),inf,ng)+gc{1,1};    
                    end
                else
                    gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1l),inf,his1l(or1l),ng)+gc{1,1};
                end
                kt1lr=round(caculatebeta(pp{1,4},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,4}=heu(kt1lr,pf1lrbest,pf1lrbest,ng);
                fc{1,4}=gc{1,4}+hc{1,4};
                fc{1,3}=inf;
                pfcun(1,2)=pf1lrbest;
            %end
            %第一个投影点：不能分左右 无方向
            else
            %if  geshul==geshur==0
                fc{1,3}=inf;
                fc{1,4}=inf;
            end
            % 建立第二个投影点的极直方图
            i3=1;mag1r = zeros(n,1);his1r=zeros(n,1);
            while (i3<=length(obstacle))  
            
            %%%%%%%%%%% 下面一段程序得到机器人360度范围视野内的障碍物分布值 72个扇区的极障碍物密度  
            
                d1r = norm(obstacle(i3,:) - pp{1,2}); % 障碍物栅格与机器人之间距离
                if (d1r<dmax)
                    beta1r = caculatebeta(pp{1,2},obstacle(i3,:));  % 障碍物栅格向量的方向
                    rangle1r=asin(rsafe/d1r);        % 扩大的角度
                    k1r = round(beta1r/alpha);       % 逆时针数，第k个扇区区域
                    if(k1r == 0)
                        k1r = 1;
                    end
                    % 更新后的极坐标直方图的h值
                    if((5*k1r>rad2deg(beta1r)-rad2deg(rangle1r))&&(5*k1r<rad2deg(beta1r)+rad2deg(rangle1r)))  
                        h1r(k1r)=1;
                    else
                        h1r(k1r)=0;
                    end
                    i3=i3+1;

                    m1r = C^2*Iij(d1r);   % 障碍物栅格的向量幅值，与VFH计算方法不同
                    mag1r(k1r)=max(mag1r(k1r),m1r.*h1r(k1r));   % mag为zeros(n,1)，mag的第k个元素为m
                    i3=i3+1;
                else
                    i3=i3+1;
                end 
            end
            %第二个投影点：用VFH+排除一些扇区
            i41r=1; %应用VFH+算法，考虑运动半径因素 排除更多的扇区
            km=pf1rbest;
            while (i41r<=length(obstacle))

                %%%%%%%%%% 考虑转向半径因素
                    
                    dirtr(1)=radius*sin(km*alpha);   dirtr(2)=radius*cos(km*alpha);         %右转向中心差量
                    centerr(1)=pp{1,2}(1)+dirtr(1); centerr(2)=pp{1,2}(2)+dirtr(2); %右转向中心坐标
                    dirtl(1)=-radius*sin(km*alpha);  dirtl(2)=-radius*cos(km*alpha);        %左转向中心差量
                    centerl(1)=pp{1,2}(1)+dirtl(1); centerl(2)=pp{1,2}(2)+dirtl(2); %左转向中心坐标
                    %dor=norm(obstacle(i,:) - centerr);                          %障碍物到右转向中心的距离
                    %dor=norm(obstacle(i,:) - centerl);                          %障碍物到左转向中心的距离
                    dirtor(1)=obstacle(i41r)-pp{1,2}(1); dirtor(2)=obstacle(2)-pp{1,2}(2); %障碍物到机器人的坐标差
                    disor=(dirtr(1)-dirtor(1))^2+(dirtr(2)-dirtor(2))^2; %障碍物到右转向中心距离（平方）
                    disol=(dirtl(1)-dirtor(1))^2+(dirtl(2)-dirtor(2))^2; %障碍物到左转向中心距离（平方）
                    if 0<=km&&km<36
                        phib=km+36; %初始极限角度=运动方向的反方向
                        phil=phib;  %初始左极限角度
                        phir=phib;  %初始右极限角度
                        beta = caculatebeta(pp{1,2},obstacle(i41r,:));
                        k = round(beta/alpha); %障碍物所在的扇区
                        if km<=k&&k<phil  %障碍物在左半边区域
                            if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                                phil=k;
                                i5=phil;
                                while (phil<=i5&&i5<=phib)
                                    mag1r(i5)=max(mag1r);
                                    i5=i5+1;
                                end
                            end
                        else
                        %if (0<=k<km|phir<=k<=n) %障碍物在右半边区域
                            if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                                phir=k;
                                if phir<=k&&k<=n
                                    i6=phib;
                                    while (phib<=i6&&i6<=phir)
                                        mag1r(i6)=max(mag1r);
                                        i6=i6+1;
                                    end
                                else
                                %if 0<=k<=km
                                    i7=phib;
                                    while (phib<=i7&&i7<=72)
                                        mag1r(i7)=max(mag1r);
                                        i7=i7+1;
                                    end
                                    i8=1;
                                    while (0<=i8&&i8<=phir)
                                        mag1r(i8)=max(mag1r);
                                        i8=i8+1;
                                    end
                                end
                            end
                        end
                    else
                    %if 36<=km<=72
                       phib=km-36; %初始极限角度=运动方向的反方向 
                       phil=phib;  %初始左极限角度
                       phir=phib;  %初始右极限角度
                       beta = caculatebeta(pp{1,2},obstacle(i41r,:));
                       k = round(beta/alpha); %障碍物所在的扇区
                       if k~=0
                       else
                          k=1;
                       end
                       if phir<=k&&k<km  %障碍物在右半边区域
                          if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                             phir=k;
                             i9=phib;
                             while (phib<=i9&&i9<=phir)
                                   mag1r(i9)=max(mag1r);
                                   i9=i9+1;
                             end
                          end
                       else
                       %if (km<=k<72|0<=k<=phil) %障碍物在左半边区域
                           if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                              phil=k;
                                if 0<=k&&k<=phib
                                    i10=phil;
                                    while (phil<=i10&&i10<=phib)
                                        i10
                                        mag1r(i10)=max(mag1r);
                                        i10=i10+1;
                                    end
                                else
                                %if km<=k<=72
                                    i11=1;
                                    while (0<=i11&&i11<=phib)
                                        mag1r(i11)=max(mag1r);
                                        i11=i11+1;
                                    end
                                    i12=1;
                                    while (phil<=i12&&i12<=72)
                                        mag1r(i12)=max(mag1r);
                                        i12=i12+1;
                                    end
                                end
                           end
                       end  
                    end
                    i41r=i41r+1;
            end
        
            his1r=mag1r;      %现在 his 是一个含72个元素的向量--各扇区极障碍物密度
            %第二个投影点：选取一组最佳备选方向
            i1=1; %自适应阈值的循环次数
            kb=cell(1,blcs);
            howth=[];
            while (i1<=blcs)   % 自适应阈值的while循环变量：i1 i1取9的时候 会在某一点停下 i1取15的时候 完成避障！说明自适应阈值生效！！！
                %kb2=zeros(9,1);
                %howth2=zeros(9,1);
                Dt=norm(pp{1,2}-endpoint);
                Dth(i1)=Dthmax-i1*dirtD;
                c=[];
                if  Dth(i1)<Dt
                    threshold(i1)=C^2*Iij(Dth(i1));
                    j=1;q=1;
                    
                    while (q<=n)       
                        %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                        if(his1r(q)< threshold(i1))
                            kr=q;                        % 找到了波谷的左端
                            while(q<=n && his1r(q)< threshold(i1))   %这一小段找到了波谷的右端
                                kl=q;
                                q=q+1;
                            end

                            if(kl-kr > smax)                  % 宽波谷
                                c   = [c round(kl - smax/2)];  % 朝向左侧
                                c   = [c round(kr + smax/2)];  % 朝向右侧
                                %j=j+2;
                                if(kt1r >= kr && kt1r <= kl)
                                    c  = [c kt1r];                % straight at look ahead
                                    %j=j+1;
                                end
                             elseif(kl-kr > smax/5)           % 窄波谷
                                c  = [c round((kr+kl)/2-2.5)];
                                %j=j+1;
                             end

                        else
                            q=q+1;                            % his(q)不为0，直接下一个

                        end                                   % 退出if选择，再次进入while条件循环
                    end                                       % 退出while循环

                    % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                    numb=length(c);
                    temp2=howmuchs(Dth(i1),Dthmax,numb,c,kt1r);
                end 

                %temp1=howmuch(Dth(i1),fk,kt,Dthmax);   
                kb{1,i1}=c;
                howth=[howth temp2];       %存储阈值综合代价    
                i1=i1+1;
            end
            ftth=find(howth==min(howth));
            kbbest1r=kb{1,ftth(1)};   %此时，获得一个最佳阈值下的若干备选方向
            %第二个投影点：能否分左右
            %情况一：
            %if  0<=km<=round((ds/radius)/alpha)||72-round((ds/radius)/alpha)<=km<=72 
            if 0<=km&&km<=round((ds/radius)/alpha)  %锐角情况
               lml=km+round((ds/radius)/alpha); %左边界
               lmr=km-round((ds/radius)/alpha)+72; %右边界
               numb1=length(kbbest1r); 
               phib=km+36; %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j1=1:numb1
                   if km<=kbbest1r(j1)&&kbbest1r(j1)<=lml  %左边方向可达
                      tempkl=[tempkl kbbest1r(j1)];
                   %end
                   elseif lml<=kbbest1r(j1)&&kbbest1r(j1)<=phib %左边方向不可达
                      tempkl=[tempkl lml];
                   %end
                   elseif phib<=kbbest1r(j1)&&kbbest1r(j1)<=lmr %右边方向不可达
                      tempkr=[tempkr lmr];
                   %end
                   elseif lmr<=kbbest1r(j1)&&kbbest1r(j1)<=72  %右边方向可达 状态一
                      tempkr=[tempkr kbbest1r(j1)];
                   %end
                   else
                   %if 0<=kbbest1r(j1)<=km  %右边方向可达 状态二   
                      tempkr=[tempkr kbbest1r(j1)]; 
                   end
               end         
            
            elseif 72-round((ds/radius)/alpha)<=km&&km<=72  %钝角情况
               lml=km+round((ds/radius)/alpha)-72;  %左边界
               if lml~=0
               else
                  lml=1; 
               end
               lmr=km-round((ds/radius)/alpha);  %右边界
               numb1=length(kbbest1r);
               phib=km-36;  %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j2=1:numb1
                   if phib<=kbbest1r(j2)&&kbbest1r(j2)<=lmr  %右边方向不可达
                      tempkr=[tempkr lmr]; 
                   %end
                   elseif lmr<=kbbest1r(j2)&&kbbest1r(j2)<=km  %右边方向可达
                      tempkr=[tempkr kbbest1r(j2)]; 
                   %end
                   elseif km<=kbbest1r(j2)&&kbbest1r(j2)<=72  %左边方向可达 状态一
                      tempkl=[tempkl kbbest1r(j2)];
                   %end
                   elseif 0<=kbbest1r(j2)&&kbbest1r(j2)<=lml  %左边方向可达 状态二
                      tempkl=[tempkl kbbest1r(j2)]; 
                   %end
                   else
                   %if lml<=kbbest1r(j2)<=phib  %左边方向不可达
                      tempkl=[tempkl lml]; 
                   end
               end
            %end
            %end
            %情况二
            elseif  round((ds/radius)/alpha)<=km&&km<=36 
                lml=km+round((ds/radius)/alpha); %左边界
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1r); 
                phib=km+36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j3=1:numb1
                    if km<=kbbest1r(j3)&&kbbest1r(j3)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1r(j3)]; 
                    %end
                    elseif lml<=kbbest1r(j3)&&kbbest1r(j3)<=phib  %左边方向不可达
                       tempkl=[tempkl lml]; 
                    %end
                    elseif km<=kbbest1r(j3)&&kbbest1r(j3)<=lmr  %右边方向可达
                       tempkr=[tempkr kbbest1r(j3)]; 
                    else
                    tempkr=[tempkr lmr];  %右边方向不可达
                    end
                end 
            %end
            %情况三
            else
            %if  36<=km<=72-round((ds/radius)/alpha)
                lml=km+round((ds/radius)/alpha); %左边界
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1r); 
                phib=km-36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j4=1:numb1
                    if lmr<=kbbest1r(j4)&&kbbest1r(j4)<=km  %右边方向可达
                       tempkr=[tempkr kbbest1r(j4)]; 
                    %end
                    elseif phib<=kbbest1r(j4)&&kbbest1r(j4)<=lmr  %右边方向不可达
                       tempkr=[tempkr lmr]; 
                    %end
                    elseif km<=kbbest1r(j4)&&kbbest1r(j4)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1r(j4)]; 
                    else
                    tempkl=[tempkl lml];  %左边方向不可达
                    end
                end
            end
            
            geshul=length(tempkl);
            geshur=length(tempkr);
            %第二个投影点：能分左右
            if  geshul>=1&&geshur>=1  
                lc=pf1rbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                        if or1l~=73
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l));
                        else
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),inf);    
                        end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),inf,his1r(or1l)); 
                    end
                end
                ft1rl=find(how1l==min(how1l));
                pf1rlbest=filt(g1l(ft1rl),kt1r);  %找到左边最佳方向 
                g1r=zeros(geshur,1);how1r=zeros(geshur,1);  %右边前进方向
                for i15=1:geshur
                    g1r(i15)=tempkr(i15);
                    order1r=g1r(i15);
                    ol1r=g1r(i15)-1;
                    or1r=g1r(i15)+1;
                    if ol1r~=0
                       if or1r~=73 
                       how1r(i15)=howmany(g1r(i15),kt1r,km,lc,his1r(order1r),his1r(ol1r),his1r(or1r));
                       else
                       how1r(i15)=howmany(g1r(i15),kt1r,km,lc,his1r(order1r),his1r(ol1r),inf);    
                       end
                    else
                       how1r(i15)=howmany(g1r(i15),kt1r,km,lc,his1r(order1r),inf,his1r(or1r)); 
                    end
                end
                ft1rr=find(how1r==min(how1r));
                pf1rrbest=filt(g1r(ft1rr),kt1r);  %找到右边最佳方向
                pp{1,5}=pp{1,2}+[v_car*time*cos(pf1rlbest*alpha),v_car*time*sin(pf1rlbest*alpha)];
                pp{1,6}=pp{1,2}+[v_car*time*cos(pf1rrbest*alpha),v_car*time*sin(pf1rrbest*alpha)];
                ol1l=pf1rlbest-1;
                or1l=pf1rlbest+1;
                order1l=pf1rlbest;
                if ol1l~=0
                   if or1l~=73
                   gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l),ng)+gc{1,2};
                   else
                   gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),inf,ng)+gc{1,2};    
                   end
                else
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),inf,his1r(or1l),ng)+gc{1,2}; 
                end
                ol1r=pf1rrbest-1;
                or1r=pf1rrbest+1;
                order1r=pf1rrbest;
                if ol1r~=0
                   if or1r~=73
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1r),his1r(ol1r),his1r(or1r),ng)+gc{1,2};
                   else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1r),his1r(ol1r),inf,ng)+gc{1,2};    
                   end
                else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1r),inf,his1r(or1r),ng)+gc{1,2}; 
                end
                kt1rl=round(caculatebeta(pp{1,5},endpoint)/alpha);  %新节点的目标方向 左
                kt1rr=round(caculatebeta(pp{1,6},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,5}=heu(kt1rl,pf1rlbest,pf1rlbest,ng);
                hc{1,6}=heu(kt1rr,pf1rrbest,pf1rrbest,ng);
                fc{1,5}=gc{1,5}+hc{1,5};
                fc{1,6}=gc{1,6}+hc{1,6};
                pfcun(1,3)=pf1rlbest;
                pfcun(1,4)=pf1rrbest;
            %end
            %第二个投影点：不能分左右 只有一个方向
            elseif  geshul>=1&&geshur==0  %该方向是左
                lc=pf1rbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                tempkl
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                        if or1l~=73 
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l));
                        else
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),inf);    
                        end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),inf,his1r(or1l)); 
                    end
                end
                ft1rl=find(how1l==min(how1l));
                pf1rlbest=filt(g1l(ft1rl),kt1r);  %找到左边最佳方向
                pp{1,5}=pp{1,2}+[v_car*time*cos(pf1rlbest*alpha),v_car*time*sin(pf1rlbest*alpha)];
                ol1l=pf1rlbest-1;
                or1l=pf1rlbest+1;
                order1l=pf1rlbest;
                if ol1l~=0
                    if or1l~=73 
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l),ng)+gc{1,2};
                    else
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),inf,ng)+gc{1,2};
                    end
                else
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),inf,his1r(or1l),ng)+gc{1,2};
                end
                kt1rl=round(caculatebeta(pp{1,5},endpoint)/alpha);  %新节点的目标方向 左
                hc{1,5}=heu(kt1rl,pf1rlbest,pf1rlbest,ng);
                fc{1,5}=gc{1,5}+hc{1,5};
                fc{1,6}=inf;
                pfcun(1,3)=pf1rlbest;
            %end
            elseif  geshur>=1&&geshul==0  %该方向是右
                lc=pf1rbest;
                g1l=zeros(geshur,1);how1l=zeros(geshur,1);  %右边前进方向 
                for i14=1:geshur
                    g1l(i14)=tempkr(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                       if or1l~=73 
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l));
                       else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),inf);
                       end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),inf,his1r(or1l)); 
                    end
                end
                ft1rr=find(how1l==min(how1l));
                pf1rrbest=filt(g1l(ft1rr),kt1r);  %找到右边最佳方向
                pp{1,6}=pp{1,2}+[v_car*time*cos(pf1rrbest*alpha),v_car*time*sin(pf1rrbest*alpha)];
                ol1l=pf1rrbest-1;
                or1l=pf1rrbest+1;
                order1l=pf1rrbest;
                if ol1l~=0
                   if or1l~=73  
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l),ng)+gc{1,2};
                   else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),inf,ng)+gc{1,2};
                   end
                else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1l),inf,his1r(or1l),ng)+gc{1,2}; 
                end
                kt1rr=round(caculatebeta(pp{1,6},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,6}=heu(kt1rr,pf1rrbest,pf1rrbest,ng);
                fc{1,6}=gc{1,6}+hc{1,6};
                fc{1,5}=inf;
                pfcun(1,4)=pf1rrbest;
            else
            %第二个投影点：不能分左右 无方向
            %if  geshul==geshur==0
                fc{1,5}=inf;
                fc{1,6}=inf;
            end
            pd=[fc{1,3},fc{1,4},fc{1,5},fc{1,6}]  %判定值
            ft=find(pd==min(pd)) %找出几个节点fc值最小的
            dcyb=pfcun(1,ft);
            %lc=dc;
            
            if  length(ft)==1
                lc=dcyb;
                dc=dcyb;
                robot=pp{1,ft+2};  %VFH*算法得到的最终位置并赋予机器人
            elseif  length(ft)==2 %注意 ft 和 dcyb 此时是2行1列的矩阵（2×1）
                geshu=length(dcyb);
                if dcyb(1)==dcyb(geshu)
                   dc=dcyb(1); 
                   lc=dc;
                   robot=pp{1,ft(1)+2};
                else
                   kt_1=round(caculatebeta(pp{1,ft(1)+2},endpoint)/alpha);
                   kt_2=round(caculatebeta(pp{1,ft(2)+2},endpoint)/alpha);
                   g_=zeros(geshu,1);how_=zeros(geshu,1);xushu=zeros(geshu,1);
                   kt_=[kt_1 kt_2];
                   for i14=1:geshu
                       g_(i14)=dcyb(i14);
                       how_(i14)=howmanyss(g_(i14),kt_(i14));
                       xushu(i14)=ft(i14);
                   end
                   if how_(1)~=how_(2)
                       ft_=find(how_==min(how_));
                       dc=g_(ft_);
                       lc=dc;
                       robot=pp{1,xushu(ft_)};
                   else
                       g__=zeros(geshu,1);how__=zeros(geshu,1);xushu_=zeros(geshu,1);
                       for i14=1:geshu
                           g__(i14)=dcyb(i14);
                           how__(i14)=dif(g__(i14),kt_(i14));
                           xushu_(i14)=ft(i14);
                       end
                       ft__=find(how__==min(how__));
                       dc=g__(ft__);
                       lc=dc;
                       robot=pp{1,xushu(ft__)};
                   end
                 end
            end

            %当前运动方向
            %上次选择方向
        else   %如果不进入VFH*算法 那就使用VFH+算法判定
            if  norm(robot-ref)==0
                lc=kt;
            else
                lc=lc;
            end
            
            i1=1; %自适应阈值的循环次数
            kb=[];%储存最佳方向集合
            howth=[];%考虑阈值与该阈值下方向的代价集合
            
            % 自适应阈值开始！
            
            while (i1<=blcs)
                Dt=norm(robot-endpoint);
                Dth(i1)=Dthmax-i1*dirtD;
                c=[];
                if  Dth(i1)<Dt
                    threshold(i1)=C^2*Iij(Dth(i1));
                    j=1;q=1;

                    while (q<=n)       
                        %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                        if(his(q)< threshold(i1))
                            kr=q;                        % 找到了波谷的左端
                            while(q<=n && his(q)< threshold(i1))   %这一小段找到了波谷的右端
                                kl=q;
                                q=q+1;
                            end

                            if(kl-kr > smax)                  % 宽波谷
                                c   =  [c round(kl - smax/2)];  % 朝向左侧
                                c   =  [c round(kr + smax/2)];  % 朝向右侧
                                j=j+2;
                                if(kt >= kr && kt <= kl)
                                    c  = [c kt];                % straight at look ahead
                                    j=j+1;
                                end
                            elseif(kl-kr > smax/5)           % 窄波谷
                                c   =  [c round((kr+kl)/2-2.5)];
                                j=j+1;
                            end

                        else
                            q=q+1;                            % his(q)不为0，直接下一个

                        end                                   % 退出if选择，再次进入while条件循环
                    end                                       % 退出while循环

                    % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                    % 开始筛选最优方向
                    if norm(robot-ref)==0                            
                       g=zeros(j-1,1);how=zeros(j-1,1);
                       for i2=1:j-1
                           g(i2)=c(i2);     %g中不含目标向量
                           order=g(i2);
                           ol=g(i2)-1;
                           or=g(i2)+1;
                           dc=kt;           %由于机器人还没动，所以目标方向就是当前运动方向
                           lc=kt;           %由于机器人还没动，所以目标方向就是上次选择方向
                           if ol~=0   %防止出现第0或第73
                              if or~=73
                               how(i2)=howmany(g(i2),kt,dc,lc,his(order),his(ol),his(or)); %代价函数计算最优方向 how为代价 元素个数与 g 是相同的
                              else
                               how(i2)=howmany(g(i2),kt,dc,lc,his(order),his(ol),inf);   
                              end
                           else
                              how(i2)=howmany(g(i2),kt,dc,lc,his(order),inf,his(or)); 
                           end 
                       end                                                             
                       ft=find(how==min(how));
                       fk=g(ft);
                       kb=[kb fk];  % 当前阈值下的最佳备选方向
                    else
                       g=zeros(j-1,1);how=zeros(j-1,1);
                       for i3=1:j-1
                           g(i3)=c(i3);
                           order=g(i3);
                           ol=g(i3)-1;
                           or=g(i3)+1;
                           if ol~=0   %防止出现第0或第73
                              if or~=73
                               how(i3)=howmany(g(i3),kt,dc,lc,his(order),his(ol),his(or));
                              else
                               how(i3)=howmany(g(i3),kt,dc,lc,his(order),his(ol),inf);   
                              end
                           else
                              how(i3)=howmany(g(i3),kt,dc,lc,his(order),inf,his(or)); 
                           end
                       end
                       ft=find(how==min(how));
                       fk=g(ft);
                       kb=[kb fk];  % 当前阈值下的最佳备选方向
                    end 
                    
                    temp1=howmuch(Dth(i1),fk,kt,Dthmax); %计算 阈值与该阈值下最佳方向的综合代价
                    howth=[howth temp1]; %存储综合代价
                end
                i1=i1+1;
            end
            ft=find(howth==min(howth));
            fbestyb=kb(ft);  %VFH+算法得到的最好方向
            % 防止有多个最优方向
            if  length(ft)==1
                dc=fbestyb;       % 当前的运动方向
                lc=dc;       % 上一次选择的方向
                robot=robot+[v_car*time*cos(fbestyb*alpha),v_car*time*sin(fbestyb*alpha)];  %VFH+算法得到的最终位置并赋予机器人
            elseif  length(ft)==2 %注意 ft 和 dcyb 此时是2行1列的矩阵（2×1）
                geshu=length(fbestyb);
                if fbestyb(1,1)==fbestyb(geshu,1)
                   dc=fbestyb(1,1);  % 当前的运动方向
                   lc=dc; % 上一次选择的方向
                   robot=robot+[v_car*time*cos(lc*alpha),v_car*time*sin(lc*alpha)];
                else
                   
                   g_=zeros(geshu,1);how_=zeros(geshu,1);  
                   for i14=1:geshu
                        g_(i14)=fbestyb(i14);
                        how_(i14)=howmanyss(g_(i14),kt);
                   end
                   ft_=find(how_==min(how_));
                   dc=g_(ft_); % 当前的运动方向
                   lc=dc; % 上一次选择的方向
                   robot=robot+[v_car*time*cos(lc*alpha),v_car*time*sin(lc*alpha)];
                end    
            elseif  length(ft)==3
                geshu=length(fbestyb);
                g_=zeros(geshu,1);how_=zeros(geshu,1);  
                   for i14=1:geshu
                        g_(i14)=fbestyb(i14);
                        how_(i14)=howmanyss(g_(i14),kt);
                   end
                   ft_=find(how_==min(how_));
                   dc=g_(ft_); % 当前的运动方向
                   lc=dc; % 上一次选择的方向
                   robot=robot+[v_car*time*cos(lc*alpha),v_car*time*sin(lc*alpha)];
            %else 
            end
        end
        ref=startpoint;
        scatter(robot(1),robot(2),'.r');
        drawnow;
        kt=round(caculatebeta(robot,endpoint)/alpha);  %新的目标方向
        if(kt==0)
            kt=n;
        end
        if(norm(robot-endpoint))>step          % 机器人位置和终点位置差距大于0.1时
        else
            break
        end
        %至此避障规划一次完毕
        obstacle([5271:8870],:)=[];    %避障完毕后剔除动态障碍物
        obstacle;
    elseif 4<i&&i<=10 %第二阶段（时间段）
        v=0.85; phiv=(1/18)*pi;  %速度大小及方向
        scatter(x0,y0,'.r');
        drawnow;
        x0=x0+v*cos(phiv)*time;  %圆心运动方程   
        y0=y0+v*sin(phiv)*time;  %圆心运动方程   
        x=x0+r*cos(phi);y=y0+r*sin(phi); %圆的方程
        xy=[];
        xy=[xy;x;y];
        xy=xy'; %转置后 得到圆上各点的坐标 
        plot(x,y,'.g');
        hold on
        mid=xy;
        obstacle=cat(1,obstacle,mid);  %动态障碍物加入静态障碍物坐标 cat为矩阵拼接函数 1竖着拼 2横着拼
        
        %%% 现在开始避障路径规划
        if(norm(robot-endpoint))>step          % 机器人位置和终点位置差距大于0.1时
        else
            break
        end
        %首先建立障碍物的极直方图
        i=1;mag = zeros(n,1);his=zeros(n,1);
        % 下面一段程序得到机器人360度范围视野内的障碍物分布值 72个扇区的极障碍物密度
        while (i<=length(obstacle)) 
           d = norm(obstacle(i,:) - robot); % 障碍物栅格与机器人之间距离
            if (d<dmax)
                beta = caculatebeta(robot,obstacle(i,:));  % 障碍物栅格向量的方向
                rangle=asin(rsafe/d);        % 扩大的角度
                k = round(beta/alpha);       % 逆时针数，第k个扇区区域
                if(k == 0)
                    k = 1;
                end
                % 更新后的极坐标直方图的h值
                if((5*k>rad2deg(beta)-rad2deg(rangle))&&(5*k<rad2deg(beta)+rad2deg(rangle)))  
                    h(k)=1;
                else
                    h(k)=0;
                end
                i=i+1;
                m = C^2*Iij(d);   % 障碍物栅格的向量幅值，与VFH计算方法不同
                mag(k)=max(mag(k),m.*h(k));   % mag为zeros(n,1)，mag的第k个元素为m
                i=i+1;
            else
                i=i+1;
            end
        end
        %接着应用VFH+算法，考虑运动半径因素 排除更多的扇区
        i4=1;
        if  norm(robot-ref)==0
            km=kt;
        else
            km=dc;
        end
        k1=0;
        k2=0;       
        alpha;
        while (i4<=length(obstacle))
            % 考虑转向半径因素
            dirtr(1)=radius*sin(km*alpha); 
            dirtr(2)=radius*cos(km*alpha);         %右转向中心差量
            centerr(1)=robot(1)+dirtr(1); centerr(2)=robot(2)+dirtr(2); %右转向中心坐标
            dirtl(1)=-radius*sin(km*alpha);  dirtl(2)=-radius*cos(km*alpha);        %左转向中心差量
            centerl(1)=robot(1)+dirtl(1); centerl(2)=robot(2)+dirtl(2); %左转向中心坐标
            %dor=norm(obstacle(i,:) - centerr);                          %障碍物到右转向中心的距离
            %dor=norm(obstacle(i,:) - centerl);                          %障碍物到左转向中心的距离
            dirtor(1)=obstacle(i4)-robot(1); dirtor(2)=obstacle(2)-robot(2); %障碍物到机器人的坐标差
            disor=(dirtr(1)-dirtor(1))^2+(dirtr(2)-dirtor(2))^2; %障碍物到右转向中心距离（平方）
            disol=(dirtl(1)-dirtor(1))^2+(dirtl(2)-dirtor(2))^2; %障碍物到左转向中心距离（平方）
            if 0<=km&&km<36
                k1=k1+1
                phib=km+36; %初始极限角度=运动方向的反方向
                phil=phib;  %初始左极限角度
                phir=phib;  %初始右极限角度
                beta = caculatebeta(robot,obstacle(i4,:));
                k = round(beta/alpha); %障碍物所在的扇区
                if km<=k&&k<phil  %障碍物在左半边区域
                    if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                        phil=k;
                        i5=phil;
                        while (phil<=i5&&i5<=phib)
                            mag(i5)=max(mag);
                            i5=i5+1;
                        end
                    end
                else
                %if (0<=k<km|phir<=k<=n) %障碍物在右半边区域
                    if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                        phir=k;
                        if phir<=k&&k<=n
                            i6=phib;
                            while (phib<=i6&&i6<=phir)
                                mag(i6)=max(mag);
                                i6=i6+1;
                            end
                        else
                        %if 0<=k<=km
                            i7=phib;
                            while (phib<=i7&&i7<=72)
                                mag(i7)=max(mag);
                                i7=i7+1;
                            end
                            i8=1;
                            while (0<=i8&&i8<=phir)
                                mag(i8)=max(mag);
                                i8=i8+1;
                            end
                        end
                    end
                end
            elseif 36<=km&&km<=72
               k2=k2+1
               phib=km-36; %初始极限角度=运动方向的反方向 
               phil=phib;  %初始左极限角度
               phir=phib;  %初始右极限角度
               beta = caculatebeta(robot,obstacle(i4,:));
               k = round(beta/alpha); %障碍物所在的扇区
               if k~=0
               else
                   k=1;
               end
               if phir<=k&&k<km  %障碍物在右半边区域
                  if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                     phir=k;
                     i9=phib;
                     while (phib<=i9&&i9<=phir)
                           mag(i9)=max(mag);
                           i9=i9+1;
                     end
                  end
               else
               %if (km<=k<72|0<=k<=phil) %障碍物在左半边区域
                   if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                      phil=k;
                        if 0<=k&&k<=phib
                            i10=phil;

                            while (phil<=i10&&i10<=phib)
                                mag(i10)=max(mag);
                                i10=i10+1;
                            end
                        else
                        %if km<=k<=72
                            i11=1;
                            while (0<=i11&&i11<=phib)
                                mag(i11)=max(mag);
                                i11=i11+1;
                            end
                            i12=1;
                            while (phil<=i12&&i12<=72)
                                mag(i12)=max(mag);
                                i12=i12+1;
                            end
                        end
                   end
               end  
            end
            i4=i4+1;
        end
        his=mag;      %现在 his 是一个含72个元素的向量--各扇区极障碍物密度
        %现在利用自适应阈值求一组备选方向（一组里有若干个备选方向）
        i1=1; %自适应阈值的循环次数
        kb=cell(1,blcs);
        howth=[];
        while (i1<=blcs)   % 自适应阈值的while循环变量：i1 i1取9的时候 会在某一点停下 i1取15的时候 完成避障！说明自适应阈值生效！！！
            %kb2=zeros(9,1);
            %howth2=zeros(9,1);
            Dt=norm(robot-endpoint);
            Dth(i1)=Dthmax-i1*dirtD;
            c=[];
            if  Dth(i1)<Dt
                threshold(i1)=C^2*Iij(Dth(i1));
                j=1;q=1;
                
                while (q<=n)       
                    %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                    if(his(q)< threshold(i1))
                        kr=q;                        % 找到了波谷的左端
                        while(q<=n && his(q)< threshold(i1))   %这一小段找到了波谷的右端
                            kl=q;
                            q=q+1;
                        end

                        if(kl-kr > smax)                  % 宽波谷
                            c   =  [c round(kl - smax/2)];  % 朝向左侧
                            c   =  [c round(kr + smax/2)];  % 朝向右侧
                            %j=j+1;
                            if(kt >= kr && kt <= kl)
                                c  = [c kt];                % straight at look ahead
                                %j=j+1;
                            end
                         elseif(kl-kr > smax/5)           % 窄波谷
                            c   =  [c round((kr+kl)/2-2.5)];
                            %j=j+1;
                         end

                    else
                        q=q+1;                            % his(q)不为0，直接下一个

                    end                                   % 退出if选择，再次进入while条件循环
                end                                       % 退出while循环
                % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                numb=length(c);
                temp2=howmuchs(Dth(i1),Dthmax,numb,c,kt);
            end 
            kb{1,i1}=c;
            howth=[howth temp2];       %存储阈值综合代价    
            i1=i1+1;
        end
        ftth=find(howth==min(howth));
        kbbest=kb{1,ftth(1)};   %此时，获得一个最佳阈值下的若干备选方向
        %现在，要判断是否采用VFH*算法进行路径规划
        %判断这组备选方向能否分成左右组：先确定当前运动方向
        %情况一：
        %if  (0<=km&&km<=round((ds/radius)/alpha))||(72-round((ds/radius)/alpha)<=km&&km<=72) 
        if 0<=km&&km<=round((ds/radius)/alpha)  %锐角情况
           lml=km+round((ds/radius)/alpha); %左边界
           lmr=km-round((ds/radius)/alpha)+72; %右边界
           numb1=length(kbbest); 
           phib=km+36; %当前运动反方向
           tempkl=[];
           tempkr=[];
           for j1=1:numb1
               if km<=kbbest(j1)&&kbbest(j1)<=lml  %左边方向可达
                  tempkl=[tempkl kbbest(j1)];
               %end
               elseif lml<=kbbest(j1)&&kbbest(j1)<=phib %左边方向不可达
                  tempkl=[tempkl lml];
               %end
               elseif phib<=kbbest(j1)&&kbbest(j1)<=lmr %右边方向不可达
                  tempkr=[tempkr lmr];
               %end
               elseif lmr<=kbbest(j1)&&kbbest(j1)<=72  %右边方向可达 状态一
                  tempkr=[tempkr kbbest(j1)];
               %end
               %if 0<=kbbest(j1)<=km  %右边方向可达 状态二 
               else
                  tempkr=[tempkr kbbest(j1)]; 
               end
           end         
        %else
        elseif 72-round((ds/radius)/alpha)<=km&&km<=72  %钝角情况
           lml=km+round((ds/radius)/alpha)-72;  %左边界
           if lml~=0
           else
              lml=1; 
           end
           lmr=km-round((ds/radius)/alpha);  %右边界
           numb1=length(kbbest);
           phib=km-36;  %当前运动反方向
           tempkl=[];
           tempkr=[];
           for j2=1:numb1
               if phib<=kbbest(j2)&&kbbest(j2)<=lmr  %右边方向不可达
                  tempkr=[tempkr lmr]; 
               %end
               elseif lmr<=kbbest(j2)&&kbbest(j2)<=km  %右边方向可达
                  tempkr=[tempkr kbbest(j2)]; 
               %end
               elseif km<=kbbest(j2)&&kbbest(j2)<=72  %左边方向可达 状态一
                  tempkl=[tempkl kbbest(j2)];
               %end
               elseif 0<=kbbest(j2)&&kbbest(j2)<=lml  %左边方向可达 状态二
                  tempkl=[tempkl kbbest(j2)]; 
               %end
               else
               %if lml<=kbbest(j2)<=phib  %左边方向不可达
                  tempkl=[tempkl lml]; 
               end
           end
        
        %end
        %情况二
        elseif  round((ds/radius)/alpha)<=km&&km<=36 
            lml=km+round((ds/radius)/alpha); %左边界
            lmr=km-round((ds/radius)/alpha); %右边界
            numb1=length(kbbest); 
            phib=km+36; %当前运动反方向
            tempkl=[];
            tempkr=[];
            for j3=1:numb1
                if km<=kbbest(j3)&&kbbest(j3)<=lml  %左边方向可达
                   tempkl=[tempkl kbbest(j3)]; 
                %end
                elseif lml<=kbbest(j3)&&kbbest(j3)<=phib  %左边方向不可达
                   tempkl=[tempkl lml]; 
                %end
                elseif km<=kbbest(j3)&&kbbest(j3)<=lmr  %右边方向可达
                   tempkr=[tempkr kbbest(j3)]; 
                %end
                else
                tempkr=[tempkr lmr];  %右边方向不可达
                end
            end 
        %end
        else
        %情况三
        %if  36<=km<=72-round((ds/radius)/alpha)
            lml=km+round((ds/radius)/alpha); %左边界
            lmr=km-round((ds/radius)/alpha); %右边界
            numb1=length(kbbest); 
            phib=km-36; %当前运动反方向
            tempkl=[];
            tempkr=[];
            for j4=1:numb1
                if lmr<=kbbest(j4)&&kbbest(j4)<=km  %右边方向可达
                   tempkr=[tempkr kbbest(j4)]; 
                %end
                elseif phib<=kbbest(j4)&&kbbest(j4)<=lmr  %右边方向不可达
                   tempkr=[tempkr lmr]; 
                %end
                elseif km<=kbbest(j4)&&kbbest(j4)<=lml  %左边方向可达
                   tempkl=[tempkl kbbest(j4)]; 
                %end
                else
                tempkl=[tempkl lml];  %左边方向不可达
                end
            end
        end
        % 判定是否进入VFH*算法 左右两边是不是都至少有一个方向
        geshul=length(tempkl);
        geshur=length(tempkr);
        if  geshul>=1&&geshur>=1  %符合条件-开始进入VFH*算法
            refp=cell(1,6);  %未来投影位置的坐标
            gc=cell(1,6);  %未来投影位置的代价
            hc=cell(1,6);  %未来投影位置的启发值
            fc=cell(1,6);  %未来投影位置的综合判定值
            pp=cell(1,6);  %未来投影位置的节点
            pfcun=zeros(1,4);  %储存每个节点的最佳方向
            ng=1;  % 开始VFH* 第一次搜索
            %先确定第一个节点的左右方向
            if  norm(robot-ref)==0
                lc=kt;
            else
            %if  norm(robot-ref)~=0
                lc=lc;
            end 
            g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
            for i14=1:geshul
                g1l(i14)=tempkl(i14);
                order1l=g1l(i14);
                ol1l=g1l(i14)-1;
                or1l=g1l(i14)+1;
                if ol1l~=0
                   if or1l~=73
                   how1l(i14,1)=howmany(g1l(i14),kt,km,lc,his(order1l),his(ol1l),his(or1l));
                   else
                   how1l(i14,1)=howmany(g1l(i14),kt,km,lc,his(order1l),his(ol1l),inf);    
                   end
                else
                   how1l(i14,1)=howmany(g1l(i14),kt,km,lc,his(order1l),inf,his(or1l)); 
                end
            end
            ft1l=find(how1l==min(how1l));
            ft1l;
            
            pf1lbest=filt(g1l(ft1l),kt);  %找到左边最佳方向
            g1r=zeros(geshur,1);how1r=zeros(geshur,1);  %右边前进方向
            for i15=1:geshur
                g1r(i15)=tempkr(i15);
                order1r=g1r(i15);
                ol1r=g1r(i15)-1;
                or1r=g1r(i15)+1;
                if ol1r~=0
                   if or1r~=73 
                   how1r(i15)=howmany(g1r(i15),kt,km,lc,his(order1r),his(ol1r),his(or1r));
                   else
                   how1r(i15)=howmany(g1r(i15),kt,km,lc,his(order1r),his(ol1r),inf);    
                   end
                else
                   how1r(i15)=howmany(g1r(i15),kt,km,lc,his(order1r),inf,his(or1r));
                end
            end
            ft1r=find(how1r==min(how1r));
            pf1rbest=filt(g1r(ft1r),kt);  %找到右边最佳方向
            %前进（投影）
            pp{1,1}=robot+[v_car*time*cos(pf1lbest*alpha),v_car*time*sin(pf1lbest*alpha)];
            pp{1,2}=robot+[v_car*time*cos(pf1rbest*alpha),v_car*time*sin(pf1rbest*alpha)];
            ol1l=pf1lbest-1;
            or1l=pf1lbest+1;
            order1l=pf1lbest;
            if ol1l~=0
               if or1l~=73 
               gc{1,1}=howmanys(pf1lbest,kt,km,lc,his(order1l),his(ol1l),his(or1l),ng);
               else
               gc{1,1}=howmanys(pf1lbest,kt,km,lc,his(order1l),his(ol1l),inf,ng);    
               end
            else
               gc{1,1}=howmanys(pf1lbest,kt,km,lc,his(order1l),inf,his(or1l),ng); 
            end
            ol1r=pf1rbest-1;
            or1r=pf1rbest+1;
            order1r=pf1rbest;
            if ol1r~=0
               if or1r~=73  
               gc{1,2}=howmanys(pf1rbest,kt,km,lc,his(order1r),his(ol1r),his(or1r),ng);
               else
               gc{1,2}=howmanys(pf1rbest,kt,km,lc,his(order1r),his(ol1r),inf,ng);    
               end
            else
               gc{1,2}=howmanys(pf1rbest,kt,km,lc,his(order1r),inf,his(or1r),ng); 
            end
            kt1l=round(caculatebeta(pp{1,1},endpoint)/alpha);  %新节点的目标方向 左
            kt1r=round(caculatebeta(pp{1,2},endpoint)/alpha);  %新节点的目标方向 右
            hc{1,1}=heu(kt1l,pf1lbest,pf1lbest,ng);
            hc{1,2}=heu(kt1r,pf1rbest,pf1rbest,ng);
            fc{1,1}=gc{1,1}+hc{1,1};
            fc{1,2}=gc{1,2}+hc{1,2};
            % 开始 VFH* 第二次搜索
            ng=ng+1;
            % 建立第一个投影点的极直方图
            i2=1;mag1l = zeros(n,1);his1l=zeros(n,1);
            while (i2<=length(obstacle))  
            
            %%%%%%%%%%% 下面一段程序得到投影点一的360度范围视野内的障碍物分布值 72个扇区的极障碍物密度  
            
                d1l = norm(obstacle(i2,:) - pp{1,1}); % 障碍物栅格与机器人之间距离
                if (d1l<dmax)
                    beta1l = caculatebeta(pp{1,1},obstacle(i2,:));  % 障碍物栅格向量的方向
                    rangle1l=asin(rsafe/d1l);        % 扩大的角度
                    k1l = round(beta1l/alpha);       % 逆时针数，第k个扇区区域
                    if(k1l == 0)
                        k1l = 1;
                    end
                    % 更新后的极坐标直方图的h值
                    if((5*k1l>rad2deg(beta1l)-rad2deg(rangle1l))&&(5*k1l<rad2deg(beta1l)+rad2deg(rangle1l)))  
                        h1l(k1l)=1;
                    else
                        h1l(k1l)=0;
                    end
                    i2=i2+1;

                    m1l = C^2*Iij(d1l);   % 障碍物栅格的向量幅值，与VFH计算方法不同
                    mag1l(k1l)=max(mag1l(k1l),m1l.*h1l(k1l));   % mag为zeros(n,1)，mag的第k个元素为m
                    i2=i2+1;
                else
                    i2=i2+1;
                end
            end
            % 第一个投影点：用VFH+排除一些扇区
            i41l=1; %应用VFH+算法，考虑运动半径因素 排除更多的扇区
            if  norm(pp{1,1}-ref)==0
                km=kt;
            else
            %if  norm(pp{1,1}-ref)~=0
                km=pf1lbest;
            end
            while (i41l<=length(obstacle))

                %%%%%%%%%% 考虑转向半径因素
                    km
                    dirtr(1)=radius*sin(km*alpha);   dirtr(2)=radius*cos(km*alpha);         %右转向中心差量
                    centerr(1)=pp{1,1}(1)+dirtr(1); centerr(2)=pp{1,1}(2)+dirtr(2); %右转向中心坐标
                    dirtl(1)=-radius*sin(km*alpha);  dirtl(2)=-radius*cos(km*alpha);        %左转向中心差量
                    centerl(1)=pp{1,1}(1)+dirtl(1); centerl(2)=pp{1,1}(2)+dirtl(2); %左转向中心坐标
                    %dor=norm(obstacle(i,:) - centerr);                          %障碍物到右转向中心的距离
                    %dor=norm(obstacle(i,:) - centerl);                          %障碍物到左转向中心的距离
                    dirtor(1)=obstacle(i41l)-pp{1,1}(1); dirtor(2)=obstacle(2)-pp{1,1}(2); %障碍物到机器人的坐标差
                    disor=(dirtr(1)-dirtor(1))^2+(dirtr(2)-dirtor(2))^2; %障碍物到右转向中心距离（平方）
                    disol=(dirtl(1)-dirtor(1))^2+(dirtl(2)-dirtor(2))^2; %障碍物到左转向中心距离（平方）
                    if 0<=km&&km<36
                        phib=km+36; %初始极限角度=运动方向的反方向
                        phil=phib;  %初始左极限角度
                        phir=phib;  %初始右极限角度
                        beta = caculatebeta(pp{1,1},obstacle(i41l,:));
                        k = round(beta/alpha); %障碍物所在的扇区
                        if km<=k&&km<phil  %障碍物在左半边区域
                            if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                                phil=k;
                                i5=phil;
                                while (phil<=i5&&i5<=phib)
                                    mag1l(i5)=max(mag1l);
                                    i5=i5+1;
                                end
                            end
                        else
                        %if (0<=k<km|phir<=k<=n) %障碍物在右半边区域
                            if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                                phir=k;
                                if phir<=k&&k<=n
                                    i6=phib;
                                    while (phib<=i6&&i6<=phir)
                                        mag1l(i6)=max(mag1l);
                                        i6=i6+1;
                                    end
                                else
                                %if 0<=k<=km
                                    i7=phib;
                                    while (phib<=i7&&i7<=72)
                                        mag1l(i7)=max(mag1l);
                                        i7=i7+1;
                                    end
                                    i8=1;
                                    while (0<=i8&&i8<=phir)
                                        mag1l(i8)=max(mag1l);
                                        i8=i8+1;
                                    end
                                end
                            end
                        end
                    else
                    %if 36<=km<=72
                       phib=km-36; %初始极限角度=运动方向的反方向 
                       phil=phib;  %初始左极限角度
                       phir=phib;  %初始右极限角度
                       beta = caculatebeta(pp{1,1},obstacle(i41l,:));
                       k = round(beta/alpha); %障碍物所在的扇区
                       if k~=0
                       else
                           k=1;
                       end
                       if phir<=k&&k<km  %障碍物在右半边区域
                          if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                             phir=k;
                             i9=phib;
                             while (phib<=i9&&i9<=phir)
                                   mag1l(i9)=max(mag1l);
                                   i9=i9+1;
                             end
                          end
                       else
                       %if (km<=k<72|0<=k<=phil) %障碍物在左半边区域
                           if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                              phil=k;
                                if 0<=k&&k<=phib
                                    i10=phil;
                                    while (phil<=i10&&i10<=phib)
                                        mag1l(i10)=max(mag1l);
                                        i10=i10+1;
                                    end
                                else
                                %if km<=k<=72
                                    i11=1;
                                    while (0<=i11&&i11<=phib)
                                        mag1l(i11)=max(mag1l);
                                        i11=i11+1;
                                    end
                                    i12=1;
                                    while (phil<=i12&&i12<=72)
                                        mag1l(i12)=max(mag1l);
                                        i12=i12+1;
                                    end
                                end
                           end
                       end  
                    end
                    i41l=i41l+1;
            end
        
            his1l=mag1l;      %现在 his 是一个含72个元素的向量--各扇区极障碍物密度
            % 第一个投影点：选取一组最佳备选方向
            i1=1; %自适应阈值的循环次数
            kb=cell(1,blcs);
            howth=[];
            while (i1<=blcs)   % 自适应阈值的while循环变量：i1 i1取9的时候 会在某一点停下 i1取15的时候 完成避障！说明自适应阈值生效！！！
                %kb2=zeros(9,1);
                %howth2=zeros(9,1);
                Dt=norm(pp{1,1}-endpoint);
                Dth(i1)=Dthmax-i1*dirtD;
                c=[];
                if  Dth(i1)<Dt
                    threshold(i1)=C^2*Iij(Dth(i1));
                    j=1;q=1;
                    
                    while (q<=n)       
                        %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                        if(his1l(q)< threshold(i1))
                            kr=q;                        % 找到了波谷的左端
                            while(q<=n && his1l(q)< threshold(i1))   %这一小段找到了波谷的右端
                                kl=q;
                                q=q+1;
                            end

                            if(kl-kr > smax)                  % 宽波谷
                                c   = [c round(kl - smax/2)];  % 朝向左侧
                                c   = [c round(kr + smax/2)];  % 朝向右侧
                                %j=j+2;
                                if(kt1l >= kr && kt1l <= kl)
                                    c  = [c kt1l];                % straight at look ahead
                                    %j=j+1;
                                end
                             elseif(kl-kr > smax/5)           % 窄波谷
                                c  = [c round((kr+kl)/2-2.5)];
                                %j=j+1;
                             end

                        else
                            q=q+1;                            % his(q)不为0，直接下一个

                        end                                   % 退出if选择，再次进入while条件循环
                    end                                       % 退出while循环

                    % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                    numb=length(c);
                    temp2=howmuchs(Dth(i1),Dthmax,numb,c,kt1l);
                end 

                %temp1=howmuch(Dth(i1),fk,kt,Dthmax);   
                kb{1,i1}=c;
                howth=[howth temp2];       %存储阈值综合代价    
                i1=i1+1;
            end
            ftth=find(howth==min(howth));
            kbbest1l=kb{1,ftth(1)};   %此时，获得一个最佳阈值下的若干备选方向
            %第一个投影点：能否分左右
            
            %情况一：
            %if  0<=km<=round((ds/radius)/alpha)||72-round((ds/radius)/alpha)<=km<=72 
            if 0<=km&&km<=round((ds/radius)/alpha)  %锐角情况
               lml=km+round((ds/radius)/alpha); %左边界
               lmr=km-round((ds/radius)/alpha)+72; %右边界
               numb1=length(kbbest1l); 
               phib=km+36; %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j1=1:numb1
                   if km<=kbbest1l(j1)&&kbbest1l(j1)<=lml  %左边方向可达
                      tempkl=[tempkl kbbest1l(j1)];
                   %end
                   elseif lml<=kbbest1l(j1)&&kbbest1l(j1)<=phib %左边方向不可达
                      tempkl=[tempkl lml];
                   %end
                   elseif phib<=kbbest1l(j1)&&kbbest1l(j1)<=lmr %右边方向不可达
                      tempkr=[tempkr lmr];
                   %end
                   elseif lmr<=kbbest1l(j1)&&kbbest1l(j1)<=72  %右边方向可达 状态一
                      tempkr=[tempkr kbbest1l(j1)];
                   %end
                   else
                   %if 0<=kbbest1l(j1)<=km  %右边方向可达 状态二   
                      tempkr=[tempkr kbbest1l(j1)]; 
                   end
               end         
            
            elseif 72-round((ds/radius)/alpha)<=km&&km<=72  %钝角情况
               lml=km+round((ds/radius)/alpha)-72;  %左边界
               if lml~=0
               else
                  lml=1; 
               end
               lmr=km-round((ds/radius)/alpha);  %右边界
               numb1=length(kbbest1l);
               phib=km-36;  %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j2=1:numb1
                   if phib<=kbbest1l(j2)&&kbbest1l(j2)<=lmr  %右边方向不可达
                      tempkr=[tempkr lmr]; 
                   %end
                   elseif lmr<=kbbest1l(j2)&&kbbest1l(j2)<=km  %右边方向可达
                      tempkr=[tempkr kbbest1l(j2)]; 
                   %end
                   elseif km<=kbbest1l(j2)&&kbbest1l(j2)<=72  %左边方向可达 状态一
                      tempkl=[tempkl kbbest1l(j2)];
                   %end
                   elseif 0<=kbbest1l(j2)&&kbbest1l(j2)<=lml  %左边方向可达 状态二
                      tempkl=[tempkl kbbest1l(j2)]; 
                   %end
                   else
                   %if lml<=kbbest1l(j2)<=phib  %左边方向不可达
                      tempkl=[tempkl lml]; 
                   end
               end
            
            %end
            %情况二
            elseif  round((ds/radius)/alpha)<=km&&km<=36 
                lml=km+round((ds/radius)/alpha); %左边界
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1l); 
                phib=km+36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j3=1:numb1
                    if km<=kbbest1l(j3)&&kbbest1l(j3)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1l(j3)]; 
                    %end
                    elseif lml<=kbbest1l(j3)&&kbbest1l(j3)<=phib  %左边方向不可达
                       tempkl=[tempkl lml]; 
                    %end
                    elseif km<=kbbest1l(j3)&&kbbest1l(j3)<=lmr  %右边方向可达
                       tempkr=[tempkr kbbest1l(j3)]; 
                    else
                    tempkr=[tempkr lmr];  %右边方向不可达
                    end
                end 
            %end
            else
            %情况三
            %if  36<=km<=72-round((ds/radius)/alpha)
                lml=km+round((ds/radius)/alpha); %左边界
                if lml~=0
                else
                   lml=1; 
                end
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1l); 
                phib=km-36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j4=1:numb1
                    if lmr<=kbbest1l(j4)&&kbbest1l(j4)<=km  %右边方向可达
                       tempkr=[tempkr kbbest1l(j4)]; 
                    %end
                    elseif phib<=kbbest1l(j4)&&kbbest1l(j4)<=lmr  %右边方向不可达
                       tempkr=[tempkr lmr]; 
                    %end
                    elseif km<=kbbest1l(j4)&&kbbest1l(j4)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1l(j4)]; 
                    else
                    tempkl=[tempkl lml];  %左边方向不可达 
                    end
                end
            end
            
            geshul=length(tempkl);
            geshur=length(tempkr);
            %第一个投影点：能分左右
            if  geshul>=1&&geshur>=1  
                lc=pf1lbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                       if or1l~=73 
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l));
                       else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),inf);   
                       end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),inf,his1l(or1l));
                    end
                end
                ft1ll=find(how1l==min(how1l));
                pf1llbest=filt(g1l(ft1ll),kt1l);  %找到左边最佳方向 
                g1r=zeros(geshur,1);how1r=zeros(geshur,1);  %右边前进方向
                for i15=1:geshur
                    g1r(i15)=tempkr(i15);
                    order1r=g1r(i15);
                    ol1r=g1r(i15)-1;
                    or1r=g1r(i15)+1;
                    if ol1r~=0
                       if or1r~=73
                       how1r(i15)=howmany(g1r(i15),kt1l,km,lc,his1l(order1r),his1l(ol1r),his1l(or1r));
                       else
                       how1r(i15)=howmany(g1r(i15),kt1l,km,lc,his1l(order1r),his1l(ol1r),inf);    
                       end
                    else
                       how1r(i15)=howmany(g1r(i15),kt1l,km,lc,his1l(order1r),inf,his1l(or1r));
                    end
                end
                ft1lr=find(how1r==min(how1r));
                pf1lrbest=filt(g1r(ft1lr),kt1l);  %找到右边最佳方向
                %前进（投影）
                pp{1,3}=pp{1,1}+[v_car*time*cos(pf1llbest*alpha),v_car*time*sin(pf1llbest*alpha)];
                pp{1,4}=pp{1,1}+[v_car*time*cos(pf1lrbest*alpha),v_car*time*sin(pf1lrbest*alpha)];
                ol1l=pf1llbest-1;
                or1l=pf1llbest+1;
                order1l=pf1llbest;
                if ol1l~=0
                   if or1l~=73 
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l),ng)+gc{1,1};
                   else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),inf,ng)+gc{1,1};    
                   end
                else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),inf,his1l(or1l),ng)+gc{1,1}; 
                end
                ol1r=pf1lrbest-1;
                or1r=pf1lrbest+1;
                order1r=pf1lrbest;
                if ol1r~=0
                   if or1r~=73 
                   gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1r),his1l(ol1r),his1l(or1r),ng)+gc{1,1};
                   else
                   gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1r),his1l(ol1r),inf,ng)+gc{1,1};    
                   end
                else
                   gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1r),inf,his1l(or1r),ng)+gc{1,1}; 
                end
                kt1ll=round(caculatebeta(pp{1,3},endpoint)/alpha);  %新节点的目标方向 左
                kt1lr=round(caculatebeta(pp{1,4},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,3}=heu(kt1ll,pf1llbest,pf1llbest,ng);
                hc{1,4}=heu(kt1lr,pf1lrbest,pf1lrbest,ng);
                fc{1,3}=gc{1,3}+hc{1,3};
                fc{1,4}=gc{1,4}+hc{1,4};
                pfcun(1,1)=pf1llbest;
                pfcun(1,2)=pf1lrbest;
            %end
            %第一个投影点：不能分左右 只有一个方向
            elseif  geshul>=1&&geshur==0  %该方向是左
                lc=pf1lbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                       if or1l~=73 
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l));
                       else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),inf);    
                       end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),inf,his1l(or1l)); 
                    end
                end
                ft1ll=find(how1l==min(how1l));
                pf1llbest=filt(g1l(ft1ll),kt1l);  %找到左边最佳方向 
                pp{1,3}=pp{1,1}+[v_car*time*cos(pf1llbest*alpha),v_car*time*sin(pf1llbest*alpha)];
                ol1l=pf1llbest-1;
                or1l=pf1llbest+1;
                order1l=pf1llbest;
                if ol1l~=0
                   if or1l~=73 
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l),ng)+gc{1,1};
                   else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),inf,ng)+gc{1,1};   
                   end
                else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),inf,his1l(or1l),ng)+gc{1,1}; 
                end
                kt1ll=round(caculatebeta(pp{1,3},endpoint)/alpha);  %新节点的目标方向 左
                hc{1,3}=heu(kt1ll,pf1llbest,pf1llbest,ng);
                fc{1,3}=gc{1,3}+hc{1,3};
                fc{1,4}=inf;
                pfcun(1,1)=pf1llbest;
            %end
            elseif  geshur>=1&&geshul==0  %该方向是右
                lc=pf1lbest;
                g1l=zeros(geshur,1);how1l=zeros(geshur,1);  %右边前进方向 
                for i14=1:geshur
                    g1l(i14)=tempkr(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                        if or1l~=73
                        how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l));
                        else
                        how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),inf);    
                        end
                    else
                        how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),inf,his1l(or1l)); 
                    end
                end
                ft1lr=find(how1l==min(how1l));
                pf1lrbest=filt(g1l(ft1lr),kt1l);  %找到右边最佳方向 
                pp{1,4}=pp{1,1}+[v_car*time*cos(pf1lrbest*alpha),v_car*time*sin(pf1lrbest*alpha)];
                ol1l=pf1lrbest-1;
                or1l=pf1lrbest+1;
                order1l=pf1lrbest;
                if ol1l~=0
                    if or1l~=73
                    gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l),ng)+gc{1,1};
                    else
                    gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),inf,ng)+gc{1,1};    
                    end
                else
                    gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1l),inf,his1l(or1l),ng)+gc{1,1};
                end
                kt1lr=round(caculatebeta(pp{1,4},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,4}=heu(kt1lr,pf1lrbest,pf1lrbest,ng);
                fc{1,4}=gc{1,4}+hc{1,4};
                fc{1,3}=inf;
                pfcun(1,2)=pf1lrbest;
            %end
            %第一个投影点：不能分左右 无方向
            else
            %if  geshul==geshur==0
                fc{1,3}=inf;
                fc{1,4}=inf;
            end
            % 建立第二个投影点的极直方图
            i3=1;mag1r = zeros(n,1);his1r=zeros(n,1);
            while (i3<=length(obstacle))  
            
            %%%%%%%%%%% 下面一段程序得到机器人360度范围视野内的障碍物分布值 72个扇区的极障碍物密度  
            
                d1r = norm(obstacle(i3,:) - pp{1,2}); % 障碍物栅格与机器人之间距离
                if (d1r<dmax)
                    beta1r = caculatebeta(pp{1,2},obstacle(i3,:));  % 障碍物栅格向量的方向
                    rangle1r=asin(rsafe/d1r);        % 扩大的角度
                    k1r = round(beta1r/alpha);       % 逆时针数，第k个扇区区域
                    if(k1r == 0)
                        k1r = 1;
                    end
                    % 更新后的极坐标直方图的h值
                    if((5*k1r>rad2deg(beta1r)-rad2deg(rangle1r))&&(5*k1r<rad2deg(beta1r)+rad2deg(rangle1r)))  
                        h1r(k1r)=1;
                    else
                        h1r(k1r)=0;
                    end
                    i3=i3+1;

                    m1r = C^2*Iij(d1r);   % 障碍物栅格的向量幅值，与VFH计算方法不同
                    mag1r(k1r)=max(mag1r(k1r),m1r.*h1r(k1r));   % mag为zeros(n,1)，mag的第k个元素为m
                    i3=i3+1;
                else
                    i3=i3+1;
                end 
            end
            %第二个投影点：用VFH+排除一些扇区
            i41r=1; %应用VFH+算法，考虑运动半径因素 排除更多的扇区
            km=pf1rbest;
            while (i41r<=length(obstacle))

                %%%%%%%%%% 考虑转向半径因素
                    
                    dirtr(1)=radius*sin(km*alpha);   dirtr(2)=radius*cos(km*alpha);         %右转向中心差量
                    centerr(1)=pp{1,2}(1)+dirtr(1); centerr(2)=pp{1,2}(2)+dirtr(2); %右转向中心坐标
                    dirtl(1)=-radius*sin(km*alpha);  dirtl(2)=-radius*cos(km*alpha);        %左转向中心差量
                    centerl(1)=pp{1,2}(1)+dirtl(1); centerl(2)=pp{1,2}(2)+dirtl(2); %左转向中心坐标
                    %dor=norm(obstacle(i,:) - centerr);                          %障碍物到右转向中心的距离
                    %dor=norm(obstacle(i,:) - centerl);                          %障碍物到左转向中心的距离
                    dirtor(1)=obstacle(i41r)-pp{1,2}(1); dirtor(2)=obstacle(2)-pp{1,2}(2); %障碍物到机器人的坐标差
                    disor=(dirtr(1)-dirtor(1))^2+(dirtr(2)-dirtor(2))^2; %障碍物到右转向中心距离（平方）
                    disol=(dirtl(1)-dirtor(1))^2+(dirtl(2)-dirtor(2))^2; %障碍物到左转向中心距离（平方）
                    if 0<=km&&km<36
                        phib=km+36; %初始极限角度=运动方向的反方向
                        phil=phib;  %初始左极限角度
                        phir=phib;  %初始右极限角度
                        beta = caculatebeta(pp{1,2},obstacle(i41r,:));
                        k = round(beta/alpha); %障碍物所在的扇区
                        if km<=k&&k<phil  %障碍物在左半边区域
                            if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                                phil=k;
                                i5=phil;
                                while (phil<=i5&&i5<=phib)
                                    mag1r(i5)=max(mag1r);
                                    i5=i5+1;
                                end
                            end
                        else
                        %if (0<=k<km|phir<=k<=n) %障碍物在右半边区域
                            if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                                phir=k;
                                if phir<=k&&k<=n
                                    i6=phib;
                                    while (phib<=i6&&i6<=phir)
                                        mag1r(i6)=max(mag1r);
                                        i6=i6+1;
                                    end
                                else
                                %if 0<=k<=km
                                    i7=phib;
                                    while (phib<=i7&&i7<=72)
                                        mag1r(i7)=max(mag1r);
                                        i7=i7+1;
                                    end
                                    i8=1;
                                    while (0<=i8&&i8<=phir)
                                        mag1r(i8)=max(mag1r);
                                        i8=i8+1;
                                    end
                                end
                            end
                        end
                    else
                    %if 36<=km<=72
                       phib=km-36; %初始极限角度=运动方向的反方向 
                       phil=phib;  %初始左极限角度
                       phir=phib;  %初始右极限角度
                       beta = caculatebeta(pp{1,2},obstacle(i41r,:));
                       k = round(beta/alpha); %障碍物所在的扇区
                       if k~=0
                       else
                          k=1;
                       end
                       if phir<=k&&k<km  %障碍物在右半边区域
                          if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                             phir=k;
                             i9=phib;
                             while (phib<=i9&&i9<=phir)
                                   mag1r(i9)=max(mag1r);
                                   i9=i9+1;
                             end
                          end
                       else
                       %if (km<=k<72|0<=k<=phil) %障碍物在左半边区域
                           if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                              phil=k;
                                if 0<=k&&k<=phib
                                    i10=phil;
                                    while (phil<=i10&&i10<=phib)
                                        i10
                                        mag1r(i10)=max(mag1r);
                                        i10=i10+1;
                                    end
                                else
                                %if km<=k<=72
                                    i11=1;
                                    while (0<=i11&&i11<=phib)
                                        mag1r(i11)=max(mag1r);
                                        i11=i11+1;
                                    end
                                    i12=1;
                                    while (phil<=i12&&i12<=72)
                                        mag1r(i12)=max(mag1r);
                                        i12=i12+1;
                                    end
                                end
                           end
                       end  
                    end
                    i41r=i41r+1;
            end
        
            his1r=mag1r;      %现在 his 是一个含72个元素的向量--各扇区极障碍物密度
            %第二个投影点：选取一组最佳备选方向
            i1=1; %自适应阈值的循环次数
            kb=cell(1,blcs);
            howth=[];
            while (i1<=blcs)   % 自适应阈值的while循环变量：i1 i1取9的时候 会在某一点停下 i1取15的时候 完成避障！说明自适应阈值生效！！！
                %kb2=zeros(9,1);
                %howth2=zeros(9,1);
                Dt=norm(pp{1,2}-endpoint);
                Dth(i1)=Dthmax-i1*dirtD;
                c=[];
                if  Dth(i1)<Dt
                    threshold(i1)=C^2*Iij(Dth(i1));
                    j=1;q=1;
                    
                    while (q<=n)       
                        %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                        if(his1r(q)< threshold(i1))
                            kr=q;                        % 找到了波谷的左端
                            while(q<=n && his1r(q)< threshold(i1))   %这一小段找到了波谷的右端
                                kl=q;
                                q=q+1;
                            end

                            if(kl-kr > smax)                  % 宽波谷
                                c   = [c round(kl - smax/2)];  % 朝向左侧
                                c   = [c round(kr + smax/2)];  % 朝向右侧
                                %j=j+2;
                                if(kt1r >= kr && kt1r <= kl)
                                    c  = [c kt1r];                % straight at look ahead
                                    %j=j+1;
                                end
                             elseif(kl-kr > smax/5)           % 窄波谷
                                c  = [c round((kr+kl)/2-2.5)];
                                %j=j+1;
                             end

                        else
                            q=q+1;                            % his(q)不为0，直接下一个

                        end                                   % 退出if选择，再次进入while条件循环
                    end                                       % 退出while循环

                    % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                    numb=length(c);
                    temp2=howmuchs(Dth(i1),Dthmax,numb,c,kt1r);
                end 

                %temp1=howmuch(Dth(i1),fk,kt,Dthmax);   
                kb{1,i1}=c;
                howth=[howth temp2];       %存储阈值综合代价    
                i1=i1+1;
            end
            ftth=find(howth==min(howth));
            kbbest1r=kb{1,ftth(1)};   %此时，获得一个最佳阈值下的若干备选方向
            %第二个投影点：能否分左右
            %情况一：
            %if  0<=km<=round((ds/radius)/alpha)||72-round((ds/radius)/alpha)<=km<=72 
            if 0<=km&&km<=round((ds/radius)/alpha)  %锐角情况
               lml=km+round((ds/radius)/alpha); %左边界
               lmr=km-round((ds/radius)/alpha)+72; %右边界
               numb1=length(kbbest1r); 
               phib=km+36; %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j1=1:numb1
                   if km<=kbbest1r(j1)&&kbbest1r(j1)<=lml  %左边方向可达
                      tempkl=[tempkl kbbest1r(j1)];
                   %end
                   elseif lml<=kbbest1r(j1)&&kbbest1r(j1)<=phib %左边方向不可达
                      tempkl=[tempkl lml];
                   %end
                   elseif phib<=kbbest1r(j1)&&kbbest1r(j1)<=lmr %右边方向不可达
                      tempkr=[tempkr lmr];
                   %end
                   elseif lmr<=kbbest1r(j1)&&kbbest1r(j1)<=72  %右边方向可达 状态一
                      tempkr=[tempkr kbbest1r(j1)];
                   %end
                   else
                   %if 0<=kbbest1r(j1)<=km  %右边方向可达 状态二   
                      tempkr=[tempkr kbbest1r(j1)]; 
                   end
               end         
            
            elseif 72-round((ds/radius)/alpha)<=km&&km<=72  %钝角情况
               lml=km+round((ds/radius)/alpha)-72;  %左边界
               if lml~=0
               else
                  lml=1; 
               end
               lmr=km-round((ds/radius)/alpha);  %右边界
               numb1=length(kbbest1r);
               phib=km-36;  %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j2=1:numb1
                   if phib<=kbbest1r(j2)&&kbbest1r(j2)<=lmr  %右边方向不可达
                      tempkr=[tempkr lmr]; 
                   %end
                   elseif lmr<=kbbest1r(j2)&&kbbest1r(j2)<=km  %右边方向可达
                      tempkr=[tempkr kbbest1r(j2)]; 
                   %end
                   elseif km<=kbbest1r(j2)&&kbbest1r(j2)<=72  %左边方向可达 状态一
                      tempkl=[tempkl kbbest1r(j2)];
                   %end
                   elseif 0<=kbbest1r(j2)&&kbbest1r(j2)<=lml  %左边方向可达 状态二
                      tempkl=[tempkl kbbest1r(j2)]; 
                   %end
                   else
                   %if lml<=kbbest1r(j2)<=phib  %左边方向不可达
                      tempkl=[tempkl lml]; 
                   end
               end
            %end
            %end
            %情况二
            elseif  round((ds/radius)/alpha)<=km&&km<=36 
                lml=km+round((ds/radius)/alpha); %左边界
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1r); 
                phib=km+36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j3=1:numb1
                    if km<=kbbest1r(j3)&&kbbest1r(j3)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1r(j3)]; 
                    %end
                    elseif lml<=kbbest1r(j3)&&kbbest1r(j3)<=phib  %左边方向不可达
                       tempkl=[tempkl lml]; 
                    %end
                    elseif km<=kbbest1r(j3)&&kbbest1r(j3)<=lmr  %右边方向可达
                       tempkr=[tempkr kbbest1r(j3)]; 
                    else
                    tempkr=[tempkr lmr];  %右边方向不可达
                    end
                end 
            %end
            %情况三
            else
            %if  36<=km<=72-round((ds/radius)/alpha)
                lml=km+round((ds/radius)/alpha); %左边界
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1r); 
                phib=km-36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j4=1:numb1
                    if lmr<=kbbest1r(j4)&&kbbest1r(j4)<=km  %右边方向可达
                       tempkr=[tempkr kbbest1r(j4)]; 
                    %end
                    elseif phib<=kbbest1r(j4)&&kbbest1r(j4)<=lmr  %右边方向不可达
                       tempkr=[tempkr lmr]; 
                    %end
                    elseif km<=kbbest1r(j4)&&kbbest1r(j4)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1r(j4)]; 
                    else
                    tempkl=[tempkl lml];  %左边方向不可达
                    end
                end
            end
            
            geshul=length(tempkl);
            geshur=length(tempkr);
            %第二个投影点：能分左右
            if  geshul>=1&&geshur>=1  
                lc=pf1rbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                        if or1l~=73
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l));
                        else
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),inf);    
                        end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),inf,his1r(or1l)); 
                    end
                end
                ft1rl=find(how1l==min(how1l));
                pf1rlbest=filt(g1l(ft1rl),kt1r);  %找到左边最佳方向 
                g1r=zeros(geshur,1);how1r=zeros(geshur,1);  %右边前进方向
                for i15=1:geshur
                    g1r(i15)=tempkr(i15);
                    order1r=g1r(i15);
                    ol1r=g1r(i15)-1;
                    or1r=g1r(i15)+1;
                    if ol1r~=0
                       if or1r~=73 
                       how1r(i15)=howmany(g1r(i15),kt1r,km,lc,his1r(order1r),his1r(ol1r),his1r(or1r));
                       else
                       how1r(i15)=howmany(g1r(i15),kt1r,km,lc,his1r(order1r),his1r(ol1r),inf);    
                       end
                    else
                       how1r(i15)=howmany(g1r(i15),kt1r,km,lc,his1r(order1r),inf,his1r(or1r)); 
                    end
                end
                ft1rr=find(how1r==min(how1r));
                pf1rrbest=filt(g1r(ft1rr),kt1r);  %找到右边最佳方向
                pp{1,5}=pp{1,2}+[v_car*time*cos(pf1rlbest*alpha),v_car*time*sin(pf1rlbest*alpha)];
                pp{1,6}=pp{1,2}+[v_car*time*cos(pf1rrbest*alpha),v_car*time*sin(pf1rrbest*alpha)];
                ol1l=pf1rlbest-1;
                or1l=pf1rlbest+1;
                order1l=pf1rlbest;
                if ol1l~=0
                   if or1l~=73
                   gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l),ng)+gc{1,2};
                   else
                   gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),inf,ng)+gc{1,2};    
                   end
                else
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),inf,his1r(or1l),ng)+gc{1,2}; 
                end
                ol1r=pf1rrbest-1;
                or1r=pf1rrbest+1;
                order1r=pf1rrbest;
                if ol1r~=0
                   if or1r~=73
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1r),his1r(ol1r),his1r(or1r),ng)+gc{1,2};
                   else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1r),his1r(ol1r),inf,ng)+gc{1,2};    
                   end
                else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1r),inf,his1r(or1r),ng)+gc{1,2}; 
                end
                kt1rl=round(caculatebeta(pp{1,5},endpoint)/alpha);  %新节点的目标方向 左
                kt1rr=round(caculatebeta(pp{1,6},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,5}=heu(kt1rl,pf1rlbest,pf1rlbest,ng);
                hc{1,6}=heu(kt1rr,pf1rrbest,pf1rrbest,ng);
                fc{1,5}=gc{1,5}+hc{1,5};
                fc{1,6}=gc{1,6}+hc{1,6};
                pfcun(1,3)=pf1rlbest;
                pfcun(1,4)=pf1rrbest;
            %end
            %第二个投影点：不能分左右 只有一个方向
            elseif  geshul>=1&&geshur==0  %该方向是左
                lc=pf1rbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                tempkl
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                        if or1l~=73 
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l));
                        else
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),inf);    
                        end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),inf,his1r(or1l)); 
                    end
                end
                ft1rl=find(how1l==min(how1l));
                pf1rlbest=filt(g1l(ft1rl),kt1r);  %找到左边最佳方向
                pp{1,5}=pp{1,2}+[v_car*time*cos(pf1rlbest*alpha),v_car*time*sin(pf1rlbest*alpha)];
                ol1l=pf1rlbest-1;
                or1l=pf1rlbest+1;
                order1l=pf1rlbest;
                if ol1l~=0
                    if or1l~=73 
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l),ng)+gc{1,2};
                    else
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),inf,ng)+gc{1,2};
                    end
                else
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),inf,his1r(or1l),ng)+gc{1,2};
                end
                kt1rl=round(caculatebeta(pp{1,5},endpoint)/alpha);  %新节点的目标方向 左
                hc{1,5}=heu(kt1rl,pf1rlbest,pf1rlbest,ng);
                fc{1,5}=gc{1,5}+hc{1,5};
                fc{1,6}=inf;
                pfcun(1,3)=pf1rlbest;
            %end
            elseif  geshur>=1&&geshul==0  %该方向是右
                lc=pf1rbest;
                g1l=zeros(geshur,1);how1l=zeros(geshur,1);  %右边前进方向 
                for i14=1:geshur
                    g1l(i14)=tempkr(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                       if or1l~=73 
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l));
                       else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),inf);
                       end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),inf,his1r(or1l)); 
                    end
                end
                ft1rr=find(how1l==min(how1l));
                pf1rrbest=filt(g1l(ft1rr),kt1r);  %找到右边最佳方向
                pp{1,6}=pp{1,2}+[v_car*time*cos(pf1rrbest*alpha),v_car*time*sin(pf1rrbest*alpha)];
                ol1l=pf1rrbest-1;
                or1l=pf1rrbest+1;
                order1l=pf1rrbest;
                if ol1l~=0
                   if or1l~=73  
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l),ng)+gc{1,2};
                   else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),inf,ng)+gc{1,2};
                   end
                else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1l),inf,his1r(or1l),ng)+gc{1,2}; 
                end
                kt1rr=round(caculatebeta(pp{1,6},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,6}=heu(kt1rr,pf1rrbest,pf1rrbest,ng);
                fc{1,6}=gc{1,6}+hc{1,6};
                fc{1,5}=inf;
                pfcun(1,4)=pf1rrbest;
            else
            %第二个投影点：不能分左右 无方向
            %if  geshul==geshur==0
                fc{1,5}=inf;
                fc{1,6}=inf;
            end
            pd=[fc{1,3},fc{1,4},fc{1,5},fc{1,6}]  %判定值
            ft=find(pd==min(pd)) %找出几个节点fc值最小的
            dcyb=pfcun(1,ft);
            %lc=dc;
            
            if  length(ft)==1
                lc=dcyb;
                dc=dcyb;
                robot=pp{1,ft+2};  %VFH*算法得到的最终位置并赋予机器人
            elseif  length(ft)==2 %注意 ft 和 dcyb 此时是2行1列的矩阵（2×1）
                geshu=length(dcyb);
                if dcyb(1)==dcyb(geshu)
                   dc=dcyb(1); 
                   lc=dc;
                   robot=pp{1,ft(1)+2};
                else
                   kt_1=round(caculatebeta(pp{1,ft(1)+2},endpoint)/alpha);
                   kt_2=round(caculatebeta(pp{1,ft(2)+2},endpoint)/alpha);
                   g_=zeros(geshu,1);how_=zeros(geshu,1);xushu=zeros(geshu,1);
                   kt_=[kt_1 kt_2];
                   for i14=1:geshu
                       g_(i14)=dcyb(i14);
                       how_(i14)=howmanyss(g_(i14),kt_(i14));
                       xushu(i14)=ft(i14);
                   end
                   if how_(1)~=how_(2)
                       ft_=find(how_==min(how_));
                       dc=g_(ft_);
                       lc=dc;
                       robot=pp{1,xushu(ft_)};
                   else
                       g__=zeros(geshu,1);how__=zeros(geshu,1);xushu_=zeros(geshu,1);
                       for i14=1:geshu
                           g__(i14)=dcyb(i14);
                           how__(i14)=dif(g__(i14),kt_(i14));
                           xushu_(i14)=ft(i14);
                       end
                       ft__=find(how__==min(how__));
                       dc=g__(ft__);
                       lc=dc;
                       robot=pp{1,xushu(ft__)};
                   end
                 end
            end

            %当前运动方向
            %上次选择方向
        else   %如果不进入VFH*算法 那就使用VFH+算法判定
            if  norm(robot-ref)==0
                lc=kt;
            else
                lc=lc;
            end
            
            i1=1; %自适应阈值的循环次数
            kb=[];%储存最佳方向集合
            howth=[];%考虑阈值与该阈值下方向的代价集合
            
            % 自适应阈值开始！
            
            while (i1<=blcs)
                Dt=norm(robot-endpoint);
                Dth(i1)=Dthmax-i1*dirtD;
                c=[];
                if  Dth(i1)<Dt
                    threshold(i1)=C^2*Iij(Dth(i1));
                    j=1;q=1;

                    while (q<=n)       
                        %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                        if(his(q)< threshold(i1))
                            kr=q;                        % 找到了波谷的左端
                            while(q<=n && his(q)< threshold(i1))   %这一小段找到了波谷的右端
                                kl=q;
                                q=q+1;
                            end

                            if(kl-kr > smax)                  % 宽波谷
                                c   =  [c round(kl - smax/2)];  % 朝向左侧
                                c   =  [c round(kr + smax/2)];  % 朝向右侧
                                j=j+2;
                                if(kt >= kr && kt <= kl)
                                    c  = [c kt];                % straight at look ahead
                                    j=j+1;
                                end
                            elseif(kl-kr > smax/5)           % 窄波谷
                                c   =  [c round((kr+kl)/2-2.5)];
                                j=j+1;
                            end

                        else
                            q=q+1;                            % his(q)不为0，直接下一个

                        end                                   % 退出if选择，再次进入while条件循环
                    end                                       % 退出while循环

                    % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                    % 开始筛选最优方向
                    if norm(robot-ref)==0                            
                       g=zeros(j-1,1);how=zeros(j-1,1);
                       for i2=1:j-1
                           g(i2)=c(i2);     %g中不含目标向量
                           order=g(i2);
                           ol=g(i2)-1;
                           or=g(i2)+1;
                           dc=kt;           %由于机器人还没动，所以目标方向就是当前运动方向
                           lc=kt;           %由于机器人还没动，所以目标方向就是上次选择方向
                           if ol~=0   %防止出现第0或第73
                              if or~=73
                               how(i2)=howmany(g(i2),kt,dc,lc,his(order),his(ol),his(or)); %代价函数计算最优方向 how为代价 元素个数与 g 是相同的
                              else
                               how(i2)=howmany(g(i2),kt,dc,lc,his(order),his(ol),inf);   
                              end
                           else
                              how(i2)=howmany(g(i2),kt,dc,lc,his(order),inf,his(or)); 
                           end 
                       end                                                             
                       ft=find(how==min(how));
                       fk=g(ft);
                       kb=[kb fk];  % 当前阈值下的最佳备选方向
                    else
                       g=zeros(j-1,1);how=zeros(j-1,1);
                       for i3=1:j-1
                           g(i3)=c(i3);
                           order=g(i3);
                           ol=g(i3)-1;
                           or=g(i3)+1;
                           if ol~=0   %防止出现第0或第73
                              if or~=73
                               how(i3)=howmany(g(i3),kt,dc,lc,his(order),his(ol),his(or));
                              else
                               how(i3)=howmany(g(i3),kt,dc,lc,his(order),his(ol),inf);   
                              end
                           else
                              how(i3)=howmany(g(i3),kt,dc,lc,his(order),inf,his(or)); 
                           end
                       end
                       ft=find(how==min(how));
                       fk=g(ft);
                       %一个阈值下也许会出现多个最佳方向 得筛选下
                       cd=length(fk); %最优方向的个数
                       if cd==1
                          fk=fk;  % 当前阈值下的最佳备选方向 
                       else 
                          g_=zeros(cd,1);how_=zeros(cd,1); 
                          for i4=1:cd
                              g_(i4)=fk(i4);
                              how_(i4)=dif(g_(i4),kt);
                          end
                          ft_=find(how_==min(how_));
                          fk=g_(ft_);  % 当前阈值下的最佳备选方向
                       end
                       kb=[kb fk];  % 当前阈值下的最佳备选方向
                    end
                    
                    temp1=howmuch(Dth(i1),fk,kt,Dthmax); %计算 阈值与该阈值下最佳方向的综合代价
                    howth=[howth temp1]; %存储综合代价
                end
                i1=i1+1;
            end
            ft=find(howth==min(howth));
            fbestyb=kb(ft);  %VFH+算法得到的最好方向
            % 防止有多个最优方向
            if  length(ft)==1
                dc=fbestyb;       % 当前的运动方向
                lc=dc;       % 上一次选择的方向
                robot=robot+[v_car*time*cos(fbestyb*alpha),v_car*time*sin(fbestyb*alpha)];  %VFH+算法得到的最终位置并赋予机器人
            elseif  length(ft)==2 %注意 ft 和 dcyb 此时是2行1列的矩阵（2×1）
                geshu=length(fbestyb);
                if fbestyb(1,1)==fbestyb(geshu,1)
                   dc=fbestyb(1,1);  % 当前的运动方向
                   lc=dc; % 上一次选择的方向
                   robot=robot+[v_car*time*cos(lc*alpha),v_car*time*sin(lc*alpha)];
                else
                   
                   g_=zeros(geshu,1);how_=zeros(geshu,1);  
                   for i14=1:geshu
                        g_(i14)=fbestyb(i14);
                        how_(i14)=howmanyss(g_(i14),kt);
                   end
                   ft_=find(how_==min(how_));
                   dc=g_(ft_); % 当前的运动方向
                   lc=dc; % 上一次选择的方向
                   robot=robot+[v_car*time*cos(lc*alpha),v_car*time*sin(lc*alpha)];
                end    
            elseif  length(ft)==3
                geshu=length(fbestyb);
                g_=zeros(geshu,1);how_=zeros(geshu,1);  
                   for i14=1:geshu
                        g_(i14)=fbestyb(i14);
                        how_(i14)=howmanyss(g_(i14),kt);
                   end
                   ft_=find(how_==min(how_));
                   dc=g_(ft_); % 当前的运动方向
                   lc=dc; % 上一次选择的方向
                   robot=robot+[v_car*time*cos(lc*alpha),v_car*time*sin(lc*alpha)];
            %else 
            end
        end
        ref=startpoint;
        scatter(robot(1),robot(2),'.r');
        drawnow;
        kt=round(caculatebeta(robot,endpoint)/alpha);  %新的目标方向
        if(kt==0)
            kt=n;
        end
        if(norm(robot-endpoint))>step          % 机器人位置和终点位置差距大于0.1时
        else
            break
        end
        %至此避障规划一次完毕
        
        obstacle([5271:8870],:)=[];    %避障完毕后剔除动态障碍物
        obstacle;
    else %第三阶段（时间段）
        v=0.85; phiv=(2/3)*pi;  %速度大小及方向
        scatter(x0,y0,'.r');
        drawnow;
        x0=x0+v*cos(phiv)*time;  %圆心运动方程   
        y0=y0+v*sin(phiv)*time;  %圆心运动方程   
        x=x0+r*cos(phi);y=y0+r*sin(phi); %圆的方程
        xy=[];
        xy=[xy;x;y];
        xy=xy'; %转置后 得到圆上各点的坐标 
        plot(x,y,'.g');
        hold on
        mid=xy;
        obstacle=cat(1,obstacle,mid);  %动态障碍物加入静态障碍物坐标 cat为矩阵拼接函数 1竖着拼 2横着拼
        
        %%% 现在开始避障路径规划
        if(norm(robot-endpoint))>step          % 机器人位置和终点位置差距大于0.1时
        else
            break
        end
        %首先建立障碍物的极直方图
        i=1;mag = zeros(n,1);his=zeros(n,1);
        % 下面一段程序得到机器人360度范围视野内的障碍物分布值 72个扇区的极障碍物密度
        while (i<=length(obstacle)) 
           d = norm(obstacle(i,:) - robot); % 障碍物栅格与机器人之间距离
            if (d<dmax)
                beta = caculatebeta(robot,obstacle(i,:));  % 障碍物栅格向量的方向
                rangle=asin(rsafe/d);        % 扩大的角度
                k = round(beta/alpha);       % 逆时针数，第k个扇区区域
                if(k == 0)
                    k = 1;
                end
                % 更新后的极坐标直方图的h值
                if((5*k>rad2deg(beta)-rad2deg(rangle))&&(5*k<rad2deg(beta)+rad2deg(rangle)))  
                    h(k)=1;
                else
                    h(k)=0;
                end
                i=i+1;
                m = C^2*Iij(d);   % 障碍物栅格的向量幅值，与VFH计算方法不同
                mag(k)=max(mag(k),m.*h(k));   % mag为zeros(n,1)，mag的第k个元素为m
                i=i+1;
            else
                i=i+1;
            end
        end
        %接着应用VFH+算法，考虑运动半径因素 排除更多的扇区
        i4=1;
        if  norm(robot-ref)==0
            km=kt;
        else
            km=dc;
        end
        k1=0;
        k2=0;       
        alpha;
        while (i4<=length(obstacle))
            % 考虑转向半径因素
            dirtr(1)=radius*sin(km*alpha); 
            dirtr(2)=radius*cos(km*alpha);         %右转向中心差量
            centerr(1)=robot(1)+dirtr(1); centerr(2)=robot(2)+dirtr(2); %右转向中心坐标
            dirtl(1)=-radius*sin(km*alpha);  dirtl(2)=-radius*cos(km*alpha);        %左转向中心差量
            centerl(1)=robot(1)+dirtl(1); centerl(2)=robot(2)+dirtl(2); %左转向中心坐标
            %dor=norm(obstacle(i,:) - centerr);                          %障碍物到右转向中心的距离
            %dor=norm(obstacle(i,:) - centerl);                          %障碍物到左转向中心的距离
            dirtor(1)=obstacle(i4)-robot(1); dirtor(2)=obstacle(2)-robot(2); %障碍物到机器人的坐标差
            disor=(dirtr(1)-dirtor(1))^2+(dirtr(2)-dirtor(2))^2; %障碍物到右转向中心距离（平方）
            disol=(dirtl(1)-dirtor(1))^2+(dirtl(2)-dirtor(2))^2; %障碍物到左转向中心距离（平方）
            if 0<=km&&km<36
                k1=k1+1
                phib=km+36; %初始极限角度=运动方向的反方向
                phil=phib;  %初始左极限角度
                phir=phib;  %初始右极限角度
                beta = caculatebeta(robot,obstacle(i4,:));
                k = round(beta/alpha); %障碍物所在的扇区
                if km<=k&&k<phil  %障碍物在左半边区域
                    if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                        phil=k;
                        i5=phil;
                        while (phil<=i5&&i5<=phib)
                            mag(i5)=max(mag);
                            i5=i5+1;
                        end
                    end
                else
                %if (0<=k<km|phir<=k<=n) %障碍物在右半边区域
                    if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                        phir=k;
                        if phir<=k&&k<=n
                            i6=phib;
                            while (phib<=i6&&i6<=phir)
                                mag(i6)=max(mag);
                                i6=i6+1;
                            end
                        else
                        %if 0<=k<=km
                            i7=phib;
                            while (phib<=i7&&i7<=72)
                                mag(i7)=max(mag);
                                i7=i7+1;
                            end
                            i8=1;
                            while (0<=i8&&i8<=phir)
                                mag(i8)=max(mag);
                                i8=i8+1;
                            end
                        end
                    end
                end
            elseif 36<=km&&km<=72
               k2=k2+1
               phib=km-36; %初始极限角度=运动方向的反方向 
               phil=phib;  %初始左极限角度
               phir=phib;  %初始右极限角度
               beta = caculatebeta(robot,obstacle(i4,:));
               k = round(beta/alpha); %障碍物所在的扇区
               if k~=0
               else
                   k=1;
               end
               if phir<=k&&k<km  %障碍物在右半边区域
                  if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                     phir=k;
                     i9=phib;
                     while (phib<=i9&&i9<=phir)
                           mag(i9)=max(mag);
                           i9=i9+1;
                     end
                  end
               else
               %if (km<=k<72|0<=k<=phil) %障碍物在左半边区域
                   if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                      phil=k;
                        if 0<=k&&k<=phib
                            i10=phil;

                            while (phil<=i10&&i10<=phib)
                                mag(i10)=max(mag);
                                i10=i10+1;
                            end
                        else
                        %if km<=k<=72
                            i11=1;
                            while (0<=i11&&i11<=phib)
                                mag(i11)=max(mag);
                                i11=i11+1;
                            end
                            i12=1;
                            while (phil<=i12&&i12<=72)
                                mag(i12)=max(mag);
                                i12=i12+1;
                            end
                        end
                   end
               end  
            end
            i4=i4+1;
        end
        his=mag;      %现在 his 是一个含72个元素的向量--各扇区极障碍物密度
        %现在利用自适应阈值求一组备选方向（一组里有若干个备选方向）
        i1=1; %自适应阈值的循环次数
        kb=cell(1,blcs);
        howth=[];
        while (i1<=blcs)   % 自适应阈值的while循环变量：i1 i1取9的时候 会在某一点停下 i1取15的时候 完成避障！说明自适应阈值生效！！！
            %kb2=zeros(9,1);
            %howth2=zeros(9,1);
            Dt=norm(robot-endpoint);
            Dth(i1)=Dthmax-i1*dirtD;
            c=[];
            if  Dth(i1)<Dt
                threshold(i1)=C^2*Iij(Dth(i1));
                j=1;q=1;
                
                while (q<=n)       
                    %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                    if(his(q)< threshold(i1))
                        kr=q;                        % 找到了波谷的左端
                        while(q<=n && his(q)< threshold(i1))   %这一小段找到了波谷的右端
                            kl=q;
                            q=q+1;
                        end

                        if(kl-kr > smax)                  % 宽波谷
                            c   =  [c round(kl - smax/2)];  % 朝向左侧
                            c   =  [c round(kr + smax/2)];  % 朝向右侧
                            %j=j+1;
                            if(kt >= kr && kt <= kl)
                                c  = [c kt];                % straight at look ahead
                                %j=j+1;
                            end
                         elseif(kl-kr > smax/5)           % 窄波谷
                            c   =  [c round((kr+kl)/2-2.5)];
                            %j=j+1;
                         end

                    else
                        q=q+1;                            % his(q)不为0，直接下一个

                    end                                   % 退出if选择，再次进入while条件循环
                end                                       % 退出while循环
                % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                numb=length(c);
                temp2=howmuchs(Dth(i1),Dthmax,numb,c,kt);
            end 
            kb{1,i1}=c;
            howth=[howth temp2];       %存储阈值综合代价    
            i1=i1+1;
        end
        ftth=find(howth==min(howth));
        kbbest=kb{1,ftth(1)};   %此时，获得一个最佳阈值下的若干备选方向
        %现在，要判断是否采用VFH*算法进行路径规划
        %判断这组备选方向能否分成左右组：先确定当前运动方向
        %情况一：
        %if  (0<=km&&km<=round((ds/radius)/alpha))||(72-round((ds/radius)/alpha)<=km&&km<=72) 
        if 0<=km&&km<=round((ds/radius)/alpha)  %锐角情况
           lml=km+round((ds/radius)/alpha); %左边界
           lmr=km-round((ds/radius)/alpha)+72; %右边界
           numb1=length(kbbest); 
           phib=km+36; %当前运动反方向
           tempkl=[];
           tempkr=[];
           for j1=1:numb1
               if km<=kbbest(j1)&&kbbest(j1)<=lml  %左边方向可达
                  tempkl=[tempkl kbbest(j1)];
               %end
               elseif lml<=kbbest(j1)&&kbbest(j1)<=phib %左边方向不可达
                  tempkl=[tempkl lml];
               %end
               elseif phib<=kbbest(j1)&&kbbest(j1)<=lmr %右边方向不可达
                  tempkr=[tempkr lmr];
               %end
               elseif lmr<=kbbest(j1)&&kbbest(j1)<=72  %右边方向可达 状态一
                  tempkr=[tempkr kbbest(j1)];
               %end
               %if 0<=kbbest(j1)<=km  %右边方向可达 状态二 
               else
                  tempkr=[tempkr kbbest(j1)]; 
               end
           end         
        %else
        elseif 72-round((ds/radius)/alpha)<=km&&km<=72  %钝角情况
           lml=km+round((ds/radius)/alpha)-72;  %左边界
           if lml~=0
           else
              lml=1; 
           end
           lmr=km-round((ds/radius)/alpha);  %右边界
           numb1=length(kbbest);
           phib=km-36;  %当前运动反方向
           tempkl=[];
           tempkr=[];
           for j2=1:numb1
               if phib<=kbbest(j2)&&kbbest(j2)<=lmr  %右边方向不可达
                  tempkr=[tempkr lmr]; 
               %end
               elseif lmr<=kbbest(j2)&&kbbest(j2)<=km  %右边方向可达
                  tempkr=[tempkr kbbest(j2)]; 
               %end
               elseif km<=kbbest(j2)&&kbbest(j2)<=72  %左边方向可达 状态一
                  tempkl=[tempkl kbbest(j2)];
               %end
               elseif 0<=kbbest(j2)&&kbbest(j2)<=lml  %左边方向可达 状态二
                  tempkl=[tempkl kbbest(j2)]; 
               %end
               else
               %if lml<=kbbest(j2)<=phib  %左边方向不可达
                  tempkl=[tempkl lml]; 
               end
           end
        
        %end
        %情况二
        elseif  round((ds/radius)/alpha)<=km&&km<=36 
            lml=km+round((ds/radius)/alpha); %左边界
            lmr=km-round((ds/radius)/alpha); %右边界
            numb1=length(kbbest); 
            phib=km+36; %当前运动反方向
            tempkl=[];
            tempkr=[];
            for j3=1:numb1
                if km<=kbbest(j3)&&kbbest(j3)<=lml  %左边方向可达
                   tempkl=[tempkl kbbest(j3)]; 
                %end
                elseif lml<=kbbest(j3)&&kbbest(j3)<=phib  %左边方向不可达
                   tempkl=[tempkl lml]; 
                %end
                elseif km<=kbbest(j3)&&kbbest(j3)<=lmr  %右边方向可达
                   tempkr=[tempkr kbbest(j3)]; 
                %end
                else
                tempkr=[tempkr lmr];  %右边方向不可达
                end
            end 
        %end
        else
        %情况三
        %if  36<=km<=72-round((ds/radius)/alpha)
            lml=km+round((ds/radius)/alpha); %左边界
            lmr=km-round((ds/radius)/alpha); %右边界
            numb1=length(kbbest); 
            phib=km-36; %当前运动反方向
            tempkl=[];
            tempkr=[];
            for j4=1:numb1
                if lmr<=kbbest(j4)&&kbbest(j4)<=km  %右边方向可达
                   tempkr=[tempkr kbbest(j4)]; 
                %end
                elseif phib<=kbbest(j4)&&kbbest(j4)<=lmr  %右边方向不可达
                   tempkr=[tempkr lmr]; 
                %end
                elseif km<=kbbest(j4)&&kbbest(j4)<=lml  %左边方向可达
                   tempkl=[tempkl kbbest(j4)]; 
                %end
                else
                tempkl=[tempkl lml];  %左边方向不可达
                end
            end
        end
        % 判定是否进入VFH*算法 左右两边是不是都至少有一个方向
        geshul=length(tempkl);
        geshur=length(tempkr);
        if  geshul>=1&&geshur>=1  %符合条件-开始进入VFH*算法
            refp=cell(1,6);  %未来投影位置的坐标
            gc=cell(1,6);  %未来投影位置的代价
            hc=cell(1,6);  %未来投影位置的启发值
            fc=cell(1,6);  %未来投影位置的综合判定值
            pp=cell(1,6);  %未来投影位置的节点
            pfcun=zeros(1,4);  %储存每个节点的最佳方向
            ng=1;  % 开始VFH* 第一次搜索
            %先确定第一个节点的左右方向
            if  norm(robot-ref)==0
                lc=kt;
            else
            %if  norm(robot-ref)~=0
                lc=lc;
            end 
            g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
            for i14=1:geshul
                g1l(i14)=tempkl(i14);
                order1l=g1l(i14);
                ol1l=g1l(i14)-1;
                or1l=g1l(i14)+1;
                if ol1l~=0
                   if or1l~=73
                   how1l(i14,1)=howmany(g1l(i14),kt,km,lc,his(order1l),his(ol1l),his(or1l));
                   else
                   how1l(i14,1)=howmany(g1l(i14),kt,km,lc,his(order1l),his(ol1l),inf);    
                   end
                else
                   how1l(i14,1)=howmany(g1l(i14),kt,km,lc,his(order1l),inf,his(or1l)); 
                end
            end
            ft1l=find(how1l==min(how1l));
            ft1l;
            
            pf1lbest=filt(g1l(ft1l),kt);  %找到左边最佳方向
            g1r=zeros(geshur,1);how1r=zeros(geshur,1);  %右边前进方向
            for i15=1:geshur
                g1r(i15)=tempkr(i15);
                order1r=g1r(i15);
                ol1r=g1r(i15)-1;
                or1r=g1r(i15)+1;
                if ol1r~=0
                   if or1r~=73 
                   how1r(i15)=howmany(g1r(i15),kt,km,lc,his(order1r),his(ol1r),his(or1r));
                   else
                   how1r(i15)=howmany(g1r(i15),kt,km,lc,his(order1r),his(ol1r),inf);    
                   end
                else
                   how1r(i15)=howmany(g1r(i15),kt,km,lc,his(order1r),inf,his(or1r));
                end
            end
            ft1r=find(how1r==min(how1r));
            pf1rbest=filt(g1r(ft1r),kt);  %找到右边最佳方向
            %前进（投影）
            pp{1,1}=robot+[v_car*time*cos(pf1lbest*alpha),v_car*time*sin(pf1lbest*alpha)];
            pp{1,2}=robot+[v_car*time*cos(pf1rbest*alpha),v_car*time*sin(pf1rbest*alpha)];
            ol1l=pf1lbest-1;
            or1l=pf1lbest+1;
            order1l=pf1lbest;
            if ol1l~=0
               if or1l~=73 
               gc{1,1}=howmanys(pf1lbest,kt,km,lc,his(order1l),his(ol1l),his(or1l),ng);
               else
               gc{1,1}=howmanys(pf1lbest,kt,km,lc,his(order1l),his(ol1l),inf,ng);    
               end
            else
               gc{1,1}=howmanys(pf1lbest,kt,km,lc,his(order1l),inf,his(or1l),ng); 
            end
            ol1r=pf1rbest-1;
            or1r=pf1rbest+1;
            order1r=pf1rbest;
            if ol1r~=0
               if or1r~=73  
               gc{1,2}=howmanys(pf1rbest,kt,km,lc,his(order1r),his(ol1r),his(or1r),ng);
               else
               gc{1,2}=howmanys(pf1rbest,kt,km,lc,his(order1r),his(ol1r),inf,ng);    
               end
            else
               gc{1,2}=howmanys(pf1rbest,kt,km,lc,his(order1r),inf,his(or1r),ng); 
            end
            kt1l=round(caculatebeta(pp{1,1},endpoint)/alpha);  %新节点的目标方向 左
            kt1r=round(caculatebeta(pp{1,2},endpoint)/alpha);  %新节点的目标方向 右
            hc{1,1}=heu(kt1l,pf1lbest,pf1lbest,ng);
            hc{1,2}=heu(kt1r,pf1rbest,pf1rbest,ng);
            fc{1,1}=gc{1,1}+hc{1,1};
            fc{1,2}=gc{1,2}+hc{1,2};
            % 开始 VFH* 第二次搜索
            ng=ng+1;
            % 建立第一个投影点的极直方图
            i2=1;mag1l = zeros(n,1);his1l=zeros(n,1);
            while (i2<=length(obstacle))  
            
            %%%%%%%%%%% 下面一段程序得到投影点一的360度范围视野内的障碍物分布值 72个扇区的极障碍物密度  
            
                d1l = norm(obstacle(i2,:) - pp{1,1}); % 障碍物栅格与机器人之间距离
                if (d1l<dmax)
                    beta1l = caculatebeta(pp{1,1},obstacle(i2,:));  % 障碍物栅格向量的方向
                    rangle1l=asin(rsafe/d1l);        % 扩大的角度
                    k1l = round(beta1l/alpha);       % 逆时针数，第k个扇区区域
                    if(k1l == 0)
                        k1l = 1;
                    end
                    % 更新后的极坐标直方图的h值
                    if((5*k1l>rad2deg(beta1l)-rad2deg(rangle1l))&&(5*k1l<rad2deg(beta1l)+rad2deg(rangle1l)))  
                        h1l(k1l)=1;
                    else
                        h1l(k1l)=0;
                    end
                    i2=i2+1;

                    m1l = C^2*Iij(d1l);   % 障碍物栅格的向量幅值，与VFH计算方法不同
                    mag1l(k1l)=max(mag1l(k1l),m1l.*h1l(k1l));   % mag为zeros(n,1)，mag的第k个元素为m
                    i2=i2+1;
                else
                    i2=i2+1;
                end
            end
            % 第一个投影点：用VFH+排除一些扇区
            i41l=1; %应用VFH+算法，考虑运动半径因素 排除更多的扇区
            if  norm(pp{1,1}-ref)==0
                km=kt;
            else
            %if  norm(pp{1,1}-ref)~=0
                km=pf1lbest;
            end
            while (i41l<=length(obstacle))

                %%%%%%%%%% 考虑转向半径因素
                    km
                    dirtr(1)=radius*sin(km*alpha);   dirtr(2)=radius*cos(km*alpha);         %右转向中心差量
                    centerr(1)=pp{1,1}(1)+dirtr(1); centerr(2)=pp{1,1}(2)+dirtr(2); %右转向中心坐标
                    dirtl(1)=-radius*sin(km*alpha);  dirtl(2)=-radius*cos(km*alpha);        %左转向中心差量
                    centerl(1)=pp{1,1}(1)+dirtl(1); centerl(2)=pp{1,1}(2)+dirtl(2); %左转向中心坐标
                    %dor=norm(obstacle(i,:) - centerr);                          %障碍物到右转向中心的距离
                    %dor=norm(obstacle(i,:) - centerl);                          %障碍物到左转向中心的距离
                    dirtor(1)=obstacle(i41l)-pp{1,1}(1); dirtor(2)=obstacle(2)-pp{1,1}(2); %障碍物到机器人的坐标差
                    disor=(dirtr(1)-dirtor(1))^2+(dirtr(2)-dirtor(2))^2; %障碍物到右转向中心距离（平方）
                    disol=(dirtl(1)-dirtor(1))^2+(dirtl(2)-dirtor(2))^2; %障碍物到左转向中心距离（平方）
                    if 0<=km&&km<36
                        phib=km+36; %初始极限角度=运动方向的反方向
                        phil=phib;  %初始左极限角度
                        phir=phib;  %初始右极限角度
                        beta = caculatebeta(pp{1,1},obstacle(i41l,:));
                        k = round(beta/alpha); %障碍物所在的扇区
                        if km<=k&&km<phil  %障碍物在左半边区域
                            if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                                phil=k;
                                i5=phil;
                                while (phil<=i5&&i5<=phib)
                                    mag1l(i5)=max(mag1l);
                                    i5=i5+1;
                                end
                            end
                        else
                        %if (0<=k<km|phir<=k<=n) %障碍物在右半边区域
                            if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                                phir=k;
                                if phir<=k&&k<=n
                                    i6=phib;
                                    while (phib<=i6&&i6<=phir)
                                        mag1l(i6)=max(mag1l);
                                        i6=i6+1;
                                    end
                                else
                                %if 0<=k<=km
                                    i7=phib;
                                    while (phib<=i7&&i7<=72)
                                        mag1l(i7)=max(mag1l);
                                        i7=i7+1;
                                    end
                                    i8=1;
                                    while (0<=i8&&i8<=phir)
                                        mag1l(i8)=max(mag1l);
                                        i8=i8+1;
                                    end
                                end
                            end
                        end
                    else
                    %if 36<=km<=72
                       phib=km-36; %初始极限角度=运动方向的反方向 
                       phil=phib;  %初始左极限角度
                       phir=phib;  %初始右极限角度
                       beta = caculatebeta(pp{1,1},obstacle(i41l,:));
                       k = round(beta/alpha); %障碍物所在的扇区
                       if k~=0
                       else
                           k=1;
                       end
                       if phir<=k&&k<km  %障碍物在右半边区域
                          if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                             phir=k;
                             i9=phib;
                             while (phib<=i9&&i9<=phir)
                                   mag1l(i9)=max(mag1l);
                                   i9=i9+1;
                             end
                          end
                       else
                       %if (km<=k<72|0<=k<=phil) %障碍物在左半边区域
                           if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                              phil=k;
                                if 0<=k&&k<=phib
                                    i10=phil;
                                    while (phil<=i10&&i10<=phib)
                                        mag1l(i10)=max(mag1l);
                                        i10=i10+1;
                                    end
                                else
                                %if km<=k<=72
                                    i11=1;
                                    while (0<=i11&&i11<=phib)
                                        mag1l(i11)=max(mag1l);
                                        i11=i11+1;
                                    end
                                    i12=1;
                                    while (phil<=i12&&i12<=72)
                                        mag1l(i12)=max(mag1l);
                                        i12=i12+1;
                                    end
                                end
                           end
                       end  
                    end
                    i41l=i41l+1;
            end
        
            his1l=mag1l;      %现在 his 是一个含72个元素的向量--各扇区极障碍物密度
            % 第一个投影点：选取一组最佳备选方向
            i1=1; %自适应阈值的循环次数
            kb=cell(1,blcs);
            howth=[];
            while (i1<=blcs)   % 自适应阈值的while循环变量：i1 i1取9的时候 会在某一点停下 i1取15的时候 完成避障！说明自适应阈值生效！！！
                %kb2=zeros(9,1);
                %howth2=zeros(9,1);
                Dt=norm(pp{1,1}-endpoint);
                Dth(i1)=Dthmax-i1*dirtD;
                c=[];
                if  Dth(i1)<Dt
                    threshold(i1)=C^2*Iij(Dth(i1));
                    j=1;q=1;
                    
                    while (q<=n)       
                        %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                        if(his1l(q)< threshold(i1))
                            kr=q;                        % 找到了波谷的左端
                            while(q<=n && his1l(q)< threshold(i1))   %这一小段找到了波谷的右端
                                kl=q;
                                q=q+1;
                            end

                            if(kl-kr > smax)                  % 宽波谷
                                c   = [c round(kl - smax/2)];  % 朝向左侧
                                c   = [c round(kr + smax/2)];  % 朝向右侧
                                %j=j+2;
                                if(kt1l >= kr && kt1l <= kl)
                                    c  = [c kt1l];                % straight at look ahead
                                    %j=j+1;
                                end
                             elseif(kl-kr > smax/5)           % 窄波谷
                                c  = [c round((kr+kl)/2-2.5)];
                                %j=j+1;
                             end

                        else
                            q=q+1;                            % his(q)不为0，直接下一个

                        end                                   % 退出if选择，再次进入while条件循环
                    end                                       % 退出while循环

                    % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                    numb=length(c);
                    temp2=howmuchs(Dth(i1),Dthmax,numb,c,kt1l);
                end 

                %temp1=howmuch(Dth(i1),fk,kt,Dthmax);   
                kb{1,i1}=c;
                howth=[howth temp2];       %存储阈值综合代价    
                i1=i1+1;
            end
            ftth=find(howth==min(howth));
            kbbest1l=kb{1,ftth(1)};   %此时，获得一个最佳阈值下的若干备选方向
            %第一个投影点：能否分左右
            
            %情况一：
            %if  0<=km<=round((ds/radius)/alpha)||72-round((ds/radius)/alpha)<=km<=72 
            if 0<=km&&km<=round((ds/radius)/alpha)  %锐角情况
               lml=km+round((ds/radius)/alpha); %左边界
               lmr=km-round((ds/radius)/alpha)+72; %右边界
               numb1=length(kbbest1l); 
               phib=km+36; %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j1=1:numb1
                   if km<=kbbest1l(j1)&&kbbest1l(j1)<=lml  %左边方向可达
                      tempkl=[tempkl kbbest1l(j1)];
                   %end
                   elseif lml<=kbbest1l(j1)&&kbbest1l(j1)<=phib %左边方向不可达
                      tempkl=[tempkl lml];
                   %end
                   elseif phib<=kbbest1l(j1)&&kbbest1l(j1)<=lmr %右边方向不可达
                      tempkr=[tempkr lmr];
                   %end
                   elseif lmr<=kbbest1l(j1)&&kbbest1l(j1)<=72  %右边方向可达 状态一
                      tempkr=[tempkr kbbest1l(j1)];
                   %end
                   else
                   %if 0<=kbbest1l(j1)<=km  %右边方向可达 状态二   
                      tempkr=[tempkr kbbest1l(j1)]; 
                   end
               end         
            
            elseif 72-round((ds/radius)/alpha)<=km&&km<=72  %钝角情况
               lml=km+round((ds/radius)/alpha)-72;  %左边界
               if lml~=0
               else
                  lml=1; 
               end
               lmr=km-round((ds/radius)/alpha);  %右边界
               numb1=length(kbbest1l);
               phib=km-36;  %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j2=1:numb1
                   if phib<=kbbest1l(j2)&&kbbest1l(j2)<=lmr  %右边方向不可达
                      tempkr=[tempkr lmr]; 
                   %end
                   elseif lmr<=kbbest1l(j2)&&kbbest1l(j2)<=km  %右边方向可达
                      tempkr=[tempkr kbbest1l(j2)]; 
                   %end
                   elseif km<=kbbest1l(j2)&&kbbest1l(j2)<=72  %左边方向可达 状态一
                      tempkl=[tempkl kbbest1l(j2)];
                   %end
                   elseif 0<=kbbest1l(j2)&&kbbest1l(j2)<=lml  %左边方向可达 状态二
                      tempkl=[tempkl kbbest1l(j2)]; 
                   %end
                   else
                   %if lml<=kbbest1l(j2)<=phib  %左边方向不可达
                      tempkl=[tempkl lml]; 
                   end
               end
            
            %end
            %情况二
            elseif  round((ds/radius)/alpha)<=km&&km<=36 
                lml=km+round((ds/radius)/alpha); %左边界
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1l); 
                phib=km+36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j3=1:numb1
                    if km<=kbbest1l(j3)&&kbbest1l(j3)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1l(j3)]; 
                    %end
                    elseif lml<=kbbest1l(j3)&&kbbest1l(j3)<=phib  %左边方向不可达
                       tempkl=[tempkl lml]; 
                    %end
                    elseif km<=kbbest1l(j3)&&kbbest1l(j3)<=lmr  %右边方向可达
                       tempkr=[tempkr kbbest1l(j3)]; 
                    else
                    tempkr=[tempkr lmr];  %右边方向不可达
                    end
                end 
            %end
            else
            %情况三
            %if  36<=km<=72-round((ds/radius)/alpha)
                lml=km+round((ds/radius)/alpha); %左边界
                if lml~=0
                else
                   lml=1; 
                end
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1l); 
                phib=km-36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j4=1:numb1
                    if lmr<=kbbest1l(j4)&&kbbest1l(j4)<=km  %右边方向可达
                       tempkr=[tempkr kbbest1l(j4)]; 
                    %end
                    elseif phib<=kbbest1l(j4)&&kbbest1l(j4)<=lmr  %右边方向不可达
                       tempkr=[tempkr lmr]; 
                    %end
                    elseif km<=kbbest1l(j4)&&kbbest1l(j4)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1l(j4)]; 
                    else
                    tempkl=[tempkl lml];  %左边方向不可达 
                    end
                end
            end
            
            geshul=length(tempkl);
            geshur=length(tempkr);
            %第一个投影点：能分左右
            if  geshul>=1&&geshur>=1  
                lc=pf1lbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                       if or1l~=73 
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l));
                       else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),inf);   
                       end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),inf,his1l(or1l));
                    end
                end
                ft1ll=find(how1l==min(how1l));
                pf1llbest=filt(g1l(ft1ll),kt1l);  %找到左边最佳方向 
                g1r=zeros(geshur,1);how1r=zeros(geshur,1);  %右边前进方向
                for i15=1:geshur
                    g1r(i15)=tempkr(i15);
                    order1r=g1r(i15);
                    ol1r=g1r(i15)-1;
                    or1r=g1r(i15)+1;
                    if ol1r~=0
                       if or1r~=73
                       how1r(i15)=howmany(g1r(i15),kt1l,km,lc,his1l(order1r),his1l(ol1r),his1l(or1r));
                       else
                       how1r(i15)=howmany(g1r(i15),kt1l,km,lc,his1l(order1r),his1l(ol1r),inf);    
                       end
                    else
                       how1r(i15)=howmany(g1r(i15),kt1l,km,lc,his1l(order1r),inf,his1l(or1r));
                    end
                end
                ft1lr=find(how1r==min(how1r));
                pf1lrbest=filt(g1r(ft1lr),kt1l);  %找到右边最佳方向
                %前进（投影）
                pp{1,3}=pp{1,1}+[v_car*time*cos(pf1llbest*alpha),v_car*time*sin(pf1llbest*alpha)];
                pp{1,4}=pp{1,1}+[v_car*time*cos(pf1lrbest*alpha),v_car*time*sin(pf1lrbest*alpha)];
                ol1l=pf1llbest-1;
                or1l=pf1llbest+1;
                order1l=pf1llbest;
                if ol1l~=0
                   if or1l~=73 
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l),ng)+gc{1,1};
                   else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),inf,ng)+gc{1,1};    
                   end
                else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),inf,his1l(or1l),ng)+gc{1,1}; 
                end
                ol1r=pf1lrbest-1;
                or1r=pf1lrbest+1;
                order1r=pf1lrbest;
                if ol1r~=0
                   if or1r~=73 
                   gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1r),his1l(ol1r),his1l(or1r),ng)+gc{1,1};
                   else
                   gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1r),his1l(ol1r),inf,ng)+gc{1,1};    
                   end
                else
                   gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1r),inf,his1l(or1r),ng)+gc{1,1}; 
                end
                kt1ll=round(caculatebeta(pp{1,3},endpoint)/alpha);  %新节点的目标方向 左
                kt1lr=round(caculatebeta(pp{1,4},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,3}=heu(kt1ll,pf1llbest,pf1llbest,ng);
                hc{1,4}=heu(kt1lr,pf1lrbest,pf1lrbest,ng);
                fc{1,3}=gc{1,3}+hc{1,3};
                fc{1,4}=gc{1,4}+hc{1,4};
                pfcun(1,1)=pf1llbest;
                pfcun(1,2)=pf1lrbest;
            %end
            %第一个投影点：不能分左右 只有一个方向
            elseif  geshul>=1&&geshur==0  %该方向是左
                lc=pf1lbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                       if or1l~=73 
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l));
                       else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),inf);    
                       end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),inf,his1l(or1l)); 
                    end
                end
                ft1ll=find(how1l==min(how1l));
                pf1llbest=filt(g1l(ft1ll),kt1l);  %找到左边最佳方向 
                pp{1,3}=pp{1,1}+[v_car*time*cos(pf1llbest*alpha),v_car*time*sin(pf1llbest*alpha)];
                ol1l=pf1llbest-1;
                or1l=pf1llbest+1;
                order1l=pf1llbest;
                if ol1l~=0
                   if or1l~=73 
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l),ng)+gc{1,1};
                   else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),inf,ng)+gc{1,1};   
                   end
                else
                   gc{1,3}=howmanys(pf1llbest,kt1l,km,lc,his1l(order1l),inf,his1l(or1l),ng)+gc{1,1}; 
                end
                kt1ll=round(caculatebeta(pp{1,3},endpoint)/alpha);  %新节点的目标方向 左
                hc{1,3}=heu(kt1ll,pf1llbest,pf1llbest,ng);
                fc{1,3}=gc{1,3}+hc{1,3};
                fc{1,4}=inf;
                pfcun(1,1)=pf1llbest;
            %end
            elseif  geshur>=1&&geshul==0  %该方向是右
                lc=pf1lbest;
                g1l=zeros(geshur,1);how1l=zeros(geshur,1);  %右边前进方向 
                for i14=1:geshur
                    g1l(i14)=tempkr(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                        if or1l~=73
                        how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l));
                        else
                        how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),his1l(ol1l),inf);    
                        end
                    else
                        how1l(i14)=howmany(g1l(i14),kt1l,km,lc,his1l(order1l),inf,his1l(or1l)); 
                    end
                end
                ft1lr=find(how1l==min(how1l));
                pf1lrbest=filt(g1l(ft1lr),kt1l);  %找到右边最佳方向 
                pp{1,4}=pp{1,1}+[v_car*time*cos(pf1lrbest*alpha),v_car*time*sin(pf1lrbest*alpha)];
                ol1l=pf1lrbest-1;
                or1l=pf1lrbest+1;
                order1l=pf1lrbest;
                if ol1l~=0
                    if or1l~=73
                    gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),his1l(or1l),ng)+gc{1,1};
                    else
                    gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1l),his1l(ol1l),inf,ng)+gc{1,1};    
                    end
                else
                    gc{1,4}=howmanys(pf1lrbest,kt1l,km,lc,his1l(order1l),inf,his1l(or1l),ng)+gc{1,1};
                end
                kt1lr=round(caculatebeta(pp{1,4},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,4}=heu(kt1lr,pf1lrbest,pf1lrbest,ng);
                fc{1,4}=gc{1,4}+hc{1,4};
                fc{1,3}=inf;
                pfcun(1,2)=pf1lrbest;
            %end
            %第一个投影点：不能分左右 无方向
            else
            %if  geshul==geshur==0
                fc{1,3}=inf;
                fc{1,4}=inf;
            end
            % 建立第二个投影点的极直方图
            i3=1;mag1r = zeros(n,1);his1r=zeros(n,1);
            while (i3<=length(obstacle))  
            
            %%%%%%%%%%% 下面一段程序得到机器人360度范围视野内的障碍物分布值 72个扇区的极障碍物密度  
            
                d1r = norm(obstacle(i3,:) - pp{1,2}); % 障碍物栅格与机器人之间距离
                if (d1r<dmax)
                    beta1r = caculatebeta(pp{1,2},obstacle(i3,:));  % 障碍物栅格向量的方向
                    rangle1r=asin(rsafe/d1r);        % 扩大的角度
                    k1r = round(beta1r/alpha);       % 逆时针数，第k个扇区区域
                    if(k1r == 0)
                        k1r = 1;
                    end
                    % 更新后的极坐标直方图的h值
                    if((5*k1r>rad2deg(beta1r)-rad2deg(rangle1r))&&(5*k1r<rad2deg(beta1r)+rad2deg(rangle1r)))  
                        h1r(k1r)=1;
                    else
                        h1r(k1r)=0;
                    end
                    i3=i3+1;

                    m1r = C^2*Iij(d1r);   % 障碍物栅格的向量幅值，与VFH计算方法不同
                    mag1r(k1r)=max(mag1r(k1r),m1r.*h1r(k1r));   % mag为zeros(n,1)，mag的第k个元素为m
                    i3=i3+1;
                else
                    i3=i3+1;
                end 
            end
            %第二个投影点：用VFH+排除一些扇区
            i41r=1; %应用VFH+算法，考虑运动半径因素 排除更多的扇区
            km=pf1rbest;
            while (i41r<=length(obstacle))

                %%%%%%%%%% 考虑转向半径因素
                    
                    dirtr(1)=radius*sin(km*alpha);   dirtr(2)=radius*cos(km*alpha);         %右转向中心差量
                    centerr(1)=pp{1,2}(1)+dirtr(1); centerr(2)=pp{1,2}(2)+dirtr(2); %右转向中心坐标
                    dirtl(1)=-radius*sin(km*alpha);  dirtl(2)=-radius*cos(km*alpha);        %左转向中心差量
                    centerl(1)=pp{1,2}(1)+dirtl(1); centerl(2)=pp{1,2}(2)+dirtl(2); %左转向中心坐标
                    %dor=norm(obstacle(i,:) - centerr);                          %障碍物到右转向中心的距离
                    %dor=norm(obstacle(i,:) - centerl);                          %障碍物到左转向中心的距离
                    dirtor(1)=obstacle(i41r)-pp{1,2}(1); dirtor(2)=obstacle(2)-pp{1,2}(2); %障碍物到机器人的坐标差
                    disor=(dirtr(1)-dirtor(1))^2+(dirtr(2)-dirtor(2))^2; %障碍物到右转向中心距离（平方）
                    disol=(dirtl(1)-dirtor(1))^2+(dirtl(2)-dirtor(2))^2; %障碍物到左转向中心距离（平方）
                    if 0<=km&&km<36
                        phib=km+36; %初始极限角度=运动方向的反方向
                        phil=phib;  %初始左极限角度
                        phir=phib;  %初始右极限角度
                        beta = caculatebeta(pp{1,2},obstacle(i41r,:));
                        k = round(beta/alpha); %障碍物所在的扇区
                        if km<=k&&k<phil  %障碍物在左半边区域
                            if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                                phil=k;
                                i5=phil;
                                while (phil<=i5&&i5<=phib)
                                    mag1r(i5)=max(mag1r);
                                    i5=i5+1;
                                end
                            end
                        else
                        %if (0<=k<km|phir<=k<=n) %障碍物在右半边区域
                            if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                                phir=k;
                                if phir<=k&&k<=n
                                    i6=phib;
                                    while (phib<=i6&&i6<=phir)
                                        mag1r(i6)=max(mag1r);
                                        i6=i6+1;
                                    end
                                else
                                %if 0<=k<=km
                                    i7=phib;
                                    while (phib<=i7&&i7<=72)
                                        mag1r(i7)=max(mag1r);
                                        i7=i7+1;
                                    end
                                    i8=1;
                                    while (0<=i8&&i8<=phir)
                                        mag1r(i8)=max(mag1r);
                                        i8=i8+1;
                                    end
                                end
                            end
                        end
                    else
                    %if 36<=km<=72
                       phib=km-36; %初始极限角度=运动方向的反方向 
                       phil=phib;  %初始左极限角度
                       phir=phib;  %初始右极限角度
                       beta = caculatebeta(pp{1,2},obstacle(i41r,:));
                       k = round(beta/alpha); %障碍物所在的扇区
                       if k~=0
                       else
                          k=1;
                       end
                       if phir<=k&&k<km  %障碍物在右半边区域
                          if disor<(radius+rsafe)^2   %如果障碍物挡住了右转向圆轨迹
                             phir=k;
                             i9=phib;
                             while (phib<=i9&&i9<=phir)
                                   mag1r(i9)=max(mag1r);
                                   i9=i9+1;
                             end
                          end
                       else
                       %if (km<=k<72|0<=k<=phil) %障碍物在左半边区域
                           if disol<(radius+rsafe)^2   %如果障碍物挡住了左转向圆轨迹
                              phil=k;
                                if 0<=k&&k<=phib
                                    i10=phil;
                                    while (phil<=i10&&i10<=phib)
                                        i10
                                        mag1r(i10)=max(mag1r);
                                        i10=i10+1;
                                    end
                                else
                                %if km<=k<=72
                                    i11=1;
                                    while (0<=i11&&i11<=phib)
                                        mag1r(i11)=max(mag1r);
                                        i11=i11+1;
                                    end
                                    i12=1;
                                    while (phil<=i12&&i12<=72)
                                        mag1r(i12)=max(mag1r);
                                        i12=i12+1;
                                    end
                                end
                           end
                       end  
                    end
                    i41r=i41r+1;
            end
        
            his1r=mag1r;      %现在 his 是一个含72个元素的向量--各扇区极障碍物密度
            %第二个投影点：选取一组最佳备选方向
            i1=1; %自适应阈值的循环次数
            kb=cell(1,blcs);
            howth=[];
            while (i1<=blcs)   % 自适应阈值的while循环变量：i1 i1取9的时候 会在某一点停下 i1取15的时候 完成避障！说明自适应阈值生效！！！
                %kb2=zeros(9,1);
                %howth2=zeros(9,1);
                Dt=norm(pp{1,2}-endpoint);
                Dth(i1)=Dthmax-i1*dirtD;
                c=[];
                if  Dth(i1)<Dt
                    threshold(i1)=C^2*Iij(Dth(i1));
                    j=1;q=1;
                    
                    while (q<=n)       
                        %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                        if(his1r(q)< threshold(i1))
                            kr=q;                        % 找到了波谷的左端
                            while(q<=n && his1r(q)< threshold(i1))   %这一小段找到了波谷的右端
                                kl=q;
                                q=q+1;
                            end

                            if(kl-kr > smax)                  % 宽波谷
                                c   = [c round(kl - smax/2)];  % 朝向左侧
                                c   = [c round(kr + smax/2)];  % 朝向右侧
                                %j=j+2;
                                if(kt1r >= kr && kt1r <= kl)
                                    c  = [c kt1r];                % straight at look ahead
                                    %j=j+1;
                                end
                             elseif(kl-kr > smax/5)           % 窄波谷
                                c  = [c round((kr+kl)/2-2.5)];
                                %j=j+1;
                             end

                        else
                            q=q+1;                            % his(q)不为0，直接下一个

                        end                                   % 退出if选择，再次进入while条件循环
                    end                                       % 退出while循环

                    % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                    numb=length(c);
                    temp2=howmuchs(Dth(i1),Dthmax,numb,c,kt1r);
                end 

                %temp1=howmuch(Dth(i1),fk,kt,Dthmax);   
                kb{1,i1}=c;
                howth=[howth temp2];       %存储阈值综合代价    
                i1=i1+1;
            end
            ftth=find(howth==min(howth));
            kbbest1r=kb{1,ftth(1)};   %此时，获得一个最佳阈值下的若干备选方向
            %第二个投影点：能否分左右
            %情况一：
            %if  0<=km<=round((ds/radius)/alpha)||72-round((ds/radius)/alpha)<=km<=72 
            if 0<=km&&km<=round((ds/radius)/alpha)  %锐角情况
               lml=km+round((ds/radius)/alpha); %左边界
               lmr=km-round((ds/radius)/alpha)+72; %右边界
               numb1=length(kbbest1r); 
               phib=km+36; %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j1=1:numb1
                   if km<=kbbest1r(j1)&&kbbest1r(j1)<=lml  %左边方向可达
                      tempkl=[tempkl kbbest1r(j1)];
                   %end
                   elseif lml<=kbbest1r(j1)&&kbbest1r(j1)<=phib %左边方向不可达
                      tempkl=[tempkl lml];
                   %end
                   elseif phib<=kbbest1r(j1)&&kbbest1r(j1)<=lmr %右边方向不可达
                      tempkr=[tempkr lmr];
                   %end
                   elseif lmr<=kbbest1r(j1)&&kbbest1r(j1)<=72  %右边方向可达 状态一
                      tempkr=[tempkr kbbest1r(j1)];
                   %end
                   else
                   %if 0<=kbbest1r(j1)<=km  %右边方向可达 状态二   
                      tempkr=[tempkr kbbest1r(j1)]; 
                   end
               end         
            
            elseif 72-round((ds/radius)/alpha)<=km&&km<=72  %钝角情况
               lml=km+round((ds/radius)/alpha)-72;  %左边界
               if lml~=0
               else
                  lml=1; 
               end
               lmr=km-round((ds/radius)/alpha);  %右边界
               numb1=length(kbbest1r);
               phib=km-36;  %当前运动反方向
               tempkl=[];
               tempkr=[];
               for j2=1:numb1
                   if phib<=kbbest1r(j2)&&kbbest1r(j2)<=lmr  %右边方向不可达
                      tempkr=[tempkr lmr]; 
                   %end
                   elseif lmr<=kbbest1r(j2)&&kbbest1r(j2)<=km  %右边方向可达
                      tempkr=[tempkr kbbest1r(j2)]; 
                   %end
                   elseif km<=kbbest1r(j2)&&kbbest1r(j2)<=72  %左边方向可达 状态一
                      tempkl=[tempkl kbbest1r(j2)];
                   %end
                   elseif 0<=kbbest1r(j2)&&kbbest1r(j2)<=lml  %左边方向可达 状态二
                      tempkl=[tempkl kbbest1r(j2)]; 
                   %end
                   else
                   %if lml<=kbbest1r(j2)<=phib  %左边方向不可达
                      tempkl=[tempkl lml]; 
                   end
               end
            %end
            %end
            %情况二
            elseif  round((ds/radius)/alpha)<=km&&km<=36 
                lml=km+round((ds/radius)/alpha); %左边界
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1r); 
                phib=km+36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j3=1:numb1
                    if km<=kbbest1r(j3)&&kbbest1r(j3)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1r(j3)]; 
                    %end
                    elseif lml<=kbbest1r(j3)&&kbbest1r(j3)<=phib  %左边方向不可达
                       tempkl=[tempkl lml]; 
                    %end
                    elseif km<=kbbest1r(j3)&&kbbest1r(j3)<=lmr  %右边方向可达
                       tempkr=[tempkr kbbest1r(j3)]; 
                    else
                    tempkr=[tempkr lmr];  %右边方向不可达
                    end
                end 
            %end
            %情况三
            else
            %if  36<=km<=72-round((ds/radius)/alpha)
                lml=km+round((ds/radius)/alpha); %左边界
                lmr=km-round((ds/radius)/alpha); %右边界
                numb1=length(kbbest1r); 
                phib=km-36; %当前运动反方向
                tempkl=[];
                tempkr=[];
                for j4=1:numb1
                    if lmr<=kbbest1r(j4)&&kbbest1r(j4)<=km  %右边方向可达
                       tempkr=[tempkr kbbest1r(j4)]; 
                    %end
                    elseif phib<=kbbest1r(j4)&&kbbest1r(j4)<=lmr  %右边方向不可达
                       tempkr=[tempkr lmr]; 
                    %end
                    elseif km<=kbbest1r(j4)&&kbbest1r(j4)<=lml  %左边方向可达
                       tempkl=[tempkl kbbest1r(j4)]; 
                    else
                    tempkl=[tempkl lml];  %左边方向不可达
                    end
                end
            end
            
            geshul=length(tempkl);
            geshur=length(tempkr);
            %第二个投影点：能分左右
            if  geshul>=1&&geshur>=1  
                lc=pf1rbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                        if or1l~=73
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l));
                        else
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),inf);    
                        end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),inf,his1r(or1l)); 
                    end
                end
                ft1rl=find(how1l==min(how1l));
                pf1rlbest=filt(g1l(ft1rl),kt1r);  %找到左边最佳方向 
                g1r=zeros(geshur,1);how1r=zeros(geshur,1);  %右边前进方向
                for i15=1:geshur
                    g1r(i15)=tempkr(i15);
                    order1r=g1r(i15);
                    ol1r=g1r(i15)-1;
                    or1r=g1r(i15)+1;
                    if ol1r~=0
                       if or1r~=73 
                       how1r(i15)=howmany(g1r(i15),kt1r,km,lc,his1r(order1r),his1r(ol1r),his1r(or1r));
                       else
                       how1r(i15)=howmany(g1r(i15),kt1r,km,lc,his1r(order1r),his1r(ol1r),inf);    
                       end
                    else
                       how1r(i15)=howmany(g1r(i15),kt1r,km,lc,his1r(order1r),inf,his1r(or1r)); 
                    end
                end
                ft1rr=find(how1r==min(how1r));
                pf1rrbest=filt(g1r(ft1rr),kt1r);  %找到右边最佳方向
                pp{1,5}=pp{1,2}+[v_car*time*cos(pf1rlbest*alpha),v_car*time*sin(pf1rlbest*alpha)];
                pp{1,6}=pp{1,2}+[v_car*time*cos(pf1rrbest*alpha),v_car*time*sin(pf1rrbest*alpha)];
                ol1l=pf1rlbest-1;
                or1l=pf1rlbest+1;
                order1l=pf1rlbest;
                if ol1l~=0
                   if or1l~=73
                   gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l),ng)+gc{1,2};
                   else
                   gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),inf,ng)+gc{1,2};    
                   end
                else
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),inf,his1r(or1l),ng)+gc{1,2}; 
                end
                ol1r=pf1rrbest-1;
                or1r=pf1rrbest+1;
                order1r=pf1rrbest;
                if ol1r~=0
                   if or1r~=73
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1r),his1r(ol1r),his1r(or1r),ng)+gc{1,2};
                   else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1r),his1r(ol1r),inf,ng)+gc{1,2};    
                   end
                else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1r),inf,his1r(or1r),ng)+gc{1,2}; 
                end
                kt1rl=round(caculatebeta(pp{1,5},endpoint)/alpha);  %新节点的目标方向 左
                kt1rr=round(caculatebeta(pp{1,6},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,5}=heu(kt1rl,pf1rlbest,pf1rlbest,ng);
                hc{1,6}=heu(kt1rr,pf1rrbest,pf1rrbest,ng);
                fc{1,5}=gc{1,5}+hc{1,5};
                fc{1,6}=gc{1,6}+hc{1,6};
                pfcun(1,3)=pf1rlbest;
                pfcun(1,4)=pf1rrbest;
            %end
            %第二个投影点：不能分左右 只有一个方向
            elseif  geshul>=1&&geshur==0  %该方向是左
                lc=pf1rbest;
                g1l=zeros(geshul,1);how1l=zeros(geshul,1);  %左边前进方向 
                tempkl
                for i14=1:geshul
                    g1l(i14)=tempkl(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                        if or1l~=73 
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l));
                        else
                        how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),inf);    
                        end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),inf,his1r(or1l)); 
                    end
                end
                ft1rl=find(how1l==min(how1l));
                pf1rlbest=filt(g1l(ft1rl),kt1r);  %找到左边最佳方向
                pp{1,5}=pp{1,2}+[v_car*time*cos(pf1rlbest*alpha),v_car*time*sin(pf1rlbest*alpha)];
                ol1l=pf1rlbest-1;
                or1l=pf1rlbest+1;
                order1l=pf1rlbest;
                if ol1l~=0
                    if or1l~=73 
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l),ng)+gc{1,2};
                    else
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),inf,ng)+gc{1,2};
                    end
                else
                    gc{1,5}=howmanys(pf1rlbest,kt1r,km,lc,his1r(order1l),inf,his1r(or1l),ng)+gc{1,2};
                end
                kt1rl=round(caculatebeta(pp{1,5},endpoint)/alpha);  %新节点的目标方向 左
                hc{1,5}=heu(kt1rl,pf1rlbest,pf1rlbest,ng);
                fc{1,5}=gc{1,5}+hc{1,5};
                fc{1,6}=inf;
                pfcun(1,3)=pf1rlbest;
            %end
            elseif  geshur>=1&&geshul==0  %该方向是右
                lc=pf1rbest;
                g1l=zeros(geshur,1);how1l=zeros(geshur,1);  %右边前进方向 
                for i14=1:geshur
                    g1l(i14)=tempkr(i14);
                    order1l=g1l(i14);
                    ol1l=g1l(i14)-1;
                    or1l=g1l(i14)+1;
                    if ol1l~=0
                       if or1l~=73 
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l));
                       else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),his1r(ol1l),inf);
                       end
                    else
                       how1l(i14)=howmany(g1l(i14),kt1r,km,lc,his1r(order1l),inf,his1r(or1l)); 
                    end
                end
                ft1rr=find(how1l==min(how1l));
                pf1rrbest=filt(g1l(ft1rr),kt1r);  %找到右边最佳方向
                pp{1,6}=pp{1,2}+[v_car*time*cos(pf1rrbest*alpha),v_car*time*sin(pf1rrbest*alpha)];
                ol1l=pf1rrbest-1;
                or1l=pf1rrbest+1;
                order1l=pf1rrbest;
                if ol1l~=0
                   if or1l~=73  
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),his1r(or1l),ng)+gc{1,2};
                   else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1l),his1r(ol1l),inf,ng)+gc{1,2};
                   end
                else
                   gc{1,6}=howmanys(pf1rrbest,kt1r,km,lc,his1r(order1l),inf,his1r(or1l),ng)+gc{1,2}; 
                end
                kt1rr=round(caculatebeta(pp{1,6},endpoint)/alpha);  %新节点的目标方向 右
                hc{1,6}=heu(kt1rr,pf1rrbest,pf1rrbest,ng);
                fc{1,6}=gc{1,6}+hc{1,6};
                fc{1,5}=inf;
                pfcun(1,4)=pf1rrbest;
            else
            %第二个投影点：不能分左右 无方向
            %if  geshul==geshur==0
                fc{1,5}=inf;
                fc{1,6}=inf;
            end
            pd=[fc{1,3},fc{1,4},fc{1,5},fc{1,6}]  %判定值
            ft=find(pd==min(pd)) %找出几个节点fc值最小的
            dcyb=pfcun(1,ft);
            %lc=dc;
            
            if  length(ft)==1
                lc=dcyb;
                dc=dcyb;
                robot=pp{1,ft+2};  %VFH*算法得到的最终位置并赋予机器人
            elseif  length(ft)==2 %注意 ft 和 dcyb 此时是2行1列的矩阵（2×1）
                geshu=length(dcyb);
                if dcyb(1)==dcyb(geshu)
                   dc=dcyb(1); 
                   lc=dc;
                   robot=pp{1,ft(1)+2};
                else
                   kt_1=round(caculatebeta(pp{1,ft(1)+2},endpoint)/alpha);
                   kt_2=round(caculatebeta(pp{1,ft(2)+2},endpoint)/alpha);
                   g_=zeros(geshu,1);how_=zeros(geshu,1);xushu=zeros(geshu,1);
                   kt_=[kt_1 kt_2];
                   for i14=1:geshu
                       g_(i14)=dcyb(i14);
                       how_(i14)=howmanyss(g_(i14),kt_(i14));
                       xushu(i14)=ft(i14);
                   end
                   if how_(1)~=how_(2)
                       ft_=find(how_==min(how_));
                       dc=g_(ft_);
                       lc=dc;
                       robot=pp{1,xushu(ft_)};
                   else
                       g__=zeros(geshu,1);how__=zeros(geshu,1);xushu_=zeros(geshu,1);
                       for i14=1:geshu
                           g__(i14)=dcyb(i14);
                           how__(i14)=dif(g__(i14),kt_(i14));
                           xushu_(i14)=ft(i14);
                       end
                       ft__=find(how__==min(how__));
                       dc=g__(ft__);
                       lc=dc;
                       robot=pp{1,xushu(ft__)};
                   end
                 end
            end

            %当前运动方向
            %上次选择方向
        else   %如果不进入VFH*算法 那就使用VFH+算法判定
            if  norm(robot-ref)==0
                lc=kt;
            else
                lc=lc;
            end
            
            i1=1; %自适应阈值的循环次数
            kb=[];%储存最佳方向集合
            howth=[];%考虑阈值与该阈值下方向的代价集合
            
            % 自适应阈值开始！
            
            while (i1<=blcs)
                Dt=norm(robot-endpoint);
                Dth(i1)=Dthmax-i1*dirtD;
                c=[];
                if  Dth(i1)<Dt
                    threshold(i1)=C^2*Iij(Dth(i1));
                    j=1;q=1;

                    while (q<=n)       
                        %%%%%%%%%%%%%%%%%%%%%           所有合适的方向全部找出来
                        if(his(q)< threshold(i1))
                            kr=q;                        % 找到了波谷的左端
                            while(q<=n && his(q)< threshold(i1))   %这一小段找到了波谷的右端
                                kl=q;
                                q=q+1;
                            end

                            if(kl-kr > smax)                  % 宽波谷
                                c   =  [c round(kl - smax/2)];  % 朝向左侧
                                c   =  [c round(kr + smax/2)];  % 朝向右侧
                                j=j+2;
                                if(kt >= kr && kt <= kl)
                                    c  = [c kt];                % straight at look ahead
                                    j=j+1;
                                end
                            elseif(kl-kr > smax/5)           % 窄波谷
                                c   =  [c round((kr+kl)/2-2.5)];
                                j=j+1;
                            end

                        else
                            q=q+1;                            % his(q)不为0，直接下一个

                        end                                   % 退出if选择，再次进入while条件循环
                    end                                       % 退出while循环

                    % %%%% 低于阈值的宽窄谷里所有备选的方向都存到 c 里面了
                    % 开始筛选最优方向
                    if norm(robot-ref)==0                            
                       g=zeros(j-1,1);how=zeros(j-1,1);
                       for i2=1:j-1
                           g(i2)=c(i2);     %g中不含目标向量
                           order=g(i2);
                           ol=g(i2)-1;
                           or=g(i2)+1;
                           dc=kt;           %由于机器人还没动，所以目标方向就是当前运动方向
                           lc=kt;           %由于机器人还没动，所以目标方向就是上次选择方向
                           if ol~=0   %防止出现第0或第73
                              if or~=73
                               how(i2)=howmany(g(i2),kt,dc,lc,his(order),his(ol),his(or)); %代价函数计算最优方向 how为代价 元素个数与 g 是相同的
                              else
                               how(i2)=howmany(g(i2),kt,dc,lc,his(order),his(ol),inf);   
                              end
                           else
                              how(i2)=howmany(g(i2),kt,dc,lc,his(order),inf,his(or)); 
                           end 
                       end                                                             
                       ft=find(how==min(how));
                       fk=g(ft);
                       kb=[kb fk];  % 当前阈值下的最佳备选方向
                    else
                       g=zeros(j-1,1);how=zeros(j-1,1);
                       for i3=1:j-1
                           g(i3)=c(i3);
                           order=g(i3);
                           ol=g(i3)-1;
                           or=g(i3)+1;
                           if ol~=0   %防止出现第0或第73
                              if or~=73
                               how(i3)=howmany(g(i3),kt,dc,lc,his(order),his(ol),his(or));
                              else
                               how(i3)=howmany(g(i3),kt,dc,lc,his(order),his(ol),inf);   
                              end
                           else
                              how(i3)=howmany(g(i3),kt,dc,lc,his(order),inf,his(or)); 
                           end
                       end
                       ft=find(how==min(how));
                       fk=g(ft);
                       %一个阈值下也许会出现多个最佳方向 得筛选下
                       cd=length(fk); %最优方向的个数
                       if cd==1
                          fk=fk;  % 当前阈值下的最佳备选方向 
                       else 
                          g_=zeros(cd,1);how_=zeros(cd,1); 
                          for i4=1:cd
                              g_(i4)=fk(i4);
                              how_(i4)=dif(g_(i4),kt);
                          end
                          ft_=find(how_==min(how_));
                          fk=g_(ft_);  % 当前阈值下的最佳备选方向
                       end
                       kb=[kb fk];  % 当前阈值下的最佳备选方向
                    end 
                    
                    temp1=howmuch(Dth(i1),fk,kt,Dthmax); %计算 阈值与该阈值下最佳方向的综合代价
                    howth=[howth temp1]; %存储综合代价
                end
                i1=i1+1;
            end
            ft=find(howth==min(howth));
            fbestyb=kb(ft);  %VFH+算法得到的最好方向
            % 防止有多个最优方向
            if  length(ft)==1
                dc=fbestyb;       % 当前的运动方向
                lc=dc;       % 上一次选择的方向
                robot=robot+[v_car*time*cos(fbestyb*alpha),v_car*time*sin(fbestyb*alpha)];  %VFH+算法得到的最终位置并赋予机器人
            elseif  length(ft)==2 %注意 ft 和 dcyb 此时是2行1列的矩阵（2×1）
                geshu=length(fbestyb);
                if fbestyb(1,1)==fbestyb(geshu,1)
                   dc=fbestyb(1,1);  % 当前的运动方向
                   lc=dc; % 上一次选择的方向
                   robot=robot+[v_car*time*cos(lc*alpha),v_car*time*sin(lc*alpha)];
                else
                   
                   g_=zeros(geshu,1);how_=zeros(geshu,1);  
                   for i14=1:geshu
                        g_(i14)=fbestyb(i14);
                        how_(i14)=howmanyss(g_(i14),kt);
                   end
                   ft_=find(how_==min(how_));
                   dc=g_(ft_); % 当前的运动方向
                   lc=dc; % 上一次选择的方向
                   robot=robot+[v_car*time*cos(lc*alpha),v_car*time*sin(lc*alpha)];
                end    
            elseif  length(ft)==3
                geshu=length(fbestyb);
                g_=zeros(geshu,1);how_=zeros(geshu,1);  
                   for i14=1:geshu
                        g_(i14)=fbestyb(i14);
                        how_(i14)=howmanyss(g_(i14),kt);
                   end
                   ft_=find(how_==min(how_));
                   dc=g_(ft_); % 当前的运动方向
                   lc=dc; % 上一次选择的方向
                   robot=robot+[v_car*time*cos(lc*alpha),v_car*time*sin(lc*alpha)];
            %else 
            end
        end
        ref=startpoint;
        scatter(robot(1),robot(2),'.r');
        drawnow;
        kt=round(caculatebeta(robot,endpoint)/alpha);  %新的目标方向
        if(kt==0)
            kt=n;
        end
        if(norm(robot-endpoint))>step          % 机器人位置和终点位置差距大于0.1时
        else
            break
        end
        %至此避障规划一次完毕
        
        obstacle([5271:8870],:)=[];    %避障完毕后剔除动态障碍物
        obstacle;
    end
end
 