
x=randi([0,150],30,1);
y=randi([0,120],30,1);     %%%一时找不到数据，就自动生成了三十个坐标点（xi,yi),i=1,2...,30;
sj0=[x,y];
ss0=[200,200];             %%%出发点选取（100，100）；
sj=[ss0;sj0;ss0];           %%%将出发点加在30个点的开始和结束；

%%%(1)计算各点之间的距离，存储于距离矩阵d中
d=zeros(length(sj));       %%%距离矩阵初始化；
for i=1:length(sj)-1
    for j=i+1:length(sj)
        d(i,j)=sqrt((sj(i,1)-sj(j,1))^2+(sj(i,2)-sj(j,2))^2);
    end
end
d=d+d';
 
%%%（2）求一个较好的初始解
path=[];                 %%%路线初始化；
long=inf;                %%%路线长度初始化；
rand('state',sum(clock));
for j=1:100000
    path0=[1,1+randperm(length(sj0)),length(sj)];
    temp=0;
    for i=1:length(sj)-1
        temp=temp+d(path0(i),path0(i+1));
    end
    if temp<long
        path=path0;
        long=temp;
    end
end
%%%（3）退火过程
L=50000;
at=0.999;  %%%降温系数
T=100;     %%%初始温度
e=0.1^30;  %%%终止温度
long_temp=[];
m=0;
for k=1:L
    c=1+randperm(length(sj0),2);
    c=sort(c);
    c1=c(1);
    c2=c(2);
    df=d(path(c1-1),path(c2))+d(path(c1),path(c2+1))-d(path(c1-1),path(c1))-d(path(c2),path(c2+1));%%%代价函数增量
    if df<0     %%%接受准则
        path=[path(1:c1-1),path(c2:-1:c1),path(c2+1:length(sj))];
        long=long+df;
    elseif exp(-df/T)>=rand   %%% 概率接受
        path=[path(1:c1-1),path(c2:-1:c1),path(c2+1:length(sj))];
        long=long+df;
    end
    t=0;
    for i=1:length(sj)-1
        t=t+d(path(i),path(i+1));
    end
    long_temp=[long_temp,t];
    T=at*T;
    if T<e
        break;
    end
    m=m+1;
end
path;
long;
subplot(2,1,1)
xx=sj(path,1);
yy=sj(path,2);
plot(xx,yy,'-*')
subplot(2,1,2)
plot(long_temp,'-')
