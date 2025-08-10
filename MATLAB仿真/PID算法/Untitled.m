    %设一被控对象G（s）=50/(0.125s^2+7s),  
    %用增量式PID控制算法编写仿真程序  
    %输入分别为单位阶跃、正弦信号，采样时间为1ms，控制器输出限幅：[-3,3],  
    %仿真曲线包括系统输出及误差曲线。  

    ts=0.001;                %采样时间  
    sys=tf(50,[0.125,7, 0]); %tf是传递函数  即被控对象函数G（）;  
    dsys=c2d(sys,ts,'z');    %把被控对象的传函离散化取Z变换n阶定常离散系统差分方程
                             %在零初始条件下取Z变换：
                             %dsys即Y(z)/U(z)
    [num,den]=tfdata(dsys,'v');% 离散化后提取分子、分母    
    u_1=0.0;  
    u_2=0.0;  
    y_1=0.0;  
    y_2=0.0;  
    x=[0,0,0]';  
    error_1=0;  
    error_2=0;  
    
    %核心代码

    for k=1:1:1000  
    time(k)=k*ts;                   %采样次数  
    S=1;  
    if S==1                         %阶跃输入
        kp=6.5;ki=0.1;kd=1;         %初始化PID    
        rin(k)=1;                   %Step Signal   
    elseif S==2                     %正弦输入
        kp=10;ki=0.1;kd=15;             
        rin(k)=0.5*sin(2*pi*k*ts);  %Sine Signal     即实际输入      
    end 
    
    %增量式PID的实现 参考上文增量式PID算法
    du(k)=kp*x(1)+kd*x(2)+ki*x(3);        
    u(k)=u_1+du(k);               %真正的PID输出应该为du+前一时刻的输出
    
    if u(k)>=3    %输出限幅   
       u(k)=3;  
    end  
    if u(k)<=-3  
       u(k)=-3;  
    end  

    %Linear model 难点就是把传递函数转化为差分方程，以实现PID控制。 
    yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;          %实际输出 num为dsys分子多项式系数，den为dsys分母多项式系数，从n阶定常离散系统差分方程变化来的。
    error(k)=rin(k)-yout(k);                                       % 误差 输入-输出 
    u_2=u_1;                                                       %保存上上次输入   为下次计算  
    u_1=u(k);                                                      %保存上一次控制系数   为下次计算  
    y_2=y_1;                                                       %保存上上次次输出   为下次计算  
    y_1=yout(k);                                                   %保存上一次输出   为下次计算  

    x(1)=error(k)-error_1;                                         %KP的系数  
    x(2)=error(k)-2*error_1+error_2;                               %KD的系数  
    x(3)=error(k);                                                 %KI的系数
    error_2=error_1;                                               %上次的变上上次误差
    error_1=error(k);                                              %这次的变上次的误差
    end 


    figure(1);  
    plot(time,rin,'b',time,yout,'r');                              %输入 和实际控制输出  
    xlabel('time(s)'),ylabel('rin,yout');   
    figure(2);  
    plot(time,error,'r')                                           %时间误差输出曲线  
    xlabel('time(s)');ylabel('error'); 