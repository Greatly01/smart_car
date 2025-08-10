
function model=Basics()

    % Start
    xs=-13;
    ys=-12;
    % Destination
    xt=13;
    yt=10;
    
    % Obsticles
    r = -1 + (1+1)*rand;
    xobs=[1.5+r*rand 4.0+r*rand   1.6+r*rand 8.0+r*rand   -6.0+r*rand  6.0+r*rand -3.0+r*rand];
    yobs=[4.5+r*rand 1.0+r*rand   7.1+r*rand 8.0+r*rand   -4.0+r*rand  -6.0+r*rand -9.0+r*rand];
    robs=[1.5+r*rand 1.0+r*rand   1.3+r*rand 1.6+r*rand   2.0+r*rand   2.0+r*rand 3.0+r*rand];
   
    n=6;
    xmin=-10;
    xmax= 10;
    ymin=-10;
    ymax= 10;
    model.xs=xs;
    model.ys=ys;
    model.xt=xt;
    model.yt=yt;
    model.xobs=xobs;
    model.yobs=yobs;
    model.robs=robs;
    model.n=n;
    model.xmin=xmin;
    model.xmax=xmax;
    model.ymin=ymin;
    model.ymax=ymax;
    
end