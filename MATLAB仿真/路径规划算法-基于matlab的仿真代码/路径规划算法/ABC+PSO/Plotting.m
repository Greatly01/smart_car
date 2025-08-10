
function Plotting(sol,model)

    xs=model.xs;
    ys=model.ys;
    xt=model.xt;
    yt=model.yt;
    xobs=model.xobs;
    yobs=model.yobs;
    robs=model.robs;
    
    XS=sol.XS;
    YS=sol.YS;
    xx=sol.xx;
    yy=sol.yy;
    
    axis square;
    theta=(1/24:1/36:1)'*2*pi;
    for k=1:numel(xobs)
        fill(xobs(k)+robs(k)*cos(theta),yobs(k)+robs(k)*sin(theta),[1 0 0],'edgecolor',[1 0 0]);
        hold on;
    end
    

% figure;
    plot(xx,yy,'k','LineWidth',2);
    plot(XS,YS,'ro');
    plot(xs,ys,'marker','o','markersize',10,'markerfacecolor','green');
    plot(xt,yt,'marker','o','markersize',10,'markerfacecolor','yellow');
    hold off;
    grid on;
    axis([-15 15 -15 15])
    axis square;

end