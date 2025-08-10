function Draw_path(pgon, x_val, paths)

close
plot(pgon,'FaceColor','red','edgecolor','red')
xlim([0,100]);
ylim([0,100]);
hold on
axis square;
grid on
plot(x_val, paths,'k');
scatter(x_val(1), paths(1,1),40, 'green', 'filled','o');
scatter(x_val(11), paths(1,11),40, 'yellow', 'filled','o');
end

