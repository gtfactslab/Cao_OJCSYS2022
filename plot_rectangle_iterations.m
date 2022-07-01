hold on
eq_rect = history(1, :);
pos = [eq_rect(1), eq_rect(2), eq_rect(3) - eq_rect(1), eq_rect(4) - eq_rect(2)];
rectangle('Position', pos, 'LineWidth', 3, 'EdgeColor', 'r', 'LineStyle', ':');
for i = 2:iterations
    eq_rect = history(i, :);
    pos = [eq_rect(1), eq_rect(2), eq_rect(3) - eq_rect(1), eq_rect(4) - eq_rect(2)];
    rectangle('Position', pos, 'LineWidth', 1, 'EdgeColor', 'k','LineStyle', '--');
end
pos = [x_eq(1), x_eq(2), x_eq(3) - x_eq(1), x_eq(4) - x_eq(2)];
if ~complete
    rectangle('Position', pos, 'EdgeColor', 'r', 'LineWidth', 3);
else
    rectangle('Position', pos, 'EdgeColor', 'g', 'LineWidth', 3, 'LineStyle', '-');
end
axis(axis_range)
grid minor
axis equal
xlabel('x_1')
ylabel('x_2')

if complete
    eq_rect = history(1, :);
    text(eq_rect(3), eq_rect(4), ' k=1');
    eq_rect = history(2, :);
    text(eq_rect(3), eq_rect(4), ' k=2');
    eq_rect = history(3, :);
    text(eq_rect(3), eq_rect(4), '      k=3');
end
hold off