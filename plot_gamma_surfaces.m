[full_x1, full_x2] = meshgrid(axis_range(1):0.25:axis_range(2),axis_range(3):0.25:axis_range(4));
full_size = size(full_x1);
full_upper = zeros(full_size);
full_lower = zeros(full_size);

for i = 1:full_size(1)
    for j = 1:full_size(2)
        [full_lower(i, j), full_upper(i, j)] = gamma([full_x1(i, j), full_x2(i, j), full_x1(i, j), full_x2(i, j)]);
    end
end
mask = (full_x1 >= x_eq(1)) & (full_x1 <= x_eq(3)) & (full_x2 >= x_eq(2)) & (full_x2 <= x_eq(4)); 
roi_x1 = full_x1;
roi_x2 = full_x2;
roi_upper = full_upper;
roi_lower = full_lower;
roi_x1(~mask) = NaN;
roi_x2(~mask) = NaN;
roi_upper(~mask) = NaN;
roi_lower(~mask) = NaN;

hold on
surf(full_x1,full_x2,full_upper,'FaceColor','b', 'EdgeAlpha', 0.5)
surf(full_x1,full_x2,full_lower,'FaceColor','m', 'EdgeAlpha', 0.5)
if ~complete
    surf(roi_x1,roi_x2,roi_upper,'FaceColor','r')
    surf(roi_x1,roi_x2,roi_lower,'FaceColor','r')
else
    surf(roi_x1,roi_x2,roi_upper,'FaceColor','g')
    surf(roi_x1,roi_x2,roi_lower,'FaceColor','g')
end
xlabel('x_1')
ylabel('x_2')
zlabel('gamma')
axis equal
hold off