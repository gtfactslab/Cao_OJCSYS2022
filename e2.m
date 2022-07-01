function F = e2(x, W)
%x = [x1, x2, x1hat, x2hat]
%W = [-3, 3];

F(1) = -x(1) - x(1)^3 - x(4) - W(2);
F(2) = -x(2) - x(2)^3 + x(1) + W(1)^3;
F(3) = -x(3) - x(3)^3 - x(2) - W(1);
F(4) = -x(4) - x(4)^3 + x(3) + W(2)^3;
