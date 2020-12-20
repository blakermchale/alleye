function valor = eucl_dist(p1,p2)
valor = 0;
for i = 1:length(p1(:))
valor  = valor + (p1(i)-p2(i))^2;
end
valor = sqrt(valor);
% valor = sqrt((p1(1)-p2(1))*(p1(1)-p2(1))+(p1(2)-p2(2))*(p1(2)-p2(2)));
end
