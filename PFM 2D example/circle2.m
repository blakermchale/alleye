function h = circle2(ponto,r)
hold on
x = ponto(1);
y = ponto(2);
th = 0:0.1:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
% size(ones(length(th),1)*0.2)
% size(vector)
cont = 1;
for (z =  0:0.1:2*pi)
    azul=[0.294, 0.658, 0.905];
    %h(cont) = plot(xunit(cont), yunit(cont),'color','blue','LineWidth',2);
    h(cont) = plot(xunit(cont), yunit(cont),'--','color',azul,'LineWidth',1);
cont = cont+1;
end
%h = plot(xunit, yunit, 'color','blue');
h = plot(xunit, yunit, 'color',azul, 'LineWidth',1);
end


