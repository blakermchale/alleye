function E = achar_curva2(obstaculo)

num_obstaculos = length(obstaculo(:,1));
E = [];
for i=1:num_obstaculos
    vertice1 = obstaculo(i,:);
    vertice2 = obstaculo(mod(i+1,num_obstaculos+1)+fix((i+1)/(num_obstaculos+1)),:);
% ax + by + c = 0
%%caso em que possuem o mesmo valor para x
if(vertice1(1)==vertice2(1))
    E = [E;[vertice1(1),0]];
else
    %%caso em que possuem o mesmo valor para y
    if (vertice1(2)==vertice2(2))
        E = [E;[vertice2(2),1]];
    else
        %%outros casos
        % y - y0 = m (x- x0)
        m = (vertice2(2)-vertice1(2))/(vertice2(1)-vertice1(1));
        E = [E;[m,2]];
    end
end
end
end