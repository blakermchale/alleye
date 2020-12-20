function V = achar_normal(obstaculo,centro,E)
num_obstaculos = length(obstaculo(:,1));
V = [];
E;
for i=1:num_obstaculos
    
    switch E(i,2)
        case 0
            if(centro(1)>E(1))
                V = [V;[-1,0]];
            else
                V = [V;[1,0]];
            end
        case 1
            if(centro(2)>E(1))
                V = [V;[0,-1]];
            else
                V = [V;[0,1]];
            end
        case 2
            %y = ax+b, 0 = -y + ax+b
            novo_m = tan(atan(E(1))-(pi/2));
            vertice1 = obstaculo(i,:);
            V = [V;([1,novo_m]/norm([1,novo_m]))];
    end
    
end
end
