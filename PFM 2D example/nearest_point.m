function ponto_minimo =nearest_point(ConjObstaculos,circles, ponto)
%referencia
%http://www.mathworks.com/matlabcentral/fileexchange/19398-distance-from-a-point-to-polygon/content/p_poly_dist.m

n  = length(ConjObstaculos{2}); % numero de obstaculos
%fazer para o primeiro obstaculo
% Obstaculos_x = ConjObstaculos{2}{1}(:,1);
% Obstaculos_y = ConjObstaculos{2}{1}(:,2);
% [d,x_poly,y_poly] = p_poly_dist(ponto(1), ponto(2), Obstaculos_x, Obstaculos_y);
d = 50;
%obstaculos seguintes
for i = 1:n
%     vertices = ConjObstaculos{2}{i};
    obs_x = ConjObstaculos{2}{i}(:,1);
    obs_y = ConjObstaculos{2}{i}(:,2);
    [aux_d,aux_x_poly,aux_y_poly] = p_poly_dist(ponto(1), ponto(2), obs_x, obs_y);
    if(abs(aux_d)<abs(d))
        d = abs(aux_d);
        x_poly=aux_x_poly;
        y_poly=aux_y_poly;
    end
end
%dist = eucl_dist([ponto(1) ponto(2)],[x_poly,y_poly])
dist=d;
%ponto_minimo = [x_poly,y_poly,d];
%plot(x_poly,y_poly,'*')

% Distance between a point and a circle
% Fonte: https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
dists = [];
% circle = [x,y,raio]

tam = size(circles);
if tam(1) > 0
    
    for i=1: length(circles(:,1))
        dists = [dists; eucl_dist(ponto,[circles(i,1), circles(i,2)]) - circles(i,3)];
    end

    [dmin pos]= min(dists);

    if(dmin > d)
        ponto_minimo = [x_poly,y_poly,d];
    else
        x=circles(pos,1);
        y=circles(pos,2);
        r=circles(pos,3);

        %p_x = bm(1)+ raio(posicao)* ((rx-bm(1))/ sqrt( (rx-bm(1))^2 + (ry-bm(2))^2 ));
        %p_y = bm(2)+ raio(posicao)* ((ry-bm(2))/ sqrt( (rx-bm(1))^2 +
        %(ry-bm(2))^2 ));    ;

        px = x+ r* ((ponto(1)-x)/ sqrt( (ponto(1)-x)^2 + (ponto(2)-y)^2 ));
        py = y+ r* ((ponto(2)-y)/ sqrt( (ponto(1)-x)^2 + (ponto(2)-y)^2 ));

        ponto_minimo = [px,py,dmin];
    end
else
    ponto_minimo = [x_poly,y_poly,d];
end


end
