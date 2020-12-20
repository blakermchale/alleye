function [N circles E V num_obstaculos] = teste_map(ponto_inicial,ponto_final, nfig)
clf('reset');
function gerarObs(largura, altura, centrox, centroy)
    num_obstaculos= num_obstaculos+1;
    centro = [centrox, centroy];
    angulo = 0;
    MatRot = [cos(angulo),sin(angulo);sin(angulo),cos(angulo)];

    pos_relativa_vertice1 = MatRot*[largura/2;-altura/2];
    pos_relativa_vertice2 = MatRot*[largura/2;altura/2];
    pos_relativa_vertice3 = MatRot*[-largura/2;altura/2];
    pos_relativa_vertice4 = MatRot*[-largura/2;-altura/2];

    obs_vertice1 = centro+pos_relativa_vertice1';
    obs_vertice2 = centro+pos_relativa_vertice2';
    obs_vertice3 = centro+pos_relativa_vertice3';
    obs_vertice4 = centro+pos_relativa_vertice4';

    obstaculos = [obs_vertice1;obs_vertice2;obs_vertice3;obs_vertice4];

    N{num_obstaculos} = obstaculos;
    V{num_obstaculos} = achar_normal2 (obstaculos,centro,achar_curva2 (obstaculos));
    E{num_obstaculos} = achar_curva2 (obstaculos);

    % plotar_curva2(E{num_obstaculos},obstaculos);
    rectangle('Position',[centro(1)-(largura/2),centro(2)-(altura/2),largura,altura],'FaceColor','black');
end
circles=[];
for id_figura=1:nfig
figure(id_figura);
num_obstaculos=0;
tipo_obstaculo=ones(1,num_obstaculos);
%area= [-20,20;-20,20];
area= [-6,8;-11,3;];
hold on;

plot(ponto_inicial(1),ponto_inicial(2),'o','color','k','MarkerFaceColor','black');
plot(ponto_final(1),ponto_final(2),'o','color','k','MarkerFaceColor','black');

%gerarObs(largura, altura, centrox, centroy)
%Adicionar aqui para desenhar
% gerarObs(1,12, -10, 12);
% gerarObs(1,12, -10, -3);
% 
% gerarObs(1,6, -10, -17);
% 
% gerarObs(1,8, 10, -5);
% gerarObs(1,8, 10, 7);
% gerarObs(1,5, 10, -15);
% gerarObs(1,6, 10, 17);
% 
% gerarObs(1,15, 0, 10.5);
% gerarObs(1,15, 0, -10.5);
% 
% gerarObs(1,5, 5, -17.5);
% gerarObs(1,5, -5, 17.5);
% gerarObs(1,5, 5, 17.5);
% gerarObs(1,5, -5, -17.5);
% 
% gerarObs(1,3, -17, 18.5);
% gerarObs(6,1, -17, 13);
% gerarObs(6,1, -13, 9);
% gerarObs(1,6, -15.5, 6);
% gerarObs(1,6, -15.5, -4);
% gerarObs(6,1, -13, -7);
% gerarObs(6,1, -13, -14.5);
% 
% gerarObs(1,3, 17, 18.5);
% gerarObs(6,1, 17, 13);
% gerarObs(6,1, 13, 9);
% gerarObs(1,6, 15.5, 6);
% gerarObs(1,6, 15.5, -4);
% gerarObs(6,1, 13, -7);
% gerarObs(6,1, 13, -14.5);
% 
% gerarObs(5,1, -5, 1);
% gerarObs(1,15, 4, 1);

gerarObs(2,1,0.72,-4.5)
gerarObs(1,1,-2.09,-1.02)
%gerarObs(.05,14,2.1,-4) %add right bound
%gerarObs(.05,14,-2.7,-4) %add left bound


%% objeto delimitador
%% 

num_obstaculos= num_obstaculos+1;
obs_vertice1 = [ area(1,1), area(2,1)];
obs_vertice2 = [ area(1,1), area(2,2)];
obs_vertice3 = [ area(1,2), area(2,2)];
obs_vertice4 = [ area(1,2), area(2,1)];
obstaculos = [obs_vertice1;obs_vertice2;obs_vertice3;obs_vertice4];
centro = (obstaculos(1,:)+obstaculos(2,:)+obstaculos(3,:)+obstaculos(4,:))/4;

N{num_obstaculos} = obstaculos;
V{num_obstaculos} = achar_normal2 (obstaculos,centro,achar_curva2 (obstaculos));
E{num_obstaculos} = achar_curva2 (obstaculos);
for id_figura=1:nfig
    figure(id_figura);
    line([obstaculos(:,1);obstaculos(1,1)],[obstaculos(:,2);obstaculos(1,2)],'LineWidth',2,'color', 'black');
end
% line([obstaculos(:,1);obstaculos(1,1)],[obstaculos(:,2);obstaculos(1,2)],'LineWidth',2,'color', 'black');
% plotar_curva2(E{num_obs_aux+1},obstaculos);

%%
% % for i=1:length(N)
% %     plot(N{i}(:,1),N{i}(:,2),'line','.','color','red');
% % end
end
figure(1);
end