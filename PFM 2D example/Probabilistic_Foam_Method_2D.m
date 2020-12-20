%% @author Luís B P Nascimento
%% lbruno@ufrn.edu.br
%% https://link.springer.com/article/10.1007/s10846-019-01134-7
%%    Probabilistic Foam Method - 2D


clear;
clc;
figure(2);
clf('reset');

figure(1);
clf('reset');

%% Initial parameters
%%
R=0.3; % Minimum Radius
K = 4; % Number of children bubbles for the Parent bubble with radius R

%init = [-17, -17];
%goal = [13,16];

init = [1.68, -9.89];
goal= [-2.36, 1.855];
[N, circles, E, V, nObstacle] = map(init,goal, 2); % loading the map
% N - vertices of the obstacles
% circles - number pof circles on the map
obstacles{2}= N;

foam = []; % list which contains the center of bubbles
position_of_parent_bubbles= []; % Position of the parent bubble based on the elements of the Foam.

foam{1} = init; %first bubble
list_radius = [eucl_dist(init,nearest_point(obstacles, circles,  init))]; %list of radius
count_generations =1;
count_bubbles=1; 
goal_was_found = 0;


%% ------------------ START ---------------
%%
figure(1)
title('probabilistic foam')
circle2(init,list_radius(1)); % plot the first bubble - Initial Bubble
tic;
while goal_was_found==0
    if(count_generations> length(foam)) %
        'You need decrease the value of R -> Minimum Radius'
        break;
    else
        parent_bubble = foam{count_generations}; % Selecting the parent bubble
    end
    
   % calculating the radius of the parent bubble
    radius_parent = eucl_dist(parent_bubble, nearest_point(obstacles, circles, parent_bubble));    
    np = ceil(K * (radius_parent/R)); % Maximum number of children bubbles for this parent bubble
    
    
    %% Sampling points on the surface of the parent bubble
    pt_rand = normrnd(0,1, [np, 2]);    
    radius_rnd = sqrt(pt_rand(:,1).^2 + pt_rand(:,2).^2);
    sampled_points = [((radius_parent./radius_rnd).*pt_rand(:,1))+parent_bubble(1) ((radius_parent./radius_rnd).*pt_rand(:,2)+parent_bubble(2))];
    sampled_points=sampled_points';
    
    
    %% while there are points on the parent bubble to be analised
    while(np>0)        
        sample = sampled_points(:,1); % Select a point sampled on the parent bubble        
        
        % Check intersections between the sample and the whole foam
        % Check if the point was not sampled in regions covered for other bubbles
        contIntersecao = 0;
        for cntIntersec=length(foam):-1:1
           if ~(foam{cntIntersec} == parent_bubble)
              if eucl_dist(foam{cntIntersec}, sample) < list_radius(cntIntersec)
                contIntersecao = contIntersecao+1;
                sampled_points(:, 1) = []; 
                np=np-1;
                break;
              end
           end
        end
        
        if (contIntersecao == 0)  % The sampled point
            pontoMaisProximo_aux = nearest_point (obstacles,circles, sample); %ponto mais proximo dos obs
            radius_new_bubble = eucl_dist(sample,pontoMaisProximo_aux);

            %% If some bubble encircles the goal configuration
            if(eucl_dist(goal,sample)<=radius_new_bubble) 
                count_bubbles= count_bubbles+1;

                position_of_parent_bubbles{count_bubbles} = count_generations;
                foam{count_bubbles} = sample;
                list_radius = [list_radius;radius_new_bubble];

                circle2(sample,radius_new_bubble);

                goal_was_found=1; % path found!
                toc
                break;               
            else
                if(radius_new_bubble>=R) % Minimum radius condition
                    count_bubbles= count_bubbles+1;

                    position_of_parent_bubbles{count_bubbles} = count_generations;
                    foam{count_bubbles} = sample;
                    list_radius = [list_radius;radius_new_bubble];

                    sampled_points(:,1) = [];
                    np=np-1;
                    
                    %% UNCOMMENT THIS LINE BELLOW TO SEE THE PROBABILISTIC FOAM - it may be slow...
                    %circle2(sample,radius_new_bubble);
                    
                    %% UNCOMMENT these lines bellow to see the grown foam  - it may be slow...
                    %figure(1)
                    %pause(0.01)
                else
                    % removing bubble smaller than minimum radius
                    sampled_points(:,1) = [];                
                    np=np-1;
                end
            end
        end
    end
    count_generations = count_generations+1;
end

aux_generation = count_bubbles;

path = []; 
radius_rosary = []; % radius of the bubbles of the rosary

figure(2)
hold on;
if(goal_was_found==1)
    while(aux_generation>1) % finding the Rosary - Sequence of bubbles from goal to initial configuration
        path = [path, foam{aux_generation}];
        radius_rosary = [radius_rosary;list_radius(aux_generation)];        
        circle2(foam{aux_generation},list_radius(aux_generation));%plotting the rosary
        aux_generation = position_of_parent_bubbles{aux_generation};
    end
    
    % adding initial and goal configurations to the path
    path = [goal', path, init'];
    radius_rosary = [eucl_dist(goal,nearest_point (obstacles,circles, goal));radius_rosary;list_radius(1)];
    
    % plotting the initial bubble
    circle2(init,list_radius(1));
    
    
    %plotting the initial and configurations
    plot(goal(1),goal(2),'o','color','b','MarkerFaceColor','b');
    plot(init(1),init(2),'o','color',[0 0.5 0],'MarkerFaceColor',[0 0.5 0]);
    
    figure(2); 
    title('PFM Path')
    line(path(1,:),path(2,:),'LineWidth',2,'color', 'red'); % plottiong the path
    plot(goal(1),goal(2),'o','color','b','MarkerFaceColor','b');
    plot(init(1),init(2),'o','color',[0 0.5 0],'MarkerFaceColor',[0 0.5 0]);  
    
    'In order to plot the probabilistic foam  at the end or during the search, please see lines 109 - 114'
end

