clear all
clc
P1 = [15 15 15.1 15.4 16 16 15.8 15.3 15;...
      38 35 30   28   25 15 10   5.2  3];

[xsim, usim,x_ref, y_ref] = Controller(P1);

Obstacles = [11 19 20 32; 11 20 30 30];
taille = size(Obstacles);
qtObstacles = taille(2);

for i = 1:qtObstacles
    axis([0 40 0 40])
    hold on 
    plot(Obstacles(1,i), Obstacles(2,i),'o','MarkerSize',18, 'MarkerEdgeColor','r','MarkerFaceColor','r');
end

grid on
scatter(xsim(1,:), xsim(2,:),'filled')
hold on
scatter(x_ref(1,:), y_ref(1,:),'filled')





