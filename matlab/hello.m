clear all
close all
clc
P1 = [ 15 15 15.1 15.4 16 16 15.8  15.3   15;...
        38 35 30   28   25 15 10    5.2    3];
P2 = [ 0 5  8  13 15 20 25 30 36;...
       0 10 15 16 18 20 26 35 36];
P3 = [ 38 36 30 25 22 16 12 8  5;...
       0  5  10 15 18 24 28 32 36];
xinit_agent = [P1(1,1) P2(1,1) P3(1,1);P1(2,1) P2(2,1) P3(2,1);0 0 0];
    
for i=1:3
   x{i} =  xinit_agent(:,i);
end

for i=1:3
    i
    if i==1
        P = P1;
    elseif i==2
        P = P2;
    else
        P = P3;
    end
   [xsim, usim,x_ref, y_ref] = Controller(P);
   for j=2:length(xsim)
      x{i} = [x{i} xsim(:,j)];
   end
end

%Obstacles statiques
Obstacles = [11 20 20 32; 11 15 30 30];
taille = size(Obstacles);
qtObstacles = taille(2);

for i = 1:qtObstacles
    axis([0 40 0 40])
    hold on 
    plot(Obstacles(1,i), Obstacles(2,i),'o','MarkerSize',18, 'MarkerEdgeColor','r','MarkerFaceColor','r');
end

axis([0 40 0 40])
hold on 
grid on
scatter(x{1}(1,:), x{1}(2,:),'filled')
hold on
scatter(x{2}(1,:), x{2}(2,:),'filled')
hold on
scatter(x{3}(1,:), x{3}(2,:),'filled')

