function IP_ex_bezier
    function y=b(i,n,t)  %Bernstein polynomials
        y=prod(1:n)/(prod(1:i)*prod(1:n-i))*(1-t)^(n-i)*t^i;
    end

    function w=setpoint(t) %Bezier curve
        w=0;
        for i=0:n,
            w=w+b(i,n,t)*P(:,i+1);
        end
    end
%----------      Main     -----------------


axis([-1 11 -1 11]); axis square; hold on;
A1=[2 4 2 ; 0 2 7 ];   %draw a polytope, in case you do not have MPT3 toolbox installed
A2=[7 8 3 ; 2 3 10];
fill(A1(1,:),A1(2,:),'black');
fill(A2(1,:),A2(2,:),'black');
x=[0;0;0;1]; 

%set of control points

% P= [ 1  2   8  8;
%      1 10   0  8 ];
% 
%  P= [ 1  1 1 1   2       8      8;
%       1  4 7 9   10      0      8 ];

P= [ 1  1 1 1   2    3 4 5 5 7   8   10    8;
     1  4 7 9   10   8 6 4 1 0   0    1    8 ];
 
 n=length(P)-1;
 
plot(P(1,:), P(2,:), 'ored', 'LineWidth', 4);

dt=0.01;
for t=0:dt:1,
    w=setpoint(t);
    plot(w(1),w(2), '.blue');
    drawnow();  % draw the trajectory
end
end
