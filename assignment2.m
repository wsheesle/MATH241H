%Question 1
syms x y z
f=cos(x^2 + 5*y) + x*(y^3) - exp(-x*y);
u=[-2,1,8];
uv= u/norm(u);
grad=gradient(f, [x,y,z]);
Duf=dot(grad, uv);

rate=double(subs(Duf, [x, y, z], [2, 2, 2]))
maxrate=double(norm(subs(grad, [x, y, z], [2, 2, 2])))
%%
%Question 2
syms x y k t real
f=x*y
g=x^2/8+y^2/2-1;
[A,B,C]=solve(gradient(f) == k*gradient(g),g==0, [x,y,k]) %finds the points of interest

fsurf(x*y,[-3,3],'ShowContours','on') %shows the surface f with level curves
figure

[X,Y] = meshgrid(-5:.1:5,-5:.1:5);
Z=X.*Y;
grad = gradient(g);
U = subs(grad(1),[x y],{A B});
V = subs(grad(2),[x y],{A B});
quiver(A, B, U, V) %this plots the gradient vectors of g at the points of interest
hold on
grad1 = gradient(f);
U1 = subs(grad1(1),[x y],{A B});
V1 = subs(grad1(2),[x y],{A B});
quiver(A, B, U1, V1) %this plots the gradient vectors of f at the points of interest
hold on
%[T,h]=
contour(X,Y,Z,[-6 -4 -2 -1 -0.5 0.5 1 2 4 6],'ShowText','on','LineWidth',2);
hold on

fplot(sqrt(8)*cos(t),sqrt(2)*sin(t),[0 2*pi],'LineWidth',2) %this plots the constraint g
hold on
scatter(A,B,150,'filled') %This will highlight the point of interest we found from solving the system of equations
text(2.5,1,'MAX','FontSize',15) %Edited line
hold off

%Points of interest are (-2, -1), (-2, 1), (2, -1), (2,1).
%(2,1) is a MAX, as f values at critical points are -2 and 2, 2 > -2 and f(2,1) = 2
%%
%Question 3
syms x y h
syms x y
f(x,y)=x^2+y^2+3*x-4*y+2;
fx=diff(f,x);
fy=diff(f,y);
L(x,y)=(subs(fx, [x, y], [.2, .2])*(x - .2))+(subs(fy, [x, y], [.2, .2])*(y - .2))+subs(f, [x,y], [.2, .2]) %Edited line

fsurf(x^2+y^2+3*x-4*y+2,[0 1 0 1],'ShowContours','on ','HandleVisibility','off') %This plots the surface
hold on
fsurf(L(x,y),[0 1 0 1],'HandleVisibility','off') %This plots the tangent plane
scatter3(.2,.2,f(.2,.2),200,'filled') %This plots the location of the tangent plane
scatter3(.9,.05,f(.9,.05),200,'filled') %This plots the exact location for the surface
scatter3(.9,.05,L(.9,.05),200,'filled') %Edited line
legend([{'Tangent plane location'},{'Exact value'},{'Approximation'}],'Location','northwest')
view(75,15)
hold off

error=abs(f(.9,.05)-L(.9,.05))
%%
%Question 4
syms x y h
f(x,y)=(x^3*y-x*y^3)/(x^2+y^2);
fx=diff(f,x)
fy=diff(f,y)

fxzero=limit(((f(h,0)-0)/h), h, 0)
fyzero=limit(((f(0,h)-0)/h), h, 0)

fxy=limit(((fx(0,h)-fxzero)/h), h, 0)
fyx=limit(((fy(h,0)-fyzero)/h), h, 0)
%fxy(0,0) =/= fyx(0,0), Clairaut's theorem cannot be applied

fsurf(f(x,y),[-.5, .5, -.5, .5])