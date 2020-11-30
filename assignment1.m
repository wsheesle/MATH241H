%Question 1
t=sym('t');
r=[(t^2)*(exp((3*t)+4)), t^3, 4-(t^2)];
assume(t,'positive')
T=diff(r)/norm(diff(r));
N=diff(T)/norm(diff(T));
D=dot(T,N);
S=simplify(D)
%Output = 0, therefore T and N are perpendicular.
%%
%Question 2
P1=[-2,-3,1];
P2=[0,1,-2];
P3=[1,-3,3];
P4=[-2,5,2];
P5=[-2,2,3];
P6=[-3,1,-4];
d1=P2-P1;
d2=P3-P1;
d3=P5-P4;
d4=P6-P4;
n1=cross(d1, d2)
n2=cross(d3, d4)
v=cross(n1, n2)
x=sym('x');
y=sym('y');
z=sym('z');
V=[x,y,z];
z1=solve(dot(n1,V-P1), z)
z2=solve(dot(n2,V-P4), z)
eqns=[z1==0, z2==0]; %Sets z-value=0 for the planes
[X, Y]=solve(eqns,[x y]); %Solves system for x and y, stores as vector X,Y
P0=[X,Y,0]; %Point of intersection
t=sym('t');
L=P0+(v*t)
fsurf([z1,z2],[-5,0,-5,0])
hold on
fplot3(L(1),L(2),L(3),[-.01,.01],'r','LineWidth',4)
hold off
%%
%Question 3
t=0:0.02:7*pi; %inputs values from 0 to 7pi in increments of 1/50, forming 350pi values
x=sin(t);
y=sin(2*t);
z=sin(3*t);
length=0;
for k=1:350*pi %adds up each line segment one at a time; since t goes from 0 to 7pi in increments of 1/50, there are a total of 350pi values
    dx=x(k+1)-x(k); %difference in x components at the kth value of the t values
    dy=y(k+1)-y(k);
    dz=z(k+1)-z(k);
    length=length+norm([dx,dy,dz]); %adding current length with newest segment
end
length

syms t;
r=[sin(t),sin(2*t),sin(3*t)];
exact=double(int(norm(diff(r)),t,0,7*pi))
error=double(abs(length-exact))

fplot3(sin(t),sin(2*t),sin(3*t),[0,5*pi])