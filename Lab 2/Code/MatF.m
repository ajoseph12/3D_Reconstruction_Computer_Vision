function F=MatF(p1,p2)
% p1,p2 : vectors of coordinates ...x...

% initialisation
A = [];
N=8;

% Write a martix of homogeneous linear system Af=0
p1 = [p1; ones(1,N)]
p2 = [p2; ones(1,N)]
for i = 1:N
   temp = []
   temp = p2(:,i)*p1(:,i)'
   temp = reshape(temp.',1,[])
   A = [A;temp]
end

% Solving the linear system for the estimate of F by SVD
% Ist step: Linear solution
[U,S,V] = svd(A)
Fest = reshape(V(:,end),3,3)

% 2nd step: Constraint enforcement
Fest=Fest';
[U,S,V] = svd(Fest);
F = U*diag([S(1,1) S(2,2) 0])*V';


