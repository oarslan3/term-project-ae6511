t = 0:pi/10:2*pi;


[X,Y,Z] = cylinder(1);


Z = 5*Z;
surf(X,Y,Z)
axis square

theta = pi/2;

R = [ 0  0 1;
     -1  0 0;
      0 -1 0];
 
 

for k = 1:size(X,2)
    rp = [X(1,k) Y(1,k) Z(1,k)]*R';
    X(1,k) = rp(1);
    Y(1,k) = rp(2);
    Z(1,k) = rp(3);
    
    rp = [X(2,k) Y(2,k) Z(2,k)]*R';
    X(2,k) = rp(1);
    Y(2,k) = rp(2);
    Z(2,k) = rp(3);
 end
figure
surf(X,Y,Z)
axis square

hold on;
vertices = [X(1,:)' Y(1,:)' Z(1,:)'];
faceList = [1:1:size(X,2), 1];

p = patch('Faces',faceList,'Vertices', vertices,'FaceColor','b');
grid on;    
hold on;

vertices = [X(2,:)' Y(2,:)' Z(2,:)'];
p = patch('Faces',faceList,'Vertices', vertices,'FaceColor','r');

