[x, y, z] = ellipsoid(0,0,0,5.9,3.25,3.25,30);
[row,col] = size(x);
a = zeros(row,col);
b = zeros(row,col);
c = zeros(row,col);
for aux=1:1:row
 for aux2=1:1:col
     
   d = rotx(-0.0043 )*roty(0.4772)*rotz(1.5127)*[x(aux,aux2);y(aux,aux2);z(aux,aux2)];
   a(aux,aux2) = d(1,1);
   b(aux,aux2) = d(2,1);
   c(aux,aux2) = d(3,1);
 end
end
figure
surf(a, b, c,'FaceAlpha',0.1)
hold on 
surf(x,y,z,'FaceAlpha',0.1)
axis equal