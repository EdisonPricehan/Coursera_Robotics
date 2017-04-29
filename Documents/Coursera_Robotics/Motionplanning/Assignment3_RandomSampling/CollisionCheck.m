function out = CollisionCheck (fv1, fv2)
% Determine if two sets of triangular faces overlap

n1 = size (fv1.faces, 1);
n2 = size (fv2.faces, 1);

for i = 1:n1
    P1 = fv1.vertices(fv1.faces(i,:), :);
    for j = 1:n2
        P2 = fv2.vertices(fv2.faces(j,:), :);
        
        if (triangle_intersection(P1,P2))
            out = true;
            return;
        end
    end
end

out = false;
end

function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
flag = false;
for k = 1:3
    if PointInTriangle(P1, P2(k,:)) 
        flag = true;
        return
    end
end
 
for k = 1:3
    if PointInTriangle(P2, P1(k,:))
        flag = true;
        return
    end
end

% *******************************************************************
end

function flag = PointInTriangle(T,p)
flag = false;

V1 = T(1,:);
V2 = T(2,:);
V3 = T(3,:);
a = V2 - V3;
b = V1 - V3;
c = p - V3;

dot00 = dot(a,a);
dot01 = dot(a,b);
dot02 = dot(a,c);
dot11 = dot(b,b);
dot12 = dot(b,c);

denom = (dot11 * dot00 - dot01 * dot01);
u = (dot11 * dot02 - dot01 * dot12) / denom;
v = (dot12 * dot00 - dot02 * dot01) / denom;

if (u >= 0) && (v >= 0) && (u + v <= 1)
    flag = true;
end

end