function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
flag = false;
for i = 1:3
    if PointInTriangle(P1, P2(i,:)) 
        flag = true;
        return
    end
end
 
for i = 1:3
    if PointInTriangle(P2, P1(i,:))
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