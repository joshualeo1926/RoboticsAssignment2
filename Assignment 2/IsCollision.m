%% IsCollision
% This is based upon Lab 5 exercises
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function [result, intersectP, i] = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
%     q = qMatrix(qIndex,:);
    
    % Get the transform of every joint (i.e. start and end of every link)  
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1) 
            if tr(1,4,i) ~= tr(1,4,i+1) && tr(2,4,i) ~= tr(2,4,i+1) && tr(3,4,i) ~= tr(3,4,i+1) && 0
                for j = 1:8
                    ctr = tr(:,:,i);
                    ftr = tr(:,:,i+1);
                    p = [rand(1);  rand(1); rand(1)];
                    p1 = [ctr(1, 4); ctr(2, 4); ctr(3, 4)];
                    p2 = [ftr(1, 4); ftr(2, 4); ftr(3, 4)];
                    r = cross((p - p1), (p2 - p1));
                    s = cross(r, (p2-p1));
                    r = r/norm(r);
                    s = s/norm(s);

                    x = [-1 -1 0 1 1 1 0 -1];
                    y = [0 1 1 1 0 -1 -1 -1];

                    radius = 0.08; %0.07

                    s = radius*x(j)*s;
                    r = radius*y(j)*r;

                    nctr = tr(:,:,i) + transl(s(1), s(2), s(3)) + transl(r(1), r(2), r(3));
                    nftr = tr(:,:,i+1) + transl(s(1), s(2), s(3)) + transl(r(1), r(2), r(3));

                    vertOnPlane = vertex(faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,nctr(1:3,4)',nftr(1:3,4)'); 
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                        %plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                        %display('Intersection');
                        result = true;
                        if returnOnceFound
                            return
                        end
                    end
                end
            else
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4, i+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    %plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    %display('Intersection');
                    result = true;
                    if returnOnceFound
                        return
                    end
                end
            end
        end    
    end
end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% GetLinkPoses
function [ transforms ] = GetLinkPoses( q, robot)
%q - robot joint angles
%robot -  seriallink robot model
%transforms - list of transforms

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);

    transforms(:,:,i + 1) = current_transform;
end
end

