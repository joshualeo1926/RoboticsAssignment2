function assignment_2()
clear all;
close all;
clf;
clc;

set(0, 'DefaultFigureWindowStyle', 'docked')

currentFile = mfilename( 'fullpath' );
[pathstr,~,~] = fileparts( currentFile );

workBenchPath = fullfile(pathstr , '..', 'PLY', 'WorkBench.ply');
wrench1Path = fullfile(pathstr , '..', 'PLY', 'Wrench1.ply');
wrench2Path = fullfile(pathstr , '..', 'PLY', 'Wrench2.ply');
wrench3Path = fullfile(pathstr , '..', 'PLY', 'Wrench3.ply');

workBenchPos = transl(0, 1, 1);
wrench1Pos = transl(-0.1, 0.75, 0.8);
wrench2Pos = transl(0, 0.75, 0.8);
wrench3Pos = transl(0.1, 0.75, 0.8);

workbench = CreateObject(workBenchPath, workBenchPos);
wrench1 = CreateObject(wrench1Path, wrench1Pos);
wrench2 = CreateObject(wrench2Path, wrench2Pos);
wrench3 = CreateObject(wrench3Path, wrench3Pos);

environment = [workbench, wrench1, wrench2, wrench3];

fetchBase = transl(0,1.3,0.5);
workspace = [-1 1 -0.5 1.5 -0.1 2];
name = 'Robot';
robot = Fetch(fetchBase, workspace, name);
q = [0 0 0 0 0 0 0];
robot.model.plot(q, 'workspace', workspace, 'noarrow', 'scale', 0)
qMatrix = robot.Move(transl(0, 0, 2));

faceNormals = zeros(size(workbench.f,1),3);
for faceIndex = 1:size(workbench.f,1)
    v1 = workbench.verts(workbench.f(faceIndex,1)',:);
    v2 = workbench.verts(workbench.f(faceIndex,2)',:);
    v3 = workbench.verts(workbench.f(faceIndex,3)',:);
    faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
end

for i = 1:50
    q = qMatrix(i, :);
    result = IsCollision(robot.model, q, workbench.f, workbench.verts, faceNormals);
    if(result == 0)
        robot.model.plot(q)
    elseif(result == 1)
        break 
        r = q
    end
end


hold on;
plot3(workBenchPos(1, 4), workBenchPos(2, 4), workBenchPos(3, 4) + 0.1, 'r.')
plot3(workBenchPos(1, 4)+0.3, workBenchPos(2, 4), workBenchPos(3, 4) + 0.1, 'r.')
plot3(workBenchPos(1, 4)-0.3, workBenchPos(2, 4), workBenchPos(3, 4) + 0.1, 'r.')
hold off;

disp('DONE!')
end

function obj = CreateObject(file, pos)
    splitLine = split(file, '\');
    splitLine = splitLine(end);
    splitLine = split(splitLine, '.');
    splitLine = splitLine(1);
    obj.name = splitLine;
    [f, v, data] = plyread(file, 'tri');
    obj.vertexCount = size(v, 1);
    obj.f = f;
    midPoint = sum(v)/obj.vertexCount;
    obj.verts = v - repmat(midPoint, obj.vertexCount, 1);
    obj.vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;
    hold on;
    obj.mesh = trisurf(f, obj.verts(:, 1)+pos(1, 4), obj.verts(:, 2)+pos(2, 4),...
        obj.verts(:, 3)+pos(3, 4), 'FaceVertexCData', obj.vertexColours, 'EdgeLighting', 'flat');
    hold off;
    
    faceNormals = zeros(size(obj.f,1),3);
    for faceIndex = 1:size(obj.f,1)
        v1 = obj.verts(obj.f(faceIndex,1)',:);
        v2 = obj.verts(obj.f(faceIndex,2)',:);
        v3 = obj.verts(obj.f(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
        obj.fn = faceNormals;
    end
end