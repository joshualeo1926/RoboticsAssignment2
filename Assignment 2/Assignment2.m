function assignment_2()
clear all;
close all;
clf;
clc;

set(0, 'DefaultFigureWindowStyle', 'docked')

% Set all locations
workspace = [-1 1 -0.5 1.5 -0.1 2];
workBenchPos = transl(0, 1, 1);
wrench1Pos = transl(-0.1, 0.75, 0.8);
wrench2Pos = transl(0, 0.75, 0.8);
wrench3Pos = transl(0.1, 0.75, 0.8);
fetchBase = transl(0, 1.2, 0.5);

% Get path to each PLY file
currentFile = mfilename( 'fullpath' );
[pathstr,~,~] = fileparts( currentFile );
workBenchPath = fullfile(pathstr , '..', 'PLY', 'WorkBench.ply');
wrench1Path = fullfile(pathstr , '..', 'PLY', 'Wrench1.ply');
wrench2Path = fullfile(pathstr , '..', 'PLY', 'Wrench2.ply');
wrench3Path = fullfile(pathstr , '..', 'PLY', 'Wrench3.ply');

% Create objects
workbench = CreateObject(workBenchPath, workBenchPos);
wrench1 = CreateObject(wrench1Path, wrench1Pos);
wrench2 = CreateObject(wrench2Path, wrench2Pos);
wrench3 = CreateObject(wrench3Path, wrench3Pos);

% Create a list of all objects in the envrionment
environment = [workbench, wrench1, wrench2, wrench3];

% Initialise robot
name = 'Robot';
robot = Fetch(fetchBase, workspace, name);
q = deg2rad([92 -80 0 -100 0 85 0]);
robot.model.plot(q, 'workspace', workspace, 'noarrow', 'scale', 0)
gui = GUI();
% Mail loop
step = 1;
while 1
    %read GUI
    %check ESTOP
    
    %check light curtain
    
    if step == 1
        [actionComplete, basePos] = robot.MoveBase(transl(0.5, 1.2, 0.5));
        %check base pos for collision befor plotting 
            %if collision -> wait for no collision
            %else -> plot
        robot.model.base = basePos;
        robot.model.plot(robot.model.getpos)
        step = step + actionComplete;
    elseif step == 2
        [actionComplete, qMatrix] = robot.Move(transl(0.8, 0.75, 0.6));
        [collision, intersectP] = robot.IsArmCollision(qMatrix, environment);
        %check arm for collisions befor plotting
            %if collisions -> move around
            %else -> plot
        if collision == 1
            %recalculate step
            while 1
                transform = GetLinkPoses(qMatrix, robot.model);
                for i = 1:size(transform,3)-1
                    linkTransform = transform(:, :, i);
                    nextLinkTransform = transform(:, :, i+1);
                    if intersectP(1) >= linkTransform(1, 4) && intersectP(1) < nextLinkTransform(1, 4)...
                            && intersectP(2) >= linkTransform(2, 4) && intersectP(2) < nextLinkTransform(2, 4)...
                            && intersectP(3) >= linkTransform(3, 4) && intersectP(3) < nextLinkTransform(3, 4)
                       disp(['colliding with link ', i]) 
                    end 
                end
                disp(['did not find link']) 
                intersectP
                for i = 1:size(transform,3)
                    linkTransform = transform(:, :, i);
                    disp(['link ', num2str(i), ' - ', num2str(linkTransform(1, 4)), ' : ', ...
                        num2str(linkTransform(2, 4)), ' : ', num2str(linkTransform(3, 4))])
                end 
                while 1
                           
                end
            end
        else
            robot.model.plot(qMatrix)
            step = step + actionComplete;
        end
    elseif step == 3
        break
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
    obj.verts(:, 1) = obj.verts(:, 1) + pos(1, 4);
    obj.verts(:, 2) = obj.verts(:, 2) + pos(2, 4);
    obj.verts(:, 3) = obj.verts(:, 3) + pos(3, 4);
    hold on;
    obj.mesh = trisurf(f, obj.verts(:, 1), obj.verts(:, 2),...
        obj.verts(:, 3), 'FaceVertexCData', obj.vertexColours, 'EdgeLighting', 'flat');
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