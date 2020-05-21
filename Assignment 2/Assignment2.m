function assignment_2()
clear all;
close all;
clf;
clc

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

fetchBase = transl(0,0,0);
workspace = [-2 2 -2 2 -0.1 3];
name = 'Robot';
robot = Fetch(fetchBase, workspace, name);
q = [0 0 0 0 0 0 0];
robot.model.plot(q, 'workspace', workspace)
end

function obj = CreateObject(file, pos)
    splitLine = split(file, '.');
    splitLine = splitLine(1);
    obj.name = splitLine{:};
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
end