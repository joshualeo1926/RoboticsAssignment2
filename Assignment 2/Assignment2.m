function assignment_2()
clear all;
close all;
clf;
clc

set(0, 'DefaultFigureWindowStyle', 'docked')

currentFile = mfilename( 'fullpath' );
[pathstr,~,~] = fileparts( currentFile );
pathstr = fullfile(pathstr , '..','PLY', 'WorkBench.ply');

fetchBase = transl(0,0,0);
workspace = [-4 4 -4 4 -4 4];
name = 'Robot';
robot = Fetch(fetchBase, workspace, name);

workBenchPos = transl(1, 1, 1);
workbench = CreateObject(pathstr, workBenchPos)
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
end