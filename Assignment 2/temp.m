
clear all;
clc;
close all;

%%
cam = webcam;
while(1)
    Image = snapshot(cam);
    targetIdentified = CameraScanner(Image);
end
%%
% currentFile = mfilename( 'fullpath' );
% [pathstr,~,~] = fileparts( currentFile );
% workBenchPos = transl(0, 1, 0.75);
% workBenchPath = fullfile(pathstr , '..', 'PLY', 'WorkBench.ply');
% workbench = CreateObject(workBenchPath, workBenchPos);
% 
%     teachModeValue = 1;
%     workspace = [-2 2 -2 2 -0.1 1.5];
%     name = 'Robot';
%     fetchBase = transl(0, 0, 0.5);
%     robot = Fetch(fetchBase, workspace, name);
%     initialQMatrix = deg2rad([92 -80 0 -100 0 85 0]);
%     robot.model.plot(initialQMatrix, 'workspace', workspace, 'noarrow', 'scale', 0)
%     hold on;
%     plot3(0.7105,    0.5376,    0.5215, 'ro')
%     hold off;

function identified = CameraScanner(scene)
    %Read Target
    Target = imread('StopSign1.jpg');
    Target_Image = rgb2gray(Target);

    %Read Scene
    Scene_Image = rgb2gray(scene);

    %Detect Features
    Target_Points = detectSURFFeatures(Target_Image);
    Scene_Points = detectSURFFeatures(Scene_Image);

    %Extract Points of Interest
    [Target_Features, Target_Points] = extractFeatures(Target_Image, Target_Points);
    [Scene_Features, Scene_Points] = extractFeatures(Scene_Image, Scene_Points);

    %Match Features
    Matched_Pairs = matchFeatures(Target_Features, Scene_Features);

    %Display Matched Features
    Matched_Target_Points = Target_Points(Matched_Pairs(:, 1), :);
    Matched_Scene_Points = Scene_Points(Matched_Pairs(:, 2), :);
    
    figure(2);
    showMatchedFeatures(Target_Image, Scene_Image, Matched_Target_Points, Matched_Scene_Points, 'montage');
    
    Matched_Scene_Points.Count;
    if(Matched_Scene_Points.Count >= 20)
        identified = true;
    else
        identified = false;
    end
end

function CreateLightCurtain()
    for i=0:0.5:2
        x = [1,2];
        y = [2,1];
        z = [i,i];
        hold on
        plot3(x,y,z,'--rs','LineWidth',0.1);
        hold off
    end
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