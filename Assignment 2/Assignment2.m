
clear all;
close all;
clf;
clc;

set(0, 'DefaultFigureWindowStyle', 'docked')

% Set all locations
workspace = [-1 1 -2.5 1.5 -0.1 2];
workBenchPos = transl(0, 1, 0.75);
wrench1Pos = transl(-0.1, 0.75, workBenchPos(3, 4) - 0.2);
wrench2Pos = transl(0, 0.75, workBenchPos(3, 4) - 0.2);
wrench3Pos = transl(0.1, 0.75, workBenchPos(3, 4) - 0.2);
fetchBase = transl(0, -2, 0.5)*trotz(pi/2);

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
%%
% Mail loop
step = 1;
while 1
    %read GUI
    pause(0.00001)
    powerState = gui.GetPowerValue()
    while(powerState == "On")
        pause(0.00001)
        teachMode = gui.GetTeachModeValue();
        if(teachMode == 1)
            clc
            clf
            name = 'Robot';
            workspace = [-1 1 -1 1 -0.1 1.5];
            fetchBase = transl(0, 0, 0.52);
            robot = Fetch(fetchBase, workspace, name);
            q = deg2rad([92 -80 0 -100 0 85 0]);
    robot.model.plot(q, 'workspace', workspace, 'noarrow', 'scale', 0);
            while(teachMode == 1)
                pause(0.00001)
                teachMode = gui.GetTeachModeValue();
                joint1 = gui.GetJoint1Value();
                joint2 = gui.GetJoint2Value();
                joint3 = gui.GetJoint3Value();
                joint4 = gui.GetJoint4Value();
                joint5 = gui.GetJoint5Value();
                joint6 = gui.GetJoint6Value();
                joint7 = gui.GetJoint7Value();
                qMatrix = [joint1 joint2 joint3 joint4 joint5 joint6 joint7];
                robot.model.plot(qMatrix);
            end
        end
        % If Start is pressed start process
        pause(0.00001)
        startValue = gui.GetStartValue();
        if(startValue == 1)
            paused = 1;
            %check ESTOP
            pause(0.00001)
            EStopValue = gui.GetEStopValue();
            if(EStopValue == 1)
                while(paused == 1)
                    pause(0.00001)
                    EStopValue = gui.GetEStopValue();
                    if(EStopValue == 0)
                    paused = 0;
                    end
                end
            end
            
            % move to workbench
            if step == 1
                destination = transl(workBenchPos(1, 4), workBenchPos(2, 4) - 0.95, fetchBase(3, 4))*trotz(pi/2);
                [actionComplete, basePos] = robot.MoveBase(destination);
                %check base pos for collision befor plotting 
                %if collision -> wait for no collision
                %else -> plot
                robot.model.base = basePos;
                robot.model.plot(robot.model.getpos)
                step = step + actionComplete;
    
                % unfold arm
                % create move function that takes in a joint state
            elseif step == 2
                    [actionComplete, qMatrix] = robot.Move(transl(0, 0.5, 0.75)*trotx(pi));
                    [collision, intersectP, i] = robot.IsArmCollision(qMatrix, environment);
                    %check arm for collisions befor plotting
                    %if collisions -> move around
                    %else -> plot
                    if collision == 1 && 0
                        i
                        intersectP
                    else
                        robot.model.plot(qMatrix)
                        step = step + actionComplete;
                    end
        
                    % pick up first wrench
            elseif step == 3
                [actionComplete, qMatrix] = robot.Move(wrench1Pos*trotx(pi));
                %[collision, intersectP, i] = robot.IsArmCollision(qMatrix, environment);
                robot.model.plot(qMatrix)
                step = step + actionComplete;
        
                % done
            elseif step == 4
                break
            end
        end
    end
end

hold on;
plot3(workBenchPos(1, 4), workBenchPos(2, 4), workBenchPos(3, 4) + 0.1, 'r.')
plot3(workBenchPos(1, 4)+0.3, workBenchPos(2, 4), workBenchPos(3, 4) + 0.1, 'r.')
plot3(workBenchPos(1, 4)-0.3, workBenchPos(2, 4), workBenchPos(3, 4) + 0.1, 'r.')
hold off;

disp('DONE!')


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