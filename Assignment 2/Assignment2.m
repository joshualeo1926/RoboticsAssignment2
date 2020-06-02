%{
    PLY FILES:
        PART                                |CREATOR
        ====================================|==========
        FETCHROBOT(arm links and base)      |
        WORKBENCH                           |Joshua Leo
        WRENCH (1,2 & 3)                    |Joshua Leo
        GANTRY                              |Joshua Leo
        GANTRY MOTOR                        |Joshua Leo
    
    CODE:
        FUNCTOION/FILE                      |CREATOR
        ====================================|==========
        IsCollion                           |41013 robotics/Dr.Gavin Paul 
        IsIntersectionPointInsideTriangle   |41013 robotics/Dr.Gavin Paul
        LinePlaneIntersection               |41013 robotics/Dr.Gavin Paul
        IsIntersectionPointInsideTriangle   |41013 robotics/Dr.Gavin Paul
        GetLinkPoses                        |41013 robotics/Dr.Gavin Paul 
        Move/RMRC                           |41013 robotics/Dr.Gavin Paul
%}

clear all;
close all;
clf;
clc;

set(0, 'DefaultFigureWindowStyle', 'docked')

% Set all locations
workspace = [-2 2 -2.5 1.5 -0.1 3.5];
workBenchPos = transl(0, 1, 0.75);
wrench1Pos = transl(-0.1, 0.75, workBenchPos(3, 4) - 0.2);
wrench2Pos = transl(0, 0.75, workBenchPos(3, 4) - 0.2);
wrench3Pos = transl(0.1, 0.75, workBenchPos(3, 4) - 0.2);
gantryPos = transl(0, -0.25, 0.1);
%gantryMotorPos = transl(-1.4, -0.25, 0.57);
gantryMotorPos = transl(-1.4, -0.25, 1.47);
fetchBase = transl(0, -2, 0.5)*trotz(pi/2);

% Get path to each PLY file
currentFile = mfilename( 'fullpath' );
[pathstr,~,~] = fileparts( currentFile );
workBenchPath = fullfile(pathstr , '..', 'PLY', 'WorkBench.ply');
wrench1Path = fullfile(pathstr , '..', 'PLY', 'Wrench1.ply');
wrench2Path = fullfile(pathstr , '..', 'PLY', 'Wrench2.ply');
wrench3Path = fullfile(pathstr , '..', 'PLY', 'Wrench3.ply');
gantryPath = fullfile(pathstr , '..', 'PLY', 'gantry.ply');
gantryMotorPath = fullfile(pathstr , '..', 'PLY', 'gantrymotor2.ply');

% Create objects
workbench = CreateObject(workBenchPath, workBenchPos);
wrench1 = CreateObject(wrench1Path, wrench1Pos);
wrench2 = CreateObject(wrench2Path, wrench2Pos);
wrench3 = CreateObject(wrench3Path, wrench3Pos);
gantry = CreateObject(gantryPath, gantryPos);
gantryMotor = CreateObject(gantryMotorPath, gantryMotorPos);

% Create a list of all objects in the envrionment
environment = [workbench, wrench1, wrench2, wrench3];

% Initialise robot
name = 'Robot';
robot = Fetch(fetchBase, workspace, name);
initialQMatrix = deg2rad([92 -80 0 -100 0 85 0]);
robot.model.plot(initialQMatrix, 'workspace', workspace, 'noarrow', 'scale', 0)
gui = GUI();
%%
% Mail loop
step = 1;
get_matrix = 1;
itteration = 1;
while 1
    %read GUI
    pause(0.00001)
    powerState = gui.GetPowerValue();
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
                pause(0.00001)
                teachMode2 = gui.GetTeachMode2Value();
                if(teachMode2 == 1)
                    pause(0.00001)
                    
                end
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
            pause(0.00001)
            
            % Move Gantry Crane
            obstructionValue = -1.4 + 2.8 * gui.GetBlockSlider()/100;
            if obstructionValue < -1.4
                obstructionValue = -1.4;
            elseif obstructionValue > 1.35
                obstructionValue = 1.35;
            end
            gantryMotor.mesh.Vertices(:, 1) = gantryMotor.verts(:, 1) + 1.4 + obstructionValue;
            
            % ========== FETCH CONTROLL ============
            
            % move to workbench
            if step == 1
                if get_matrix == 1
                    destination = transl(workBenchPos(1, 4), workBenchPos(2, 4) - 0.95, fetchBase(3, 4))*trotz(pi/2);
                    basePos = robot.MoveBase(destination);
                    get_matrix = 0;
                end
                if itteration <= size(basePos, 3)
                    collision = robot.CheckBaseCollision(gantryMotor);
                    if collision == 0   
                        robot.model.base = basePos(:, :, itteration);
                        itteration = itteration + 1;
                        robot.model.plot(robot.model.getpos)
                    end
                elseif itteration > size(basePos, 3)
                    itteration = 1;
                    get_matrix = 1;
                    step = step + 1;
                end
            
            % unfold arm
            elseif step == 2
                if get_matrix == 1
                    qMatrix = robot.ArmRMRCJoints(deg2rad([92 -50 0 -115 0 15 0]));
                    get_matrix = 0;
                end
                if itteration <= size(qMatrix, 1)
                    robot.model.plot(qMatrix(itteration, :))
                    itteration = itteration + 1;
                elseif itteration > size(qMatrix, 1)
                    itteration = 1;
                    get_matrix = 1;
                    step = step + 1;
                end
                
            elseif step == 3
                if get_matrix == 1
                    qMatrix = robot.ArmRMRCJoints(deg2rad([92 30 0 -100 0 80 0]));
                    get_matrix = 0;
                end
                if itteration <= size(qMatrix, 1)
                    robot.model.plot(qMatrix(itteration, :))
                    itteration = itteration + 1;
                elseif itteration > size(qMatrix, 1)
                    itteration = 1;
                    get_matrix = 1;
                    step = step + 1;
                end

            % move to above table
            elseif step == 4
                if get_matrix == 1
                    qMatrix = robot.ArmRMRCPos(transl(0, 0.5, 0.75)*trotx(pi));
                    get_matrix = 0;
                end
                if itteration <= size(qMatrix, 1)
                    robot.model.plot(qMatrix(itteration, :))
                    itteration = itteration + 1;
                elseif itteration > size(qMatrix, 1)
                    itteration = 1;
                    get_matrix = 1;
                    step = step + 1;
                end                
                %[collision, intersectP, i] = robot.IsArmCollision(qMatrix, environment);
                %check arm for collisions befor plotting
                %if collisions -> move around
                %else -> plot
                %if collision == 1 && 0
                %    i
                %    intersectP
                %else
                %    robot.model.plot(qMatrix)
                %    step = step + actionComplete;
                %end
                
            % move closer
            initialBasePose = robot.model.base;
            elseif step == 5
                if get_matrix == 1
                    destination = transl(workBenchPos(1, 4), workBenchPos(2, 4) - 0.65, fetchBase(3, 4))*trotz(pi/2);
                    basePos = robot.MoveBase(destination);
                    get_matrix = 0;
                end
                if itteration <= size(basePos, 3)
                    collision = robot.CheckBaseCollision(gantryMotor);
                    if collision == 0   
                        robot.model.base = basePos(:, :, itteration);
                        itteration = itteration + 1;
                        robot.model.plot(robot.model.getpos)
                    end
                elseif itteration > size(basePos, 3)
                    itteration = 1;
                    get_matrix = 1;
                    step = step + 1;
                end
                
            % pick up first wrench
            elseif step == 6
                if get_matrix == 1
                    qMatrix = robot.ArmRMRCPos(wrench1Pos*trotx(pi));
                    get_matrix = 0;
                end
                if itteration <= size(qMatrix, 1)
                    robot.model.plot(qMatrix(itteration, :))
                    itteration = itteration + 1;
                elseif itteration > size(qMatrix, 1)
                    itteration = 1;
                    get_matrix = 1;
                    step = step + 1;
                end  
            
            % pre placment position wrench 1
            elseif step == 7
                if get_matrix == 1
                    qMatrix = robot.ArmRMRCPos(transl(workBenchPos(1, 4)-0.3,...
                        workBenchPos(2, 4) - 0.1, workBenchPos(3, 4) + 0.08)*trotx(-pi/2));
                    get_matrix = 0;
                end
                if itteration <= size(qMatrix, 1)
                    robot.model.plot(qMatrix(itteration, :))
                    itteration = itteration + 1;
                elseif itteration > size(qMatrix, 1)
                    itteration = 1;
                    get_matrix = 1;
                    step = step + 1;
                end  
                
            % place wrench 1
            elseif step == 8
                if get_matrix == 1
                    qMatrix = robot.ArmRMRCPos(transl(workBenchPos(1, 4)-0.3,...
                        workBenchPos(2, 4), workBenchPos(3, 4) + 0.08)*trotx(-pi/2));
                    get_matrix = 0;
                end
                if itteration <= size(qMatrix, 1)
                    robot.model.plot(qMatrix(itteration, :))
                    itteration = itteration + 1;
                elseif itteration > size(qMatrix, 1)
                    itteration = 1;
                    get_matrix = 1;
                    step = step + 1;
                end 
                
            % pick up wrench 2
            % pre placment position wrench 2
            % place wrench 2
            
            % pick up wrench 3
            % pre placment position wrench 3
            % place wrench 3
            
            % go to saftey sign?
            
            % done
            elseif step == 9
                disp('DONE 1 !')
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