%{
    PLY FILES:
        PART                                |CREATOR
        ====================================|==========
        FETCHROBOT(arm links and base)      |ROS/GAZIBO - https://docs.fetchrobotics.com/gazebo.html
                                                        - https://github.com/fetchrobotics/fetch_gazebo
        WORKBENCH                           |Joshua Leo
        WRENCH (1,2 & 3)                    |Joshua Leo
        GANTRY                              |Joshua Leo
        GANTRY MOTOR                        |Joshua Leo
        FENCE                               |KW3DGraphics - https://www.turbosquid.com/3d-models/stadium-fence-3ds-free/545545
                                             Also used in assignment 1 by Johnson Nguyen
    IMAGES:
        IMAGE                               |CREATOR
        ====================================|==========
        floor                               |http://www.skyliteconstruction.com/
        StopSign1                           |https://www.amazon.com/Roll-Up-Compact-Reflective-Crossbracing-Included/dp/B008J4ZLW6
        BrickWall                           |Bernard Hermant - https://unsplash.com/s/photos/brick-wall
    
    CODE:
        FUNCTOION/FILE                      |CREATOR                            
        ====================================|==========
        IsCollion                           |41013 robotics/Dr.Gavin Paul
        IsIntersectionPointInsideTriangle   |41013 robotics/Dr.Gavin Paul
        LinePlaneIntersection               |41013 robotics/Dr.Gavin Paul
        IsIntersectionPointInsideTriangle   |41013 robotics/Dr.Gavin Paul
        GetLinkPoses                        |41013 robotics/Dr.Gavin Paul 
        Move/RMRC LAB9 - tutorial           |41013 robotics/Dr.Gavin Paul
%}

clear all;
close all;
clf;
clc;
lighting gouraud
lightangle(gca,-60,20)
set(0, 'DefaultFigureWindowStyle', 'docked')

% Set all locations
workspace = [-2 2 -2.5 1.5 -0.1 3.5];
workBenchPos = transl(0, 1, 0.75);
wrench1Pos = transl(-0.2, 0.75, workBenchPos(3, 4) - 0.2) * trotz(pi);
wrench2Pos = transl(0, 0.75, workBenchPos(3, 4) - 0.2) * trotz(pi);
wrench3Pos = transl(0.2, 0.75, workBenchPos(3, 4) - 0.2) * trotz(pi);
gantryPos = transl(0, 0.75, 0.05); % y was -0.25
gantryMotorPos = transl(-1.4, 0.75, 1.42); % y was -0.25
fetchBase = transl(0, -2, 0.5)*trotz(pi/2);
cubePos = transl(1.8, -3, 0);
fencePos = transl(0,0,0);

% Get path to each PLY file
currentFile = mfilename( 'fullpath' );
[pathstr,~,~] = fileparts( currentFile );
workBenchPath = fullfile(pathstr , '..', 'PLY', 'WorkBench2.ply');
wrench1Path = fullfile(pathstr , '..', 'PLY', 'Wrench1.ply');
wrench2Path = fullfile(pathstr , '..', 'PLY', 'Wrench2.ply');
wrench3Path = fullfile(pathstr , '..', 'PLY', 'Wrench3.ply');
gantryPath = fullfile(pathstr , '..', 'PLY', 'gantry.ply');
gantryMotorPath = fullfile(pathstr , '..', 'PLY', 'gantrymotor2.ply');
cubePath = fullfile(pathstr , '..', 'PLY', 'cube.ply');
fencePath = fullfile(pathstr , '..', 'PLY', 'fence.ply');

% Create objects
figure(1);
workbench = CreateObject(workBenchPath, workBenchPos);
wrench1 = CreateObject(wrench1Path, wrench1Pos);
wrench2 = CreateObject(wrench2Path, wrench2Pos);
wrench3 = CreateObject(wrench3Path, wrench3Pos);
gantry = CreateObject(gantryPath, gantryPos);
gantryMotor = CreateObject(gantryMotorPath, gantryMotorPos);
cube = CreateObject(cubePath,cubePos);

% Create fences
CreateWorkspace(fencePath, fencePos)

% Create light curtain
lines = CreateLightCurtain();

% Add wall/floor textures
CreateFloorWall();

% Create a list of all objects in the envrionment to check collision with
environment = [workbench, gantryMotor, gantry];

% Initialise robot
name = 'Robot';
robot = Fetch(fetchBase, workspace, name);
initialQMatrix = deg2rad([92 -80 0 -100 0 85 0]);
figure(1);
robot.model.plot(initialQMatrix, 'workspace', workspace, 'noarrow', 'scale', 0)

% Initialise GUI
gui = GUI();

num = webcamlist;
TH = isempty(num);

if(TH == 0)
    cam = webcam;
end

%%
% Mail loop
step = 1;
get_matrix = 1;
itteration = 1;
insideWorkspace = false;
view(-40, 30)
while 1
    %read GUI
    % Checks If Power Switch is turned on
    pause(0.00001)
    powerState = gui.GetPowerValue();
    while(powerState == "On")
        pause(0.00001)
        % Checks if Teach Mode Button is selected
        teachMode = gui.GetTeachModeValue();
        % ------TEACH MODE-------
        if(teachMode == 1)
            clc
            clf
            name = 'Robot';
            workspace = [-1 1 -1 1 -0.7 1];
            fetchBase = transl(0, 0, 0);
            robot = Fetch(fetchBase, workspace, name);
            q = deg2rad([92 -80 0 -100 0 85 0]);
            figure(1);
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
                figure(1);
                robot.model.plot(qMatrix);
                pause(0.00001)
                % Checks if the 2nd mode of Teach Mode is selected
                teachMode2 = gui.GetTeachMode2Value();
                if(teachMode2 == 1)
                    pause(0.00001)
                    joint1 = gui.GetJoint1Value();
                    joint2 = gui.GetJoint2Value();
                    joint3 = gui.GetJoint3Value();
                    joint4 = gui.GetJoint4Value();
                    joint5 = gui.GetJoint5Value();
                    joint6 = gui.GetJoint6Value();
                    joint7 = gui.GetJoint7Value();
                    qMatrix = [joint1 joint2 joint3 joint4 joint5 joint6 joint7];
                    figure(1);
                    robot.model.plot(qMatrix);
                end
            end
        end
        % If Start is pressed start process
        pause(0.00001)
        startValue = gui.GetStartValue();
        if(startValue == 1)
            %paused = 1;
            %check ESTOP
            %pause(0.00001)
            %EStopValue = gui.GetEStopValue();
            %if(EStopValue == 1)
                %while(paused == 1)
                    %pause(0.00001)
                    %EStopValue = gui.GetEStopValue();
                    %if(EStopValue == 0)
                    %paused = 0;
                    %end
                %end
            %end
            
            if(TH == 0)
                Image = snapshot(cam);
                targetIdentified = CameraScanner(Image);
            end
            
            % Move Gantry Crane
            pause(0.00001)
            obstructionValue = -1.4 + 2.8 * gui.GetBlockSlider()/100;
            if obstructionValue < -1.4
                obstructionValue = -1.4;
            elseif obstructionValue > 1.35
                obstructionValue = 1.35;
            end
            gantryMotor.mesh.Vertices(:, 1) = gantryMotor.verts(:, 1) + 1.4 + obstructionValue;
            
            % Move the cube for Light Curtain
            pause(0.00001)
            cubeValue = 1.5 * gui.GetLightBlockSlider()/100;
            cube.mesh.Vertices(:, 2) = cube.verts(:, 2) + cubeValue;
            
            pause(0.00001)
            resetButtonVal = gui.GetResetValue();
            if (resetButtonVal == 1)
                insideWorkspace = false;
            end
            
            if CheckLightCurtain(lines, cube)
                insideWorkspace = true;
            end
            % ========== FETCH CONTROLL ============
            % move to workbench
            pause(0.00001);
            % Checks if the EStop Button is pushed
            eStopValue = gui.GetEStopValue();
            if ~insideWorkspace && eStopValue == 0
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
                            figure(1);
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
                        qMatrix = robot.MoveJointState(deg2rad([92 -50 0 -115 0 15 0]));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end

                elseif step == 3
                    if get_matrix == 1
                        %qMatrix = robot.ArmRMRCJoints(deg2rad([92 30 0 -100 0 80 0]));
                        qMatrix = robot.MoveJointState(deg2rad([92 30 0 -100 0 80 0]));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end

                % move to above table
                elseif step == 4
                    if get_matrix == 1
                        qMatrix = robot.Move(transl(0, 0.5, 0.75)*trotx(pi));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end                

                % move closer
                elseif step == 5
                    if get_matrix == 1
                        destination = transl(workBenchPos(1, 4), workBenchPos(2, 4) - 0.75, fetchBase(3, 4))*trotz(pi/2);
                        basePos = robot.MoveBase(destination);
                        get_matrix = 0;
                    end 
                    if itteration <= size(basePos, 3)
                        collision = robot.CheckBaseCollision(gantryMotor);
                        if collision == 0   
                            robot.model.base = basePos(:, :, itteration);
                            itteration = itteration + 1;
                            currentPos = robot.model.getpos;
                            figure(1);
                            robot.model.plot(currentPos)
                        end
                    elseif itteration > size(basePos, 3)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end
                    
                % ======WRENCH 1======
                % pick up first wrench
                elseif step == 6
                    if get_matrix == 1
                        qMatrix = robot.ArmRMRCPos(wrench1Pos*trotx(pi)*trotz(pi));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end  
                    
                elseif step == 7
                    if get_matrix == 1
                        currentPos = robot.model.getpos;
                        qMatrix = robot.MoveJointState([currentPos(1:6) currentPos(7)-pi]);
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end  
                    
                elseif step == 8
                    if get_matrix == 1
                        currentPos = robot.model.getpos;
                        qMatrix = robot.MoveJointState([currentPos(1:6) currentPos(7)+pi]);
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                        robot.AttachObject(wrench1);
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            robot.UpdateObjectPos;
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end  

                elseif step == 9
                    if get_matrix == 1
                        qMatrix = robot.Move(transl(workBenchPos(1, 4)-0.3275,...
                            workBenchPos(2, 4) - 0.1, workBenchPos(3, 4) + 0.0625)*trotx(-pi/2));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            robot.UpdateObjectPos;
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end  

                % place wrench 1
                elseif step == 10
                    if get_matrix == 1
                        qMatrix = robot.ArmRMRCPos(transl(workBenchPos(1, 4)-0.3255,... %-0.3275
                            workBenchPos(2, 4) + 0.055, workBenchPos(3, 4) + 0.0625)*trotx(-pi/2));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            robot.UpdateObjectPos;
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end
 
                elseif step == 11
                    if get_matrix == 1
                        qMatrix = robot.ArmRMRCPos(transl(workBenchPos(1, 4)-0.3275,...
                            workBenchPos(2, 4) - 0.1, workBenchPos(3, 4) + 0.0625)*trotx(-pi/2));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        robot.DetachObject()
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end

                % ======WRENCH 2======
                % pick up second wrench
                elseif step == 12
                    if get_matrix == 1
                        qMatrix = robot.Move(wrench2Pos*trotx(pi)*trotz(pi));
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end  
                    
                elseif step == 13
                    if get_matrix == 1
                        currentPos = robot.model.getpos;
                        qMatrix = robot.MoveJointState([currentPos(1:6) currentPos(7)-pi]);
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end
                    
                elseif step == 14
                    if get_matrix == 1
                        currentPos = robot.model.getpos;
                        qMatrix = robot.MoveJointState([currentPos(1:6) currentPos(7)+pi]);
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        robot.AttachObject(wrench2);
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            robot.UpdateObjectPos;
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end
                    
                 elseif step == 15
                    if get_matrix == 1
                        qMatrix = robot.Move(transl(workBenchPos(1, 4),...
                            workBenchPos(2, 4) - 0.1, workBenchPos(3, 4) + 0.0625)*trotx(-pi/2));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            robot.UpdateObjectPos;
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end 
                    
                % place wrench 2
                elseif step == 16
                    if get_matrix == 1
                        qMatrix = robot.ArmRMRCPos(transl(workBenchPos(1, 4),...
                            workBenchPos(2, 4) + 0.03, workBenchPos(3, 4) + 0.0625)*trotx(-pi/2));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            robot.UpdateObjectPos;
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end      

                elseif step == 17
                    if get_matrix == 1
                        qMatrix = robot.ArmRMRCPos(transl(workBenchPos(1, 4),...
                            workBenchPos(2, 4) - 0.1, workBenchPos(3, 4) + 0.0625)*trotx(-pi/2));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        robot.DetachObject()
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end  

                % ======WRENCH 3======
                % pick up third wrench
                elseif step == 18
                    if get_matrix == 1
                        qMatrix = robot.Move(wrench3Pos*trotx(pi)*trotz(pi));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end  
                    
                elseif step == 19
                    if get_matrix == 1
                        currentPos = robot.model.getpos;
                        qMatrix = robot.MoveJointState([currentPos(1:6) currentPos(7)+pi]);
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end  
                    
                elseif step == 20
                    if get_matrix == 1
                        currentPos = robot.model.getpos;
                        qMatrix = robot.MoveJointState([currentPos(1:6) currentPos(7)+pi]);
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        robot.AttachObject(wrench3);
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            robot.UpdateObjectPos;
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end  
                    
                 elseif step == 21
                    if get_matrix == 1
                        qMatrix = robot.ArmRMRCPos(transl(workBenchPos(1, 4) + 0.305,...
                            workBenchPos(2, 4) - 0.1, workBenchPos(3, 4) + 0.0385)*trotx(-pi/2));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            robot.UpdateObjectPos;
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end 
                
                 % Hang Wrench 3
                 elseif step == 22
                    if get_matrix == 1
                        qMatrix = robot.ArmRMRCPos(transl(workBenchPos(1, 4) + 0.305,...
                            workBenchPos(2, 4) + 0.045, workBenchPos(3, 4) + 0.0365)*trotx(-pi/2));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            robot.UpdateObjectPos;
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end
                    
                elseif step == 23
                    if get_matrix == 1
                        qMatrix = robot.ArmRMRCPos(transl(workBenchPos(1, 4) + 0.305,...
                            workBenchPos(2, 4) - 0.1, workBenchPos(3, 4) + 0.0385)*trotx(-pi/2));
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix, environment);
                        if isCollision
                           disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' cannot complete this motion']) 
                        end
                        robot.DetachObject();
                        get_matrix = 0;
                    end
                    if itteration <= size(qMatrix, 1)
                        [isCollision, intersectP, l, j] = robot.IsArmCollision(qMatrix(itteration, :), gantryMotor);
                        if ~isCollision
                            figure(1);
                            robot.model.plot(qMatrix(itteration, :));
                            itteration = itteration + 1;
                        else
                            disp(['link: ', num2str(l), ' will collide with ', environment(j).name{1}, ' waiting for path to be clear'])
                        end
                    elseif itteration > size(qMatrix, 1)
                        itteration = 1;
                        get_matrix = 1;
                        step = step + 1;
                    end 
                    
                % done
                elseif step == 24
                    disp('DONE!')
                    return
                end
            end
        end
    end
end

function result = CheckLightCurtain(lines, object)
    result = false;
    for faceIndex = 1:size(object.f,1) 
        for i = 1:size(lines, 2)
            vertOnPlane = object.mesh.Vertices(object.f(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(object.fn(faceIndex,:), vertOnPlane, [lines(i).X(1) lines(i).Y(1) lines(i).Z(1)] , [lines(i).X(2) lines(i).Y(2) lines(i).Z(2)]); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,object.mesh.Vertices(object.f(faceIndex,:)',:))
                result = true;
                return
            end
        end
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
    obj.oriVerts = v - repmat(midPoint, obj.vertexCount, 1);
    obj.vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;
    obj.verts(:, 1) = obj.verts(:, 1) + pos(1, 4);
    obj.verts(:, 2) = obj.verts(:, 2) + pos(2, 4);
    obj.verts(:, 3) = obj.verts(:, 3) + pos(3, 4);
    hold on;
    obj.mesh = trisurf(f, obj.verts(:, 1), obj.verts(:, 2),...
        obj.verts(:, 3), 'FaceVertexCData', obj.vertexColours, 'EdgeLighting', 'flat', 'LineStyle', 'none');
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

function lines = CreateLightCurtain()
    lineGapZ = 0.5;
    LineStartX = -2;
    LineEndX = 2;
    LineStartY = -2.5;
    LineEndY = -2.5;
    LineStartZ = 0;
    LineEndZ = 2;
    %lines = nan((LineEndZ-LineStartZ)/lineGapZ, 1);
    for i = 1:(LineEndZ-LineStartZ)/lineGapZ
       zVal = lineGapZ * (i-1);
       line.X = [LineStartX, LineEndX];
       line.Y = [LineStartY, LineEndY];
       line.Z = [zVal, zVal];
       hold on
       plot3(line.X,line.Y,line.Z,'--rs','LineWidth',0.1);
       hold off
       lines(i) = line;
    end
end

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

function CreateFloorWall()
    I = imread('floor.jpg');
	xOffset = [-2 2];
	yOffset = [-3 2];
	zOffset = [-0.09 -0.09; -0.09 -0.09];
    figure(1)
    hold on;
	surf(xOffset, yOffset, zOffset,'CData',I,'FaceColor','texturemap');
    hold off;
    
    I = imread('brickWall.jpg');
    xOffset = [-2 2];
	yOffset = [1.6 1.6];
	zOffset = [-0.09 -0.09; 3.5 3.5];
    figure(1)
    hold on;
	surf(xOffset, yOffset, zOffset,'CData',I,'FaceColor','texturemap');
    hold off;
end

function CreateWorkspace(fencePath, fencePos)
    fence1 = CreateObject(fencePath,fencePos);
    fence1Pos = makehgtform('translate',[-1.95,-1.8,1.2]);
    rotatef1 = makehgtform('zrotate', -pi/2);
    updatePoints = [fence1Pos * rotatef1 * [fence1.verts,ones(fence1.vertexCount,1)]']';
    fence1.mesh.Vertices = updatePoints(:,1:3);

    fence2 = CreateObject(fencePath,fencePos);
    fence2Pos = makehgtform('translate',[-1.95,-0.3,1.2]);
    rotatef2 = makehgtform('zrotate', -pi/2);
    updatePoints = [fence2Pos * rotatef2 * [fence2.verts,ones(fence2.vertexCount,1)]']';
    fence2.mesh.Vertices = updatePoints(:,1:3);

    fence3 = CreateObject(fencePath,fencePos);
    fence3Pos = makehgtform('translate',[1.95,-1.8,1.2]);
    rotatef3 = makehgtform('zrotate', pi/2);
    updatePoints = [fence3Pos * rotatef3 * [fence3.verts,ones(fence3.vertexCount,1)]']';
    fence3.mesh.Vertices = updatePoints(:,1:3);

    fence4 = CreateObject(fencePath,fencePos);
    fence4Pos = makehgtform('translate',[1.95,-0.3,1.2]);
    rotatef4 = makehgtform('zrotate', pi/2);
    updatePoints = [fence4Pos * rotatef4 * [fence4.verts,ones(fence4.vertexCount,1)]']';
    fence4.mesh.Vertices = updatePoints(:,1:3);

    fence5 = CreateObject(fencePath,fencePos);
    fence5Pos = makehgtform('translate',[1.95,1.1,1.2]);
    rotatef5 = makehgtform('zrotate', pi/2);
    updatePoints = [fence5Pos * rotatef5 * [fence5.verts,ones(fence5.vertexCount,1)]']';
    fence5.mesh.Vertices = updatePoints(:,1:3);

    fence6 = CreateObject(fencePath,fencePos);
    fence6Pos = makehgtform('translate',[-1.95,1.1,1.2]);
    rotatef6 = makehgtform('zrotate', -pi/2);
    updatePoints = [fence6Pos * rotatef6 * [fence6.verts,ones(fence6.vertexCount,1)]']';
    fence6.mesh.Vertices = updatePoints(:,1:3);
end