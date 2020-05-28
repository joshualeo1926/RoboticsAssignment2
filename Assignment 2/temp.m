clear all;
clc;
close all;

fetchBase = transl(0,0,0.5);
workspace = [-0.2 1 -0.5 0.5 -0.1 1.5];
name = 'Robot';
   
% robot = Fetch(fetchBase, workspace, name);
% %robot.model.teach
% %qMatrix = robot.Move(transl(0, 0, 2));
% %robot.model.plot(qMatrix);
% robot.Move2(transl(0.45, 0.55, 0.5))
gui = GUI();

%%
    teachModeValue = 1;
    workspace = [-1 1 -1 1 -0.1 1.5];
    name = 'Robot';
    fetchBase = transl(0, 0, 0.52);
    robot = Fetch(fetchBase, workspace, name);
    q = deg2rad([92 -80 0 -100 0 85 0]);
    robot.model.plot(q, 'workspace', workspace, 'noarrow', 'scale', 0)
    EndEffector = robot.model.fkine(q);
    while(1)
        pause(0.00001)
        teachModeValue = gui.GetTeachModeValue();
        joint1 = gui.GetJoint1Value();
        joint2 = gui.GetJoint2Value();
        joint3 = gui.GetJoint3Value();
        joint4 = gui.GetJoint4Value();
        joint5 = gui.GetJoint5Value();
        joint6 = gui.GetJoint6Value();
        joint7 = gui.GetJoint7Value();
        qMatrix = [joint1 joint2 joint3 joint4 joint5 joint6 joint7];
        gui.EndEffector = robot.model.fkine(qMatrix)
        robot.model.plot(qMatrix);
        
    end
    
    function endEffector = GetEndEffector()
        
    end