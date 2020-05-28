clear all;
clc;
close all;

gui = GUI();

%%
    teachModeValue = 1;
    workspace = [-1 1 -1 1 -0.1 1.5];
    name = 'Robot';
    fetchBase = transl(0, 0, 0.52);
    robot = Fetch(fetchBase, workspace, name);
    q = deg2rad([92 -80 0 -100 0 85 0]);
    robot.model.plot(q, 'workspace', workspace, 'noarrow', 'scale', 0)
    endEffector = robot.model.fkine(q);
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
%         endEffector = robot.model.fkine(qMatrix);
        robot.model.plot(qMatrix);
    end
   