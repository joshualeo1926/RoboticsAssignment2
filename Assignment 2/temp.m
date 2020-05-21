clear
clc

fetchBase = transl(0,0,0);
workspace = [-1.5 1.5 -1.5 1.5 -0.1 1.5];
name = 'Robot';
robot = Fetch(fetchBase, workspace, name);
robot.model.teach