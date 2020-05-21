clear all;
clc;
close all;

fetchBase = transl(0,0,1);
workspace = [-0.2 1 -0.5 0.5 -0.1 1.5];
name = 'Robot';
robot = Fetch(fetchBase, workspace, name);
robot.model.teach
%robot.Move(transl(0.2, 0.2, 0.1))