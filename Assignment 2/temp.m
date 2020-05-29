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
    robot.model.teach