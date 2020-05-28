clear all;
clc;
close all;

fetchBase = transl(0,0,0.5);
workspace = [-0.2 1 -0.5 0.5 -0.1 1.5];
name = 'Robot';
   
robot = Fetch(fetchBase, workspace, name);
robot.model.teach
% %qMatrix = robot.Move(transl(0, 0, 2));
% %robot.model.plot(qMatrix);
% robot.Move2(transl(0.45, 0.55, 0.5))
gui = GUI();
%%
%gui.EStopButtonPushed()
%gui.Lamp.Color = [0,1,0];
EStopValue = gui.getEStopValue()
%%
while(1)
    paused = 1;
    disp('Not paused');
    pause(0.00001)
    EStopValue = gui.getEStopValue();
    if(EStopValue == 1)
        while(paused == 1)
            disp('paused');
            pause(0.00001)
            EStopValue = gui.getEStopValue();
            if(EStopValue == 0)
                paused = 0;
            end
        end
    end
end