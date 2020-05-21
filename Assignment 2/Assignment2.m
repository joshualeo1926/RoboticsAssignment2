clear
clc
%     link1 = Link('d',1,'a',0,'alpha',pi/2,'qlim',[deg2rad(-92),deg2rad(92)]); %shoulderpan
%     link2 = Link('d',1,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-70),deg2rad(87)]); % shoulder lift
% 	link3 = Link('d',1,'a',0,'alpha',pi/2); % upperarm roll
% 	link4 = Link('d',-1,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-129),deg2rad(129)]); %elbow flex
% 	link5 = Link('d',1,'a',0,'alpha',pi/2); %forearm roll
% 	link6 = Link('d',1,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-125),deg2rad(125)]); %wrist flex
%     link7 = Link('d',1,'a',0,'alpha',pi/2); %wrist roll
%     
%     workspace = [-2 2 -2 2 -0.8 2];
%     base = transl(0,0,0);
% 	robot = SerialLink([link1 link2 link3 link4 link5 link6 link7],'name','robot', 'base', base);
%     qMatrix = eye(1,7);
    robot = Fetch();
    robot.model.teach
    %robot.plot(qMatrix,'noarrow','workspace',workspace);
   