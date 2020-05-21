classdef Fetch < handle
    properties
        model;
        base = transl(0,0,0);
        workspace = [-2 2 -2 2 -2 2];
        name = 'Robot';
        scale = 0;
    end
    
    methods
        function self = Fetch(base, workspace, name)
            if 0 < nargin
                self.base = base;
                self.workspace = workspace;
                self.name = name;
            end
            
            self.GetFetchRobot();
            self.PlotAndColourRobot();
        end
        
        function GetFetchRobot(self)
            pause(0.001);
        	link1 = Link('d',0.107511,'a',0.120238,'alpha',pi/2,'qlim',[deg2rad(-92),deg2rad(92)]); %shoulderpan
            link2 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-87),deg2rad(70)], 'offset', deg2rad(90)); % shoulder lift
            link3 = Link('d',0.35,'a',0,'alpha',-pi/2); % upperarm roll
            
            link4 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-129),deg2rad(129)]); %elbow flex
            link5 = Link('d',0.315,'a',0,'alpha',pi/2); %forearm roll
            link6 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-125),deg2rad(125)]); %wrist flex
            link7 = Link('d',0.15,'a',0,'alpha',0); %wrist roll
            qMatrix = [0 0 0 0 0 0 0];
            
            self.model = SerialLink([link1 link2 link3 link4 link5 link6 link7], 'name', self.name, 'base', self.base);
            self.model.plot(qMatrix, 'workspace', self.workspace, 'scale', self.scale, 'noarrow');
            %self.model.plot(qMatrix, 'workspace', self.workspace,'noarrow');
        end
        
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                currentFile = mfilename( 'fullpath' );
                [pathstr,~,~] = fileparts( currentFile );
                plyPath = fullfile(pathstr , '..', 'PLY', ['L',  num2str(linkIndex), '.ply']);
                [f, v, d] = plyread(plyPath,'tri');
                self.model.faces{linkIndex + 1} = f;
                self.model.points{linkIndex + 1} = v;
                Data{linkIndex + 1} = d;
            end 
            
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [Data{linkIndex+1}.vertex.red ...
                                                                  , Data{linkIndex+1}.vertex.green ...
                                                                  , Data{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        function Move(self, pos)
            steps = 50;
            initialPos = self.model.getpos;
            finalPos = self.model.ikcon(pos, initialPos);
            s = lspb(0,1,steps);
            qMatrix = zeros(steps, 7);
            for i=1:steps
                qMatrix(i, :) = (1-s(i))*initialPos(1, 4) + s(i)*finalPos;
            end
            self.model.plot(qMatrix, 'noarrow')
        end
    end
end