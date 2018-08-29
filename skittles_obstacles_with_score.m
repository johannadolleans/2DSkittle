%% Virtual Skittle w/Obstacles and score code by Zhaoran Zhang 
% Modified by Johanna DOlleans
%code to force exploration by creating obstacle and creating a score in 
%case of repetitve behavoir


function skittles_obstacles_with_score(name,session,hand,i)
%% Parameter
trialsPerBlock = 25; % number of trials
thresholdO=.05; %threshold for Obstacles
thresholda=15; %threshold for angle
thresholdv=30; %threshold for velocity
threshold=.05; %threshold for target


%% Information regarding the Session
   
    block=i;
    subject = name;
    
    Score =0;   
    %% Information regarding the target and the figure   
         if hand == 'R'
            xtarget = .38;
               else
            xtarget = -.38 ;
               end
            ytarget=.43;
   target=[xtarget, ytarget];
    graphicsScale = .25; 
    if block > 1
            Calling=strcat(subject,'_',num2str(session),'_',num2str(block-1),'_','Obstacles');
        load(Calling);
%         Obstacles1=Obstaclex;
    end
    if session > 1
            Calling=strcat(subject,'_',num2str(session-1),'_',num2str(block+1),'_','Obstacles');
        load(Calling);
%         Obstacles1=Obstaclex;
    end
    f=size(Obstacles1,1);
    close all;
    if f > 1
            fig = createUI2(target,graphicsScale,Obstacles1);
            h=1
    else
    fig = createUI(target,graphicsScale);
    h=2
    end
    handles = guidata(fig);
    
    %% Data Acquisition
    s = daq.createSession('ni');
    ai=s.addAnalogInputChannel('Dev2','ai0','Voltage');
    ai.TerminalConfig='SingleEnded';
    di=s.addDigitalChannel('Dev2','port0/line11:0','InputOnly');
    s.Rate = 1000;
    s.IsContinuous = true;
    s.IsNotifyWhenDataAvailableExceedsAuto=false;
    s.NotifyWhenDataAvailableExceeds=50;
    setappdata(fig, 'encoder', zeros(1,12));

    fid = fopen(sprintf('%s_%d_%d.bin', subject, session, block),'w');
    lh = s.addlistener('DataAvailable', @(src, event)getData(src, event, fid));
    s.startBackground();
    
    trial = 1;
    R=[];
    T=[];
    setappdata(fig, 'simStep', 'waitToStart');
    releaseData=zeros(trialsPerBlock,4);
    Obsx=0;
    Obsy=0;
    Obstaclex=[];
    k=1; 
    while s.IsRunning
        angle = gc2dec(getappdata(fig, 'encoder'))./4095*360;
        set(handles.arm, 'XData', [0 .4*-cosd(angle)]*graphicsScale, 'YData', [-1.5 -1.5+.4*sind(angle)]*graphicsScale);
        switch(getappdata(fig, 'simStep'))
            case 'waitToStart' 
            case 'waitToThrow' 
                set(handles.trajectory,'Visible', 'off');
                set(handles.target, 'FaceColor', 'y');
                set(handles.ball, 'Position', [(.4*-cosd(angle))-.05,(-1.5+.4*sind(angle))-.05,.05*2,.05*2]*graphicsScale);
            case 'ballReleased'
                minDistance = 100;
                postHit = false;
                targetHit = false;
                spring = getappdata(fig, 'springParameters');
                tStart = getappdata(fig, 'drawTimeStart');
                time = toc(tStart);
                prevTime = time;
                i=1;
                while(time < 1.9 && postHit == false)
                    time = toc(tStart);
                    for t=prevTime:.001:time
                        checkx=spring(1).*sin(spring(5).*t+spring(3)).*exp(-t/spring(6));
                        checky=spring(2).*sin(spring(5).*t+spring(4)).*exp(-t/spring(6));
                        if (sqrt((checkx-target(1))^2 + (checky-target(2))^2) < minDistance)
                            minDistance = sqrt((checkx-target(1))^2 + (checky-target(2))^2);
                          if minDistance < threshold
                                targetHit = true;
                               set(handles.target, 'FaceColor', 'g'); 
                               postHit= true;
                               R=[R;getappdata(fig, 'release')];
                          end
                        end
                        if(checkx^2+checky^2 < .25^2)
                            postHit = true;
                            minDistance = 1;
                        end
                        for o=1:f
                        if(sqrt((checkx-Obstacles1(o,2))^2+(checky-Obstacles1(o,3))^2) < thresholdO)
                            postHit = true;
                            minDistance = 1;
                        end
                        end
                    end
                    x(i)=spring(1).*sin(spring(5).*time+spring(3)).*exp(-time/spring(6));
                    y(i)=spring(2).*sin(spring(5).*time+spring(4)).*exp(-time/spring(6));
                    set(handles.trajectory, 'XData', x*graphicsScale, 'YData', y*graphicsScale, 'Visible', 'on');
                    set(handles.ball, 'Position', [x(i)-.05,y(i)-.05,.05*2,.05*2]*graphicsScale);
                    angle = gc2dec(getappdata(fig, 'encoder'))./4095*360;
                    set(handles.arm, 'XData', [0 .4*-cosd(angle)]*graphicsScale, 'YData', [-1.5 -1.5+.4*sind(angle)]*graphicsScale);
                    drawnow;
                    prevTime = time;
                    i=i+1;
                  end 
                if targetHit == true
                    T=[T;trial];
                end
                
                releaseData(trial,:) = [getappdata(fig, 'release') minDistance];                
 if releaseData(trial,4)<.05
       if Score > 9
          delete(ms1) 
       end
       Score=Score+10;
       ms1 = msgbox(sprintf( 'Great Job! \n Your  score is %d !', Score), 'test');     %create msgbox
th = findall(ms1, 'Type', 'Text');                   %get handle to text within msgbox
th.FontSize = 20; 
     set(ms1, 'position', [1060 350 350 75]); %makes box bigger
%        set(ms1, 'position', [1100 75 50 50]); %makes box bigger
 else
       ms1 = msgbox(sprintf( 'Nice try, keep up the good work!!'), 'test');     %create msgbox
th = findall(ms1, 'Type', 'Text');                   %get handle to text within msgbox
th.FontSize = 20; 
     set(ms1, 'position', [1060 350 350 75]); %makes box bigger
 end
   if trial<5
       a=0;v=0;
        Calling=strcat(subject,'_',num2str(session),'_',num2str(block-1),'_','releases');
        A=load(Calling);
        C=[];
        for i=1:5
        if A(i,4)<.05
            C=[C;A(i,2:3)];
        end
        end
  if size(C,1)>2
      for i=2:size(C,1)
          if abs(C(i,2)-C(i-1,2))<thresholda
           a=a+1;
          end
          if abs(C(i,3)-C(i-1,3))<thresholdv
           v=v+1;
          end
      end
      if v > 2
             meanV=C(2,3);
         end
          if a > 2
             meanA=C(2,2);
       end
  end
   end
                if (trial>4)
     meanA=0; meanV=0;
     if releaseData(trial,4)<.05
         a=0; v=0;
         indice=find(T==trial);
         if indice>4
         for i=indice-4:indice-1
             if abs(releaseData(T(i),2)-releaseData(trial,2))<thresholda
                 a=a+1;
             end
             if abs(releaseData(T(i),3)-releaseData(trial,3))<thresholdv
                 v=v+1;
             end
         end
         if v > 2
             meanV=releaseData(trial,3);
         end
          if a > 2
             meanA=releaseData(trial,2);
          end
         end
     
if abs(meanA)>0
 if abs(meanV)>0
spring=getSpringParameters(meanA, meanV);
time=linspace(0.01,2,200);
x2=spring(1).*sin(spring(5).*time+spring(3)).*exp(time./spring(6));
y2=spring(2).*sin(spring(5).*time+spring(4)).*exp(time./spring(6));
indice=find(abs(y2)<0.05);
Obsx=x2(indice(1));
f=f+1;
           Obstaclex(k,1)=trial;
           Obstaclex(k,2)=Obsx;
           Obstaclex(k,3)=Obsy;
           if a > v
               Obstaclex(k,4)=a;
           else
               Obstaclex(k,4)=v;
           end
           k=k+1;
           clear meanA meanV Obsx x2 y2 T
           T=[];
           switch k-1
               case 1
           set(handles.obstacle,'Visible', 'on');
           set(handles.obstacle, 'Position', [Obstaclex(1,2)-.05,Obstaclex(1,3)-.05,.05*2,.05*2]*graphicsScale); 
               case 2
           set(handles.obstacle,'Visible', 'on');
           set(handles.obstacle, 'Position', [Obstaclex(1,2)-.05,Obstaclex(1,3)-.05,.05*2,.05*2]*graphicsScale);
           set(handles.obstacle2,'Visible', 'on');
           set(handles.obstacle2, 'Position', [Obstaclex(2,2)-.05,Obstaclex(2,3)-.05,.05*2,.05*2]*graphicsScale); 
               case 3
           set(handles.obstacle,'Visible', 'on');
           set(handles.obstacle, 'Position', [Obstaclex(1,2)-.05,Obstaclex(1,3)-.05,.05*2,.05*2]*graphicsScale);
           set(handles.obstacle2,'Visible', 'on');
           set(handles.obstacle2, 'Position', [Obstaclex(2,2)-.05,Obstaclex(2,3)-.05,.05*2,.05*2]*graphicsScale);
           set(handles.obstacle3,'Visible', 'on');
           set(handles.obstacle3, 'Position', [Obstaclex(3,2)-.05,Obstaclex(3,3)-.05,.05*2,.05*2]*graphicsScale); 
           end
 end
           Obstacles1=[Obstacles1;Obstaclex];
end
     end
      %delete(ms);
    end
            
                if trial == trialsPerBlock
                    s.stop();
                else
                    trial = trial+1;
                    setappdata(fig, 'simStep', 'waitToStart');
                    clear x y;
                end
        end
        drawnow;
        
    end
   %% end of acquisition and saving of data
    delete(lh);
    fclose(fid);
    save(sprintf('%s_%d_%d_releases.mat', subject, session, block), 'releaseData');
    save(sprintf('%s_%d_%d_Obstacles.mat', subject, session, block), 'Obstacles1');
    close all;
%     trialName=strcat(subject,'_',num2str(session),'_',num2str(block));
%     transformTrajectoryData(trialName);
end
%% Function to create the figure of the virtual Skitlle
function fig = createUI(target,graphicsScale)
    fig = figure('Tag','fig',...
                 'Units','normalized',...
                 'OuterPosition',[1, 0, 1, 1],...
                 'Color',[0 0 0],...
                 'Renderer', 'zbuffer', ...
                 'MenuBar', 'none', ...
                 'ToolBar', 'none', ...
                 'Resize', 'off');

    ax = axes('Parent',fig,...
              'Tag','ax', ...
              'Units','normalized',...
              'Position',[0 0 1 1],...
              'XLim',[-.6 .6],...
              'YLim',[-.51 .51],...
              'XLimMode', 'manual', ...
              'YLimMode', 'manual', ...
              'Color',[0 0 0]);
          
    daspect('manual');

    post = rectangle('Parent',ax,...
                       'Tag','post',....
                       'Position',[0-.25,0-.25,.25*2,.25*2]*graphicsScale,...
                       'LineStyle', 'none', ...
                       'Curvature',[1,1], ...
                       'FaceColor',[1 0 0]);
                   
    target = rectangle('Parent',ax,...
       'Tag','target',....
       'Position',[target(1)-.05,target(2)-.05,.05*2,.05*2]*graphicsScale,...
       'LineStyle', 'none', ...,
       'Curvature',[1,1], ...
       'FaceColor',[1 1 0]);   

    armJoint = rectangle('Parent',ax,...
                       'Tag','armJoint',....
                       'Position',[0-.02,-1.5-.02,.01*2,.02*2]*graphicsScale,...
                       'LineStyle', 'none', ...
                       'Curvature',[1,1], ...
                       'FaceColor',[1 1 1]);

    arm = line('Parent',ax, ...
               'Tag','arm', ...
               'LineStyle', '-',...
               'XData', [0.4 0]*graphicsScale,...
               'YData', [-1.5 -1.5]*graphicsScale,...
               'LineStyle', '-', ...
               'LineWidth', 3, ...
               'Color',[1 0 1]); 
           
   trajectory = line('Parent',ax, ...
               'Tag','trajectory', .....
               'XData', [-10 -10]*graphicsScale,...
               'YData', [-10 -10]*graphicsScale,...
               'LineStyle', '-', ...
               'LineWidth', 2, ...
               'Color','w', ...
               'Visible', 'off'); 

    ball = rectangle('Parent',ax,...
       'Tag','ball',....
       'LineStyle', 'none', ...
       'Position', [-10,-10,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[1 1 1], ...
       'Visible', 'off');   
   
    obstacle = rectangle('Parent',ax,...
       'Tag','obstacle',....
       'LineStyle', 'none', ...
       'Position', [-5,0,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[0 0 1], ...
       'Visible', 'off');
   
   obstacle2 = rectangle('Parent',ax,...
       'Tag','obstacle2',....
       'LineStyle', 'none', ...
       'Position', [-5,0,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[0 0 1], ...
       'Visible', 'off');
   obstacle3 = rectangle('Parent',ax,...
       'Tag','obstacle3',....
       'LineStyle', 'none', ...
       'Position', [-5,0,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[0 0 1], ...
       'Visible', 'off');
    % create handles structure
    ad = guihandles(fig);
    
    % save application data
    guidata(fig, ad);
end
function fig = createUI2(target,graphicsScale,Obstacle1)
    fig = figure('Tag','fig',...
                 'Units','normalized',...
                 'OuterPosition',[0, 0, 1, 1],...
                 'Color',[0 0 0],...
                 'Renderer', 'zbuffer', ...
                 'MenuBar', 'none', ...
                 'ToolBar', 'none', ...
                 'Resize', 'off');

    ax = axes('Parent',fig,...
              'Tag','ax', ...
              'Units','normalized',...
              'Position',[0 0 1 1],...
              'XLim',[-.6 .6],...
              'YLim',[-.51 .51],...
              'XLimMode', 'manual', ...
              'YLimMode', 'manual', ...
              'Color',[0 0 0]);
          
    daspect('manual');

    post = rectangle('Parent',ax,...
                       'Tag','post',....
                       'Position',[0-.25,0-.25,.25*2,.25*2]*graphicsScale,...
                       'LineStyle', 'none', ...
                       'Curvature',[1,1], ...
                       'FaceColor',[1 0 0]);
                   
    target = rectangle('Parent',ax,...
       'Tag','target',....
       'Position',[target(1)-.05,target(2)-.05,.05*2,.05*2]*graphicsScale,...
       'LineStyle', 'none', ...,
       'Curvature',[1,1], ...
       'FaceColor',[1 1 0]);   

    armJoint = rectangle('Parent',ax,...
                       'Tag','armJoint',....
                       'Position',[0-.02,-1.5-.02,.01*2,.02*2]*graphicsScale,...
                       'LineStyle', 'none', ...
                       'Curvature',[1,1], ...
                       'FaceColor',[1 1 1]);

    arm = line('Parent',ax, ...
               'Tag','arm', ...
               'LineStyle', '-',...
               'XData', [0.4 0]*graphicsScale,...
               'YData', [-1.5 -1.5]*graphicsScale,...
               'LineStyle', '-', ...
               'LineWidth', 3, ...
               'Color',[1 0 1]); 
           
   trajectory = line('Parent',ax, ...
               'Tag','trajectory', .....
               'XData', [-10 -10]*graphicsScale,...
               'YData', [-10 -10]*graphicsScale,...
               'LineStyle', '-', ...
               'LineWidth', 2, ...
               'Color','w', ...
               'Visible', 'off'); 

    ball = rectangle('Parent',ax,...
       'Tag','ball',....
       'LineStyle', 'none', ...
       'Position', [-10,-10,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[1 1 1], ...
       'Visible', 'off');   
   
    obstacle = rectangle('Parent',ax,...
       'Tag','obstacle',....
       'LineStyle', 'none', ...
       'Position', [-5,0,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[0 0 1], ...
       'Visible', 'off');
   
   obstacle2 = rectangle('Parent',ax,...
       'Tag','obstacle2',....
       'LineStyle', 'none', ...
       'Position', [-5,0,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[0 0 1], ...
       'Visible', 'off');
   obstacle3 = rectangle('Parent',ax,...
       'Tag','obstacle3',....
       'LineStyle', 'none', ...
       'Position', [-5,0,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[0 0 1], ...
       'Visible', 'off');
    if size(Obstacle1,1)>2
          obstacle13 = rectangle('Parent',ax,...
       'Tag','obstacle3',....
       'LineStyle', 'none', ...
       'Position', [Obstacle1(3,2)-.05,Obstacle1(3,3)-.05,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[0 0 1], ...
       'Visible', 'on');
    end
    if size(Obstacle1,1)>1
            obstacle12 = rectangle('Parent',ax,...
       'Tag','obstacle2',....
       'LineStyle', 'none', ...
       'Position', [Obstacle1(2,2)-.05,Obstacle1(2,3)-.05,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[0 0 1], ...
       'Visible', 'on');
        end
       obstacle11 = rectangle('Parent',ax,...
       'Tag','obstacle',....
       'LineStyle', 'none', ...
       'Position', [Obstacle1(1,2)-.05,Obstacle1(1,3)-.05,.05*2,.05*2]*graphicsScale, ...
       'Curvature',[1,1], ...
       'FaceColor',[0 0 1], ...
       'Visible', 'on');

    % create handles structure
    ad = guihandles(fig);
    
    % save application data
    guidata(fig, ad);
end

%% functions to get the data and calculate the velocity and trajectory
function getData(src, event, fid) 
    fig = findall(0, 'Tag', 'fig');
    setappdata(fig, 'encoder', event.Data(end, 2:13)) 
    %setappdata(fig2,'encoder2',event2.Data(end,2:13));
    data = [event.TimeStamps, event.Data]';
    %data2=[event2.TimeStamps,event2.Data];
    fwrite(fid,data,'double');
    switch(getappdata(fig, 'simStep'))
        case 'waitToStart'
            if any(event.Data(:,1) > 2.0)%&&(event2.Data(:,1) > 2.0)
                setappdata(fig,'simStep', 'waitToThrow');
                handles = guidata(fig);
                set(handles.ball, 'Visible', 'on');
            end
        case 'waitToThrow'
            if any(event.Data(:,1) < 1)%&&(event2.Data(:,1) < 0.5)
               setappdata(fig,'simStep', 'ballReleased');
               releaseIndex = find(event.Data(:,1) < 1.0, 1);
               releaseAngle =  gc2dec(event.Data(releaseIndex, 2:13))./4095*360;
               
               if releaseIndex < 20
                   pastAngles = getappdata(fig, 'pastAngles');
                   releaseVelocity = getVelocity([pastAngles(end-(19-releaseIndex):end, :); event.Data(1:releaseIndex, 2:13)]);
               else
                   releaseVelocity = getVelocity(event.Data(releaseIndex-19:releaseIndex, 2:13));
               end
               setappdata(fig, 'release', [double(src.ScansAcquired-50+releaseIndex), releaseAngle, releaseVelocity]);
               setappdata(fig, 'drawTimeStart', tic);
               spring = getSpringParameters(releaseAngle, releaseVelocity);
               setappdata(fig, 'springParameters', spring);
            end
            setappdata(fig,'pastAngles', event.Data(end-18:end, 2:13));
        case 'ballReleased'
    end
end

function slope = getVelocity(data)
    angle = zeros(1,20);
    for i = 1:20
        angle(i) = gc2dec(data(i,:))./4095*360;
    end
    boundary = find(abs(diff(angle))>300);
    if ~isempty(boundary)
        if angle(boundary+1)-angle(boundary) > 0
            angle(boundary+1:end) = angle(boundary+1:end)-360;
        else
        	angle(1:boundary) = angle(1:boundary)-360;
        end
    end
    time = 0:.001:.019;
    slope = sum((time-mean(time)).*(angle-mean(angle)))/sum((time-mean(time)).*(time-mean(time))); 
       
end

function spring = getSpringParameters(releaseAngle, releaseVelocity)
    m = 0.1; %mass
    k = 1; %spring constant
    c = 0.01;   %viscous damping
    T = (2*m/c);    %relaxation time
    %L=0.95;
    %g=9.81;
    %T=2*m*g/L;
    %w=sqrt(g)/L;
    w = sqrt(abs((k/m)-((1/T)^2))); %frequency
    l = 0.4; %arm length
    
    x0=-l*cosd(releaseAngle);
    y0=-1.5+l*sind(releaseAngle);
    vx0=(releaseVelocity*pi/180*l)*sind(releaseAngle); 
    vy0=(releaseVelocity*pi/180*l)*cosd(releaseAngle);
    
    ax=sqrt((x0)^2+((vx0/w)+(x0/T)/w)^2);
    ay=sqrt((y0)^2+((vy0/w)+(y0/T)/w)^2);
    
    if ax~=0
        px=acos((1/ax)*((vx0/w)+((x0/T)/w)));
    else
        px=0;
    end
    if x0<0
        px=-px;
    end

    if ay~=0
        py=acos((1/ay)*((vy0/w)+((y0/T)/w)));
    else py=0;
    end
    if y0<0
        py=-py;
    end 
    spring = [ax,ay,px,py,w,T];
end
function dec = gc2dec(gra)
    s2 = size(gra,2);
    bin = char(zeros(1,s2));

    for j2 = s2:-1:2
        if mod(sum(gra(1:j2-1)),2) == 1
            bin(j2) = int2str(1 - gra(j2));
        else
            bin(j2) = int2str(gra(j2));
        end
    end

    bin(1) = int2str(gra(1));
    dec = bin2dec(bin);
end