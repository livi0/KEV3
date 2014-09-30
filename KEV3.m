function varargout = KEV3(varargin)
% KEV3_gui MATLAB code for KEV3_gui.fig
%      KEV3_gui, by itself, creates a new KEV3_gui or raises the existing
%      singleton*.
%
%      H = KEV3_gui returns the handle to a new KEV3_gui or the handle to
%      the existing singleton*.
%
%      KEV3_gui('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KEV3_gui.M with the given input arguments.
%
%      KEV3_gui('Property','Value',...) creates a new KEV3_gui or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before KEV3_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to KEV3_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help KEV3_gui

% Last Modified by GUIDE v2.5 25-Sep-2014 15:49:33

% KEV3: Kinect integration with the EV3 robot (available at Matlab Central)
% Made by: Liviu Ivanescu, 30-Sep-2014, gruiva@gmail.com

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @KEV3_OpeningFcn, ...
                   'gui_OutputFcn',  @KEV3_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Outputs from this function are returned to the command line.
function varargout = KEV3_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
handles.output = hObject;
varargout{1} = handles.output;




% --- Executes during object creation, after setting all properties.
function KEV3_gui_CreateFcn(hObject, eventdata, handles)
% hObject    handle to KEV3_gui (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function cameraAxes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cameraAxes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate cameraAxes
handles.ax = gca;
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function logo_lab_CreateFcn(hObject, eventdata, handles)
% hObject    handle to logo_lab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate logo_lab

handles.axlog = gca;
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function wheelAxes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wheelAxes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate wheelAxes

handles.axw = gca;
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function tilt_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tilt_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
string1 = sprintf('%.1f%c', 0, char(176));
set(hObject, 'String', string1);


% --- Executes during object creation, after setting all properties.
function slider_tilt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
set(hObject,'Value',0.5);

% --- Executes during object creation, after setting all properties.
function reset_tilt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to reset_tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function start_stop_cam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to start_stop_cam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function slider_drive_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_drive (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
set(hObject,'Min',-1,'Max',1,'Value',0);


% --- Executes during object creation, after setting all properties.
function slider_turn_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_turn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.

set(hObject,'Min',-1,'Max',1,'Value',0);


% --- Executes during object creation, after setting all properties.
function pushbutton_enableEV3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_enableEV3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

set(hObject,'String','Stop EV3')
set(hObject,'Enable','off')








% --- Executes just before KEV3_gui is made visible.
function KEV3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to KEV3_gui (see VARARGIN)

% dbstop if error

global t2;
t2 = clock();

imaqreset %deletes any image acquisition objects that exsist in memory and uploads
handles.video = videoinput('kinect', 1);
handles.depth = videoinput('kinect', 2);

set(handles.video,'Timeout',5); %set the Timeout property of VIDEOINPUT object 'vid' to 10 seconds
set(handles.depth,'Timeout',5); %set the Timeout property of VIDEOINPUT object 'vid' to 10 seconds

triggerconfig(handles.video,'manual');
triggerconfig(handles.depth,'manual');

handles.video.FramesPerTrigger = 1; % Capture 1 frame
handles.depth.FramesPerTrigger = 1; % Capture 1 frame
handles.video.TriggerRepeat = Inf;
handles.depth.TriggerRepeat = Inf;

handles.srcDepth = getselectedsource(handles.depth);
set(handles.srcDepth, 'BodyPosture', 'Seated','TrackingMode','Skeleton');

% plot lab logo
logo = imread('logograyong.png');
imshow(logo,'parent',handles.axlog)

% initiate wheel plots
acrc = [0:0.01:2*pi];
xcrc = cos(acrc);
ycrc = sin(acrc);
hold(handles.axw,'on');
plot(handles.axw,xcrc,ycrc,':k')
plot(handles.axw,[0 0],[0 1],'k','LineWidth',3)
plot(handles.axw,0,1,'ko','MarkerSize',8,'MarkerFaceColor','r')
hold(handles.axw,'off');
set(handles.axw,'XTick',[],'YTick',[],'XTickLabel',{},'YTickLabel',{},'Box','off','Visible','off','Color','none')
set(handles.axw,'XLim',[-1 1],'YLim',[-1 1])

% initiate video box
% axpos = get(handles.ax,'Position');
% axpos(3) = 640; axpos(4) = 480;
% set(handles.ax,'Position',axpos,'Units','pixels');

%create a timer for calling the display function every 0.05 secs
handles.tmr = timer('TimerFcn', {@startAq_Callback,handles},'Period', 0.05,'executionMode','fixedRate');

% Choose default command line output for KEV3_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);









% --- Executes on button press in start_stop_cam.
function start_stop_cam_Callback(hObject, eventdata, handles)
% hObject    handle to start_stop_cam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Start/Stop Camera
if strcmp(get(handles.start_stop_cam,'String'),'Start Camera')
      runstat = get(handles.video,'Running');
      if strcmp(runstat,'off')
          start([handles.video handles.depth])
      end 
      waitfor(handles.video,'Running','on')
      start(handles.tmr)% start timer
      % Camera is off. Change button string and start camera.
      set(handles.start_stop_cam,'String','Stop Camera')
      set(handles.start_stop_cam,'Enable','on');
else
      set(handles.start_stop_cam,'Enable','off');
      runstat = get(handles.video,'Running');
      if strcmp(runstat,'on')
          stop(handles.tmr)% stop timer
          stop([handles.video handles.depth])
          waitfor(handles.video,'Running','off')
          start([handles.video handles.depth])
          set(handles.start_stop_cam,'Enable','on');
          % Camera is on. Stop camera and change button string.
          set(handles.start_stop_cam,'String','Start Camera')
      end
end
% Update handles structure
guidata(hObject, handles);


% --- Executes on slider movement.
function slider_tilt_Callback(hObject, eventdata, handles)
% hObject    handle to slider_tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.tilt = (get(hObject,'Value')-0.5)*27/0.5;
string1 = sprintf(' %.1f%c', handles.tilt, char(176));
set(handles.tilt_value, 'String', string1);
set(handles.srcDepth, 'CameraElevationAngle', handles.tilt)
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in reset_tilt.
function reset_tilt_Callback(hObject, eventdata, handles)
% hObject    handle to reset_tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.slider_tilt,'Value',0.5);
string1 = sprintf(' %.1f%c', 0, char(176));
set(handles.tilt_value, 'String', string1);
set(handles.srcDepth, 'CameraElevationAngle', 0)
% Update handles structure
guidata(hObject, handles);


function tilt_value_Callback(hObject, eventdata, handles)
% hObject    handle to tilt_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tilt_value as text
%        str2double(get(hObject,'String')) returns contents of tilt_value as a double
set(handles.srcDepth, 'CameraElevationAngle', 0)
% Update handles structure
guidata(hObject, handles);



function startAq_Callback(tmr_obj, tmr_data, handles)
% function startAq_Callback(hObject, eventdata, handles)

global t2;

% Trigger both objects.
if strcmp(get(handles.video,'Logging'),'off')
    trigger(handles.video);
end
if strcmp(get(handles.depth,'Logging'),'off')
    trigger(handles.depth);
end

% logstat = get(handles.video,'Logging')
waitfor(handles.video,'Logging','on')
waitfor(handles.depth,'Logging','on')

% Get the acquired frames and metadata.
frmv = get(handles.video,'FramesAvailable');
frmd = get(handles.depth,'FramesAvailable');
% flg = 1;
if frmv>=1
    [imgColor, ts_color, metaData_Color] = getdata(handles.video);
    % show movie
    imshow(imgColor,'parent',handles.ax);
%     hold(handles.ax,'on');
end
if frmd>=1
    
    [imgDepth, ts_depth, metaData_Depth] = getdata(handles.depth);
    
    % See which skeletons were tracked.
    trackedSkeletons = find(metaData_Depth(1).IsSkeletonTracked);
    jointCoordinates = metaData_Depth(1).JointWorldCoordinates(:, :, trackedSkeletons);
    % Skeleton's joint indices with respect to the color image
    jointIndices = metaData_Depth(1).JointImageIndices(:, :, trackedSkeletons);
    % Find number of Skeletons tracked
    nSkeleton = length(trackedSkeletons);
    
    % Skeleton connection map to link the joints
    SkeletonConnectionMap = [[1 2]; % Spine
        [2 3];
        [3 4];
        [3 5]; %Left Hand
        [5 6];
        [6 7];
        [7 8];
        [3 9]; %Right Hand
        [9 10];
        [10 11];
        [11 12];
        [1 17]; % Right Leg
        [17 18];
        [18 19];
        [19 20];
        [1 13]; % Left Leg
        [13 14];
        [14 15];
        [15 16]];
    
    % Draw the skeletons on the RGB image
    jointIndices(jointIndices==0) = nan;
    
    if nSkeleton > 0

        hold(handles.ax,'on');
        for i = 1:19
            X1 = [jointIndices(SkeletonConnectionMap(i,1),1,1) jointIndices(SkeletonConnectionMap(i,2),1,1)];
            Y1 = [jointIndices(SkeletonConnectionMap(i,1),2,1) jointIndices(SkeletonConnectionMap(i,2),2,1)];
            plot(handles.ax,X1,Y1, 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '+', 'Color', 'r');
        end
        hold(handles.ax,'off');
        
        % get skeleton positions (l for left, r for rigth)
        x_lhand = jointIndices(8,1);
        y_lhand = jointIndices(8,2);
        x_rhand = jointIndices(12,1);
        y_rhand = jointIndices(12,2);
        
        x_lelbow = jointIndices(6,1);
        y_lelbow = jointIndices(6,2);
        x_relbow = jointIndices(10,1);
        y_relbow = jointIndices(10,2);
        
        x_lshoulder = jointIndices(5,1);
        y_lshoulder = jointIndices(5,2);
        x_rshoulder = jointIndices(9,1);
        y_rshoulder = jointIndices(9,2);
        
        x_neck = jointIndices(3,1);
        y_neck = jointIndices(3,2);
        
        % build driving data
        
        dxl = x_lhand - x_lelbow;
        dxr = x_rhand - x_relbow;
        dyl = y_lhand - y_lelbow;
        dyr = y_rhand - y_relbow;
        
        dxls = x_neck - x_lshoulder;
        dxrs = x_neck - x_rshoulder;
        dyls = y_neck - y_lshoulder;
        dyrs = y_neck - y_rshoulder;
        
        % angles left and rigth
        al = atan2(-dyl,dxl);
        ar = atan2(-dyr,dxr);
        
        % buid turn command
        turn = cos(ar);
        
        % prepare to plot turn data
        cla(handles.axw,'reset')
        set(handles.axw,'XTick',[],'YTick',[],'XTickLabel',{},'YTickLabel',{},'Box','off','Visible','off','Color','none')
        set(handles.axw,'XLim',[-1 1],'YLim',[-1 1])

        % plot turn data
        hold(handles.axw,'on');
        plot(handles.axw,[0 cos(ar)],[0 sin(ar)],'k','LineWidth',3)
        plot(handles.axw,cos(ar),sin(ar),'ko','MarkerSize',8,'MarkerFaceColor','r')
        hold(handles.axw,'off');
        
        % buid drive command
        lenle = abs(dyl);
        lenls = sqrt(dxls.^2+dyls.^2);
        lmag = lenle/(2*lenls);
        if(lmag>1) lmag=1; end
        drive = sign(-dyl)*lmag;
        
        % built start info from skeleton; if one moves left hand horizontally 
        % beyond the neck x position, then allow transfer of commands to
        % sliders; the sliders are used as a storage of commands to be used 
        % further by the robot
        dt = etime(clock(),t2); % wait >1 second to avoid multiple activations
        if x_lhand>x_neck && dt>1
            t2 = clock();
            if strcmp(get(handles.pushbutton_halt,'String'),'HALT')
                set(handles.slider_drive,'Value',0);
                set(handles.slider_turn,'Value',0);
                set(handles.pushbutton_halt,'String','GO')
            else
                set(handles.pushbutton_halt,'String','HALT')
                set(handles.slider_drive,'Value',0);
                set(handles.slider_turn,'Value',0);
            end
        end
        
        % transfer of commands to the sliders
        if strcmp(get(handles.pushbutton_halt,'String'),'HALT')
            set(handles.slider_drive,'Value',drive);
            set(handles.slider_turn,'Value',turn);
        end
        
    end
end



% --- Executes on slider movement.
function slider_drive_Callback(hObject, eventdata, handles)
% hObject    handle to slider_drive (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes on slider movement.
function slider_turn_Callback(hObject, eventdata, handles)
% hObject    handle to slider_turn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes on button press in pushbutton_halt.
function pushbutton_halt_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_halt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Go/Stop the transfer of camera comands
if strcmp(get(handles.pushbutton_halt,'String'),'GO')
    set(handles.pushbutton_halt,'String','HALT')
    set(handles.slider_drive,'Value',0);
    pause(0.5)
else
    set(handles.slider_drive,'Value',0);
    set(handles.pushbutton_halt,'String','GO')
    pause(0.5)
end
% Update handles structure
guidata(hObject, handles);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EV3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in pushbutton_BT.
function pushbutton_BT_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_BT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

robot = Brick('ioType','instrbt','btDevice','EV3','btChannel',1);   % Creates a "Brick" object connected with Matlab by bluetooth
robot.beep();       % One beep to show that the connection is working properly

% clear the tachometer
robot.outputClrCount(0,Device.MotorA);

% setting motor power
robot.outputPower(0,Device.MotorB,10);
robot.outputPower(0,Device.MotorC,10);
robot.outputStart(0,Device.MotorB);
robot.outputStart(0,Device.MotorC);
% robot moves forward for 0.1s
pause(0.1)
robot.outputStopAll()
pause(0.1)

set(handles.pushbutton_BT,'Enable','off');    % Disable the BT button
% set(handles.pushbutton_StopEV3,'Enable','on');  % Enable Stop button

handles.robot = robot;

%create a timer for calling the EV3 run function every 0.1 secs
handles.tmrev3 = timer('TimerFcn', {@startEV3_Callback,handles},'Period', 0.1,'executionMode','fixedRate');
start(handles.tmrev3)% start EV3 timer
set(handles.pushbutton_enableEV3,'Enable','on')

% Update handles structure
guidata(hObject, handles); 


% --- Executes on button press in pushbutton_enableEV3.
function pushbutton_enableEV3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_enableEV3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Enable comamnds to go to robot
if strcmp(get(handles.pushbutton_enableEV3,'String'),'Stop EV3')
    set(handles.pushbutton_enableEV3,'String','Start EV3')
    stop(handles.tmrev3)% start EV3 timer
    handles.robot.outputStopAll();
else
    start(handles.tmrev3)% start EV3 timer
    set(handles.pushbutton_enableEV3,'String','Stop EV3')
end
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbutton_moretilt.
function pushbutton_moretilt_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_moretilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.robot.outputPower(0,Device.MotorA,15);
% handles.robot.outputStepSpeed(0,Device.MotorA,10,0,90,0,Device.Brake);
handles.robot.outputStart(0,Device.MotorA);
pause(0.5)
handles.robot.outputStop(0,Device.MotorA,Device.Coast);


% --- Executes on button press in pushbutton_lesstilt.
function pushbutton_lesstilt_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_lesstilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.robot.outputPower(0,Device.MotorA,-15);
% handles.robot.outputStepSpeed(0,Device.MotorA,10,0,90,0,Device.Brake);
handles.robot.outputStart(0,Device.MotorA);
pause(0.5)
handles.robot.outputStop(0,Device.MotorA,Device.Coast);


function startEV3_Callback(tmrev3_obj, tmrev3_data, handles)
% function startAq_Callback(hObject, eventdata, handles)

% get commands from kinect (as stored on the sliders)
drive = get(handles.slider_drive,'Value')*80; % normalize drive at max 80% of power but one can go at 100%
turn = get(handles.slider_turn,'Value')*50; % turn at maximum 50% of power
driveB = (drive+turn);
driveC = (drive-turn);

% cut at max 100% power
if abs(driveB)>100
    driveB = sign(driveB)*100;
end
if abs(driveC)>100
    driveC = sign(driveC)*100;
end

% apply commends to the robot
robot = handles.robot;
robot.outputPower(0,Device.MotorB,driveB);
robot.outputPower(0,Device.MotorC,driveC);
robot.outputStart(0,Device.MotorB);
robot.outputStart(0,Device.MotorC);




% --- Executes when user attempts to close KEV3_gui.
function KEV3_gui_CloseRequestFcn(hObject, ~, handles)
% hObject    handle to KEV3_gui (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure

% verify if the robot was connected and if so, stop the motors and close the connection
ev3stat = get(handles.pushbutton_BT,'Enable');
if strcmp(ev3stat,'off')
    handles.robot.outputStopAll();
    pause(0.1)
    delete(handles.robot)
end

% clean up everything when closing the GUI
delete(hObject);
clear all
close all
% delete(imaqfind);


% --- Executes during object deletion, before destroying properties.
function KEV3_gui_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to KEV3_gui (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
