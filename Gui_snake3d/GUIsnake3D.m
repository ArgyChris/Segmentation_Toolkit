function varargout = GUIsnake3D(varargin)
% GUISNAKE3D M-file for GUIsnake3D.fig
%      GUISNAKE3D, by itself, creates a new GUISNAKE3D or raises the existing
%      singleton*.
%
%      H = GUISNAKE3D returns the handle to a new GUISNAKE3D or the handle to
%      the existing singleton*.
%
%      GUISNAKE3D('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUISNAKE3D.M with the given input arguments.
%
%      GUISNAKE3D('Property','Value',...) creates a new GUISNAKE3D or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUIsnake3D_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUIsnake3D_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUIsnake3D

% Last Modified by GUIDE v2.5 18-Apr-2012 18:50:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUIsnake3D_OpeningFcn, ...
                   'gui_OutputFcn',  @GUIsnake3D_OutputFcn, ...
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


% --- Executes just before GUIsnake3D is made visible.
function GUIsnake3D_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUIsnake3D (see VARARGIN)

% Choose default command line output for GUIsnake3D
handles.output = hObject;
set(gcf,'CurrentAxes',handles.axes1)
handles.size_of_axis1_reset=axis;
imshow('snake.jpeg');
set(gcf,'CurrentAxes',handles.axes2)
handles.size_of_axis2_reset=axis;
imshow('noose.jpeg');
set(gcf,'CurrentAxes',handles.axes3)
handles.size_of_axis3_reset=axis;
membrane_data=membrane;
surf(membrane_data);
set(gcf,'CurrentAxes',handles.axes8)
imshow('logo2.jpg');

set(handles.edit1,'String','1');
set(handles.edit3,'String','1');
set(handles.Radius_text,'String','1');

set(handles.Alpha,'string','0.2');
set(handles.Beta,'string','0.1');
set(handles.Tau,'string','0.25');
set(handles.Sigma,'string','6');
set(handles.Iterations,'string','500'); %500 iterations for GVF,VFC
set(handles.Mu,'string','0.2');
set(handles.Radius,'string','32');
set(handles.Gamma,'string','2.5');
set(handles.Tolerance,'string','250'); %250 for adaptive GVF/CVF

set(handles.Result_iterations,'string','0');
set(handles.Result_time,'string','0');

handles.slice_snake_slct=[];
handles.snaxels=cell(1,0);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUIsnake3D wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUIsnake3D_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Select_Folder.
function Select_Folder_Callback(hObject, eventdata, handles)
% hObject    handle to Select_Folder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.Select_Folder=uigetdir;                 %shows a gui for folder selection
handles.Output_folder=[];
set(handles.Output_folder,'String',handles.Select_Folder);
folder_name=handles.Select_Folder;              %we get the folder name
struct_dir=dir(folder_name);                    %we create a struct where we place all the files of the selected folder
length_dir=length(struct_dir);      
p=struct([]);
for i=1:length_dir                              %we find all the files in the specific folder
    p{end+1}=struct_dir(i).name;
end
set(handles.Files,'String',p);                  %assign to the listbox the files that the selected folder contains
guidata(hObject,handles);

% --- Executes on selection change in Files.
function Files_Callback(hObject, eventdata, handles)
% hObject    handle to Files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns Files contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Files

index_selected=get(hObject,'Value');                %we get the selection=the value from the listbox
list = get(hObject,'String');                       %the files in the selected folder
file_selected = list{index_selected};               %selects the specific file
handles.Files=file_selected;            
back_lash='\';
file=strcat(handles.Select_Folder,back_lash);       %we add the back_lash character in order to form the correct path
root_dir=cd;                                            
cd (file)                                           %we change the current directory to get the selected file/image
if ~isempty(regexpi(list{index_selected},'.bmp')) || ~isempty(regexpi(list{index_selected},'.tiff'))...
        || ~isempty(regexpi(list{index_selected},'.png')) || ~isempty(regexpi(list{index_selected},'.jpeg')) || ~isempty(regexpi(list{index_selected},'.mat'))
    if  ~isempty(regexpi(list{index_selected},'.mat'))  %if the file is .mat then is loaded in a struct, and then we have to convert the struct to cell in order to extract the image
        S=load(list{index_selected});
        c=struct2cell(S);
        I=c{1};
    else
        I=imread(list{index_selected}); 
    end
    [m,n,k]=size(I);
    set(gcf,'CurrentAxes',handles.axes1)            %We select the first axes to show the initial image
    imshow(I(:,:,1),[]);                               %We show the first slice, if it has only one it shows that, else if it is a volume it shows again the first slice
    axis off;                               
    handles.current_slice=1;                    
    handles.size_slice=k;
    handles.Image=I;
    
    if k>1                                          %If the image is a volume, we set the initial values of the slider
        set(handles.Slider,'Value',1);
        set(handles.Slider,'Max',k);
        set(handles.Slider,'Min',1);
        set(handles.Slider,'SliderStep',[1/(k-1) 1/(k-1)]);  %we set the step in the slider [max min]: both are given in percentages, we set it in a way to get the next slice each time the user press the slider or the arrow
        
        set(handles.Radius_slider,'Value',1);
        set(handles.Radius_slider,'Max',30);
        set(handles.Radius_slider,'Min',1);
        set(handles.Radius_slider,'SliderStep',[1/(30-1) 1/(30-1)]);
    end
end
cd (root_dir);

guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function Files_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double

if isfield(handles,'size_slice') && handles.size_slice>1 
    no_slice=str2double(get(hObject,'String'));
    if (no_slice>=1) && (no_slice<=handles.size_slice) && ~(rem(no_slice,1))
        set(handles.Slider,'Value',no_slice);
        handles.current_slice=no_slice;
        drawing(handles);
        guidata(hObject,handles);
    end
end

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function Slider_Callback(hObject, eventdata, handles)
% hObject    handle to Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

if isfield(handles,'size_slice') && handles.size_slice>1
    no_slice=get(hObject,'Value');
    if (no_slice>=1) && (no_slice<=handles.size_slice) && ~(rem(no_slice,1))
        set(handles.edit1,'String',num2str(no_slice));
        handles.current_slice=no_slice;
        drawing(handles);
        guidata(hObject,handles);
    end
end

% --- Executes during object creation, after setting all properties.
function Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function Radius_slider_Callback(hObject, eventdata, handles)
% hObject    handle to Radius_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

Radius_circle=get(hObject,'Value');
if (Radius_circle>=1) && (Radius_circle<=30)
    set(handles.Radius_text,'String',num2str(Radius_circle));                   %We set the value in slider
    handles.Radius_diameter=Radius_circle;
    guidata(hObject,handles);
end

% --- Executes during object creation, after setting all properties.
function Radius_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Radius_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function Radius_text_Callback(hObject, eventdata, handles)
% hObject    handle to Radius_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Radius_text as text
%        str2double(get(hObject,'String')) returns contents of Radius_text as a double

Radius_circle=str2double(get(hObject,'String'));
if (Radius_circle>=1) && (Radius_circle<=30) 
    set(handles.Radius_slider,'Value',Radius_circle);
    handles.Radius_diameter=Radius_circle;
    guidata(hObject,handles);
end

% --- Executes during object creation, after setting all properties.
function Radius_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Radius_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in Initialize_snake.
function Initialize_snake_Callback(hObject, eventdata, handles)
% hObject    handle to Initialize_snake (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if strcmp(handles.Initialization,'General_cylinder')
    if handles.current_slice==handles.slice_snake_slct
        delete(handles.getpts);
    end
    [x,y]=getpts;                                   %We select the points based on the in-built getpts() function
    Center_of_circle=[x,y]; % centre of the initial snake
    Radius=get(handles.Radius_slider,'Value');
    Number_of_points=25; 
    Initial_Snake=round([Center_of_circle(1)+Radius*cos(0:(2*pi/(Number_of_points-1)):(2*pi));Center_of_circle(2)+Radius*sin(0:(2*pi/(Number_of_points-1)):(2*pi)) ]);
    handles.selected_points=1;                      %flag that we selected the points
    originalSpacing = 1 : length(Initial_Snake(1,:));                %We increase the spacing, we apply splines
    finerSpacing = 1 : 0.1 : length(Initial_Snake(1,:));         
    vertex=[Initial_Snake(1,:);Initial_Snake(2,:)];
    splineXY = spline(originalSpacing, vertex, finerSpacing);
    hold on;
    handles.getpts=plot(vertex(1, :), vertex(2, :), 'r*', splineXY(1, :), splineXY(2, :));    %We plot the selected points
    hold off;
    handles.snaxels{end+1}=splineXY;                       %We save the selected points
    handles.slice_snake_slct=handles.current_slice;
    
elseif strcmp(handles.Initialization,'Adaptive_cylinder')
    if ~isempty(handles.slice_snake_slct) && (handles.current_slice==handles.slice_snake_slct(end))
        delete(handles.getpts);
    end
    [x,y]=getpts;                                   %We select the points based on the in-built getpts() function
    Center_of_circle=[x,y]; % centre of the initial snake
    Radius=get(handles.Radius_slider,'Value');
%   Number_of_points=25; %for splines
    Number_of_points=70; 
    Initial_Snake=round([Center_of_circle(1)+Radius*cos(0:(2*pi/(Number_of_points-1)):(2*pi));Center_of_circle(2)+Radius*sin(0:(2*pi/(Number_of_points-1)):(2*pi)) ]);    
    %Here I dont use splines
%     originalSpacing = 1 : length(Initial_Snake(1,:));                %We increase the spacing, we apply splines
%     finerSpacing = 1 : 0.1 : length(Initial_Snake(1,:));         
    vertex=[Initial_Snake(1,:);Initial_Snake(2,:)];
%     splineXY = spline(originalSpacing, vertex, finerSpacing);
    hold on;
%     handles.getpts=plot(vertex(1, :), vertex(2, :), 'r*', splineXY(1, :), splineXY(2, :));    %We plot the selected points
    handles.getpts=plot(vertex(1, :), vertex(2, :), 'r*');    %We plot the selected points
    hold off;   
    %
    index=find(handles.slice_snake_slct==handles.current_slice);
    if ~isempty(index)
%         handles.snaxels{index}=splineXY;
        handles.snaxels{index}=vertex;
        handles.slice_snake_slct(index)=handles.current_slice;
    else
%         handles.snaxels{end+1}=splineXY;
        handles.snaxels{end+1}=vertex;
        handles.slice_snake_slct(end+1)=handles.current_slice;
    end

else
    
end
guidata(hObject,handles);

% --- Executes on button press in Finish_initialization.
function Finish_initialization_Callback(hObject, eventdata, handles)
% hObject    handle to Finish_initialization (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%We have different cases: In this part we intialize the snake from the first slice to the last slice
if strcmp(handles.Initialization,'General_cylinder')
    Contour=repmat(handles.snaxels{end},[1 1 handles.size_slice]);
elseif strcmp(handles.Initialization,'Adaptive_cylinder')
    selected_slices=handles.slice_snake_slct;
    Contour=[];
    if handles.slice_snake_slct(1)==1
        for i=1:length(selected_slices)-1
            slice_start=handles.slice_snake_slct(i);
            slice_end=handles.slice_snake_slct(i+1);
            snake_slice_start=handles.snaxels{i};
            snake_slice_end=handles.snaxels{i+1};
            distance_slice=slice_end-slice_start-1;
            if distance_slice>=1
                x_all=zeros(distance_slice,length(snake_slice_start));
                y_all=zeros(distance_slice,length(snake_slice_start));
                for j=1:length(snake_slice_start)
                    x_all(:,j)=interp1([slice_start,slice_end],[snake_slice_start(1,j),snake_slice_end(1,j)],[slice_start+1:slice_end-1])';
                    y_all(:,j)=interp1([slice_start,slice_end],[snake_slice_start(2,j),snake_slice_end(2,j)],[slice_start+1:slice_end-1])';
                end
                Contour(:,:,end+1)=snake_slice_start;
                if i==1
                    Contour(:,:,1)=[];
                end
                for k=1:size(x_all,1)
                    Contour(:,:,end+1)=cat(1,x_all(k,:),y_all(k,:));
                end  
            else
                Contour(:,:,end+1)=snake_slice_start;
            end
        end
        last_selection=selected_slices(end);
        if last_selection<=handles.size_slice
            last_slice=Contour(:,:,end);
            for l=last_selection:handles.size_slice
                Contour(:,:,end+1)=last_slice;
            end
        end
    %In this case we do not want to initialize the surface in the first slice, and we do not want also to finish it in the last slice of the volume
    elseif handles.slice_snake_slct(1)>1
        for i=1:length(selected_slices)-1
            slice_start=handles.slice_snake_slct(i);
            slice_end=handles.slice_snake_slct(i+1);
            snake_slice_start=handles.snaxels{i};
            snake_slice_end=handles.snaxels{i+1};
            distance_slice=slice_end-slice_start-1;
            if distance_slice>=1
                x_all=zeros(distance_slice,length(snake_slice_start));
                y_all=zeros(distance_slice,length(snake_slice_start));
                for j=1:length(snake_slice_start)
                    x_all(:,j)=interp1([slice_start,slice_end],[snake_slice_start(1,j),snake_slice_end(1,j)],[slice_start+1:slice_end-1])';
                    y_all(:,j)=interp1([slice_start,slice_end],[snake_slice_start(2,j),snake_slice_end(2,j)],[slice_start+1:slice_end-1])';
                end
                Contour(:,:,end+1)=snake_slice_start;
                if i==1
                    Contour(:,:,1)=[];
                end
                for k=1:size(x_all,1)
                    Contour(:,:,end+1)=cat(1,x_all(k,:),y_all(k,:));
                end  
            else
                Contour(:,:,end+1)=snake_slice_start;
            end
        end        
    else
        
    end

else
    
end
handles.Contour=Contour;
guidata(hObject,handles);


% --- Executes on button press in Show_contour.
function Show_contour_Callback(hObject, eventdata, handles)
% hObject    handle to Show_contour (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.Vertex2DX=zeros(size(handles.Contour,2),size(handles.Contour,3));
handles.Vertex2DY=zeros(size(handles.Contour,2),size(handles.Contour,3));
handles.Vertex2DZ=zeros(size(handles.Contour,2),size(handles.Contour,3));

set(gcf,'CurrentAxes',handles.axes2);
imshow(handles.Image(:,:,1),[]); 
axis off;                               
handles.current_contour=1;
hold on;
plot(handles.Contour(1,:,handles.current_contour),handles.Contour(2,:,handles.current_contour),'r');
hold off;

set(handles.Slider2,'Value',1);
set(handles.Slider2,'Min',1);
set(handles.Slider2,'Max',size(handles.Contour,3));
set(handles.Slider2,'SliderStep',[1/(size(handles.Contour,3)-1) 1/(size(handles.Contour,3)-1)]);
set(handles.edit3,'String',num2str(2));

for i=1:size(handles.Contour,3)
    handles.Vertex2DX(:,i)=handles.Contour(1,:,i);
    handles.Vertex2DY(:,i)=handles.Contour(2,:,i);
    handles.Vertex2DZ(:,i)=i;    
end

set(gcf,'CurrentAxes',handles.axes3); 
surf(double(handles.Vertex2DX),double(handles.Vertex2DY),double(handles.Vertex2DZ));

guidata(hObject,handles);

% --- Executes on button press in Edge_map.
function Edge_map_Callback(hObject, eventdata, handles)
% hObject    handle to Edge_map (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setappdata(0,'hImage',handles.Image)
Edge_map
handles.BW=getappdata(0,'hBW');
guidata(hObject,handles);


% --- Executes on button press in Evaluate_snake.
function Evaluate_snake_Callback(hObject, eventdata, handles)
% hObject    handle to Evaluate_snake (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Sigma_val = str2double(get(handles.Sigma,'String'));    %We get the sigma
handles.Sigma_val=Sigma_val;
iterations=str2double(get(handles.Iterations,'String'));
tolerance=str2double(get(handles.Tolerance,'String'));
snake_Vertex2DX=handles.Vertex2DX;
snake_Vertex2DY=handles.Vertex2DY;
snake_Vertex2DZ=handles.Vertex2DZ;
Alpha=str2double(get(handles.Alpha,'String'));
Beta=str2double(get(handles.Beta,'String'));
Tau=str2double(get(handles.Tau,'String'));
Mu=str2double(get(handles.Mu,'String'));
result_time=0;
i=1;
s0=1;
if strcmp(handles.Solving_VF,'GVF') && strcmp(handles.Solving_model,'Explicit')                %We check which model is defined
     h = fspecial('gaussian',[floor(handles.Sigma_val) floor(handles.Sigma_val)],handles.Sigma_val);
%      f = imfilter(double(1-handles.Image),h);     %for the original U image
     f=imfilter(handles.BW,h);
     Fext = AM_GVF(f, Mu , 100, 1); 
     while (i<iterations+1) 
         tic
         [snake_Vertex2DX(:,:,end+1),snake_Vertex2DY(:,:,end+1),snake_Vertex2DZ(:,:,end+1)]= AC_deform3D(snake_Vertex2DX(:,:,end),snake_Vertex2DY(:,:,end),snake_Vertex2DZ(:,:,end),Alpha,Beta,Tau,Fext,5);
         elasped_time=toc;
         if mod(i,1)==0
             if s0==size(snake_Vertex2DZ,2) 
                s0=1;
             end
             s0=s0+1;  
             set(gcf,'CurrentAxes',handles.axes2);
             imshow(handles.Image(:,:,round(snake_Vertex2DZ(1,s0))),[]); 
             hold on;
             plot(snake_Vertex2DX(:,s0,end),snake_Vertex2DY(:,s0,end),'g');
             drawnow
             hold off;           
             xlabel({['GVF' ' iteration ' num2str(i), ' slice: ', int2str(s0)]})
             set(gcf,'CurrentAxes',handles.axes3);
             surf(double(snake_Vertex2DX(:,:,end)),double(snake_Vertex2DY(:,:,end)),double(snake_Vertex2DZ(:,:,end)));
             drawnow
         end
         result_time=result_time+elasped_time;
         set(handles.Result_iterations,'String',sprintf('%f',i));
         set(handles.Result_time,'String',sprintf('%f', result_time));
         distance=sum(sum(sqrt(((snake_Vertex2DX(:,:,end)-snake_Vertex2DX(:,:,end-1)).^2+((snake_Vertex2DY(:,:,end)-snake_Vertex2DY(:,:,end-1)).^2)+((snake_Vertex2DZ(:,:,end)-snake_Vertex2DZ(:,:,end-1)).^2)))))
         if (distance<=tolerance)
            break;
         end
         i=i+1;
     end   
elseif strcmp(handles.Solving_VF,'VFC') && strcmp(handles.Solving_model,'Explicit') 
    Radius=str2double(get(handles.Radius,'String'));
    Gamma=str2double(get(handles.Gamma,'String'));
    K = AM_VFK(3, Radius, 'power',Gamma);
    Fext=AM_VFC(handles.BW,K,1);
%     Fext = AM_VFC(1-handles.Image, K, 1);
    while (i<iterations+1) 
        tic
        [snake_Vertex2DX(:,:,end+1),snake_Vertex2DY(:,:,end+1),snake_Vertex2DZ(:,:,end+1)]= AC_deform3D(snake_Vertex2DX(:,:,end),snake_Vertex2DY(:,:,end),snake_Vertex2DZ(:,:,end),Alpha,Beta,Tau,Fext,5);
        elasped_time=toc;
        if mod(i,1)==0
            if s0==size(snake_Vertex2DZ,2) 
               s0=1;
            end
            s0=s0+1;  
            set(gcf,'CurrentAxes',handles.axes2);
            imshow(handles.Image(:,:,round(snake_Vertex2DZ(1,s0))),[]); 
            hold on;
            plot(snake_Vertex2DX(:,s0,end),snake_Vertex2DY(:,s0,end),'g');
            drawnow
            hold off;
            xlabel({['VFC' ' iteration ' num2str(i), ' slice: ', int2str(s0)]})
            set(gcf,'CurrentAxes',handles.axes3);
            surf(double(snake_Vertex2DX(:,:,end)),double(snake_Vertex2DY(:,:,end)),double(snake_Vertex2DZ(:,:,end)));
            drawnow
        end
        result_time=result_time+elasped_time;
        set(handles.Result_iterations,'String',sprintf('%f',i));
        set(handles.Result_time,'String',sprintf('%f',result_time));
        distance=sum(sum(sqrt(((snake_Vertex2DX(:,:,end)-snake_Vertex2DX(:,:,end-1)).^2+((snake_Vertex2DY(:,:,end)-snake_Vertex2DY(:,:,end-1)).^2)+((snake_Vertex2DZ(:,:,end)-snake_Vertex2DZ(:,:,end-1)).^2)))))
        if (distance<=tolerance)
            break;
        end
        i=i+1;
    end
elseif strcmp(handles.Solving_VF,'GVF') && strcmp(handles.Solving_model,'Semi-implicit') 
    %nothing
elseif strcmp(handles.Solving_VF,'VFC') && strcmp(handles.Solving_model,'Semi-implicit') 
    %nothing
else
    %nothing
end
save ('experiment.mat', 'snake_Vertex2DX', 'snake_Vertex2DY', 'snake_Vertex2DZ')
guidata(hObject,handles);

% --- Executes on selection change in Initialization.
function Initialization_Callback(hObject, eventdata, handles)
% hObject    handle to Initialization (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns Initialization contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Initialization

number_choice = get(hObject, 'Value');                  %We get the selected value
switch number_choice
    case 1
        handles.Initialization='General_cylinder';               
    case 2
        handles.Initialization='Adaptive_cylinder';    
end
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function Initialization_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Initialization (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function Slider2_Callback(hObject, eventdata, handles)
% hObject    handle to Slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

if isfield(handles,'current_contour') 
    no_contour=get(hObject,'Value');
    if (no_contour>=1) && (no_contour<=handles.size_slice) && ~(rem(no_contour,1))
        set(handles.edit3,'String',num2str(no_contour));
        handles.current_contour=no_contour;
        drawing2(handles);
        guidata(hObject,handles);
    end
end

% --- Executes during object creation, after setting all properties.
function Slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double

if isfield(handles,'current_contour')
    no_contour=str2double(get(hObject,'String'));
    if (no_contour>=1) && (no_contour<=handles.size_slice) && ~(rem(no_contour,1))
        set(handles.Slider2,'Value',no_contoure);
        handles.current_contour=no_contour;
        drawing2(handles);
        guidata(hObject,handles);
    end
end

% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in Solving_VF.
function Solving_VF_Callback(hObject, eventdata, handles)
% hObject    handle to Solving_VF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns Solving_VF contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Solving_VF

number_choice = get(hObject, 'Value');
switch number_choice
    case 1
        handles.Solving_VF='VFC';
    case 2
        handles.Solving_VF='GVF';    
end
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function Solving_VF_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Solving_VF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Solving_model.
function Solving_model_Callback(hObject, eventdata, handles)
% hObject    handle to Solving_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns Solving_model contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Solving_model

number_choice = get(hObject, 'Value');                  %We get the selected value
switch number_choice
    case 1
        handles.Solving_model='Explicit';               
    case 2
        handles.Solving_model='Semi-implicit';    
end
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function Solving_model_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Solving_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Alpha_Callback(hObject, eventdata, handles)
% hObject    handle to Alpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Alpha as text
%        str2double(get(hObject,'String')) returns contents of Alpha as a double


% --- Executes during object creation, after setting all properties.
function Alpha_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Alpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Beta_Callback(hObject, eventdata, handles)
% hObject    handle to Beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Beta as text
%        str2double(get(hObject,'String')) returns contents of Beta as a double


% --- Executes during object creation, after setting all properties.
function Beta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tau_Callback(hObject, eventdata, handles)
% hObject    handle to Tau (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tau as text
%        str2double(get(hObject,'String')) returns contents of Tau as a double


% --- Executes during object creation, after setting all properties.
function Tau_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tau (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Iterations_Callback(hObject, eventdata, handles)
% hObject    handle to Iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Iterations as text
%        str2double(get(hObject,'String')) returns contents of Iterations as a double


% --- Executes during object creation, after setting all properties.
function Iterations_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tolerance_Callback(hObject, eventdata, handles)
% hObject    handle to Tolerance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tolerance as text
%        str2double(get(hObject,'String')) returns contents of Tolerance as a double


% --- Executes during object creation, after setting all properties.
function Tolerance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tolerance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Mu_Callback(hObject, eventdata, handles)
% hObject    handle to Mu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mu as text
%        str2double(get(hObject,'String')) returns contents of Mu as a double


% --- Executes during object creation, after setting all properties.
function Mu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Sigma_Callback(hObject, eventdata, handles)
% hObject    handle to Sigma (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Sigma as text
%        str2double(get(hObject,'String')) returns contents of Sigma as a double


% --- Executes during object creation, after setting all properties.
function Sigma_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sigma (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Radius_Callback(hObject, eventdata, handles)
% hObject    handle to Radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Radius as text
%        str2double(get(hObject,'String')) returns contents of Radius as a double


% --- Executes during object creation, after setting all properties.
function Radius_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Gamma_Callback(hObject, eventdata, handles)
% hObject    handle to Gamma (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Gamma as text
%        str2double(get(hObject,'String')) returns contents of Gamma as a double


% --- Executes during object creation, after setting all properties.
function Gamma_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Gamma (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Result_iterations_Callback(hObject, eventdata, handles)
% hObject    handle to Result_iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Result_iterations as text
%        str2double(get(hObject,'String')) returns contents of Result_iterations as a double


% --- Executes during object creation, after setting all properties.
function Result_iterations_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Result_iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Result_time_Callback(hObject, eventdata, handles)
% hObject    handle to Result_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Result_time as text
%        str2double(get(hObject,'String')) returns contents of Result_time as a double


% --- Executes during object creation, after setting all properties.
function Result_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Result_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in Reset.
function Reset_Callback(hObject, eventdata, handles)
% hObject    handle to Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = ResetGUI(handles);
guidata(hObject,handles);

function handles = ResetGUI(handles)
 % Code to reset various controls to known states
 % For example set scrollbar values to known initial values.
 

function drawing(handles)

set(gcf,'CurrentAxes',handles.axes1)
imshow(handles.Image(:,:,handles.current_slice),[]);
axis off;

function drawing2(handles)

set(gcf,'CurrentAxes',handles.axes2)
imshow(handles.Image(:,:,handles.current_contour),[]);
axis off;
hold on;
plot(handles.Contour(1,:,handles.current_contour),handles.Contour(2,:,handles.current_contour),'r');
hold off;
