function varargout = Edge_map(varargin)
% EDGE_MAP M-file for Edge_map.fig
%      EDGE_MAP, by itself, creates a new EDGE_MAP or raises the existing
%      singleton*.
%
%      H = EDGE_MAP returns the handle to a new EDGE_MAP or the handle to
%      the existing singleton*.
%
%      EDGE_MAP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in EDGE_MAP.M with the given input arguments.
%
%      EDGE_MAP('Property','Value',...) creates a new EDGE_MAP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Edge_map_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Edge_map_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Edge_map

% Last Modified by GUIDE v2.5 19-Apr-2012 13:25:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Edge_map_OpeningFcn, ...
                   'gui_OutputFcn',  @Edge_map_OutputFcn, ...
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


% --- Executes just before Edge_map is made visible.
function Edge_map_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Edge_map (see VARARGIN)

% Choose default command line output for Edge_map
handles.output = hObject;

set(gcf,'CurrentAxes',handles.axes1)
handles.Image=getappdata(0,'hImage');
imshow(handles.Image(:,:,1),[]);
[m,n,k]=size(handles.Image);

if k>1                                          %If the image is a volume, we set the initial values of the slider
    set(handles.Slice_slider,'Value',1);
    set(handles.Slice_slider,'Max',k);
    set(handles.Slice_slider,'Min',1);
    set(handles.Slice_slider,'SliderStep',[1/(k-1) 1/(k-1)]);  %we set the step in the slider [max min]: both are given in percentages, we set it in a way to get the next slice each time the user press the slider or the arrow
end
set(handles.Slice_text,'String',1);   
set(handles.High_threshold_text,'String','0.5');
set(handles.High_threshold_slider,'Value',0.5);
handles.High_threshold=0.5;
set(handles.Low_threshold_text,'String','0.2');
set(handles.Low_threshold_slider,'Value',0.2);
handles.Low_threshold=0.2;
handles.Slice=1;
handles.BWimage=[];
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Edge_map wait for user response (see UIRESUME)
% uiwait(handles.Edge_map);


% --- Outputs from this function are returned to the command line.
function varargout = Edge_map_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in Algorithm_selection.
function Algorithm_selection_Callback(hObject, eventdata, handles)
% hObject    handle to Algorithm_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns Algorithm_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Algorithm_selection

number_choice = get(hObject, 'Value'); 
if number_choice==3
    handles.algorithm='Canny';
else
    handles.algorithm='';
end
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function Algorithm_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Algorithm_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function High_threshold_slider_Callback(hObject, eventdata, handles)
% hObject    handle to High_threshold_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

HT=get(hObject,'Value');
set(handles.High_threshold_text,'String',num2str(HT));
handles.High_threshold=HT;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function High_threshold_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to High_threshold_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function High_threshold_text_Callback(hObject, eventdata, handles)
% hObject    handle to High_threshold_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of High_threshold_text as text
%        str2double(get(hObject,'String')) returns contents of High_threshold_text as a double
HT=str2double(get(hObject,'String'));
set(handles.High_threshold_slider,'Value',HT);
handles.High_threshold=HT;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function High_threshold_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to High_threshold_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Low_threshold_text_Callback(hObject, eventdata, handles)
% hObject    handle to Low_threshold_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Low_threshold_text as text
%        str2double(get(hObject,'String')) returns contents of Low_threshold_text as a double
LT=str2double(get(hObject,'String'));
set(handles.Low_threshold_slider,'Value',LT);
handles.Low_threshold=LT;
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function Low_threshold_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Low_threshold_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function Low_threshold_slider_Callback(hObject, eventdata, handles)
% hObject    handle to Low_threshold_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
LT=get(hObject,'Value');
set(handles.Low_threshold_text,'String',num2str(LT));
handles.Low_threshold=LT;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function Low_threshold_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Low_threshold_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Slice_slider_Callback(hObject, eventdata, handles)
% hObject    handle to Slice_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Slice=get(hObject,'Value');
set(handles.Slice_text,'String',num2str(Slice));
handles.Slice=Slice;
drawing(handles);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function Slice_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slice_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function Slice_text_Callback(hObject, eventdata, handles)
% hObject    handle to Slice_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Slice_text as text
%        str2double(get(hObject,'String')) returns contents of Slice_text as a double

Slice=str2double(get(hObject,'String'));
set(handles.Slice_slider,'Value',Slice);
handles.Slice=Slice;
drawing(handles);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function Slice_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slice_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in Apply.
function Apply_Callback(hObject, eventdata, handles)
% hObject    handle to Apply (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
method=handles.algorithm;
if strcmp(method, 'Canny')
    for i=1:size(handles.Image,3)
       %a good threshold for segmenting the Trachea is [Thigh:0.2 Tlow:0.1]
        handles.BWimage(:,:,i)=edge(handles.Image(:,:,i),method,[handles.Low_threshold handles.High_threshold]);
    end
end
guidata(hObject,handles);


% --- Executes on button press in Proceed.
function Proceed_Callback(hObject, eventdata, handles)
% hObject    handle to Proceed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setappdata(0,'hBW',handles.BWimage);
close(Edge_map);

function drawing(handles)

set(gcf,'CurrentAxes',handles.axes1)
imshow(handles.Image(:,:,handles.Slice),[]);
if ~isempty(handles.BWimage)
    set(gcf,'CurrentAxes',handles.axes3)
    imshow(handles.BWimage(:,:,handles.Slice),[]);
end
axis off;
