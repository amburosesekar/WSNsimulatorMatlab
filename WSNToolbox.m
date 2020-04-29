function varargout = WSNToolbox(varargin)
% WSNTOOLBOX MATLAB code for WSNToolbox.fig
%      WSNTOOLBOX, by itself, creates a new WSNTOOLBOX or raises the existing
%      singleton*.
%
%      H = WSNTOOLBOX returns the handle to a new WSNTOOLBOX or the handle to
%      the existing singleton*.
%
%      WSNTOOLBOX('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in WSNTOOLBOX.M with the given input arguments.
%
%      WSNTOOLBOX('Property','Value',...) creates a new WSNTOOLBOX or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before WSNToolbox_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to WSNToolbox_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help WSNToolbox

% Last Modified by GUIDE v2.5 29-Apr-2020 15:22:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @WSNToolbox_OpeningFcn, ...
                   'gui_OutputFcn',  @WSNToolbox_OutputFcn, ...
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


% --- Executes just before WSNToolbox is made visible.
function WSNToolbox_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to WSNToolbox (see VARARGIN)
% Choose default command line output for WSNToolbox
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
% UIWAIT makes WSNToolbox wait for user response (see UIRESUME)
% uiwait(handles.figure1);
I=imread('Logo.jpg');
imshow(I)


% --- Outputs from this function are returned to the command line.
function varargout = WSNToolbox_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


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



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


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

function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double
% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double
global Rc
Rc=(str2double(get(handles.edit7,'String')));

% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2


% --- Executes on button press in checkbox3.
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global E X Y Xb Yb N max1 SN Rc stop1


alpha=0.01;%0.001 energy per distance% 0.001                                 %% node to CH power Ratio
beta=0.045;%0.0015                                                           %% CH to sink distance power ratio

Sector1=get(handles.checkbox4,'value');
nodes=str2double(get(handles.edit1,'String'));% Total No. of Nodes
E=(str2double(get(handles.edit6,'String'))).*ones(1,nodes);
nodesCH=str2double(get(handles.edit8,'String'));
EexL=E;
E1=E;
%% Ch selection
chalg1=get(handles.edit9,'String');
chalg=strrep(chalg1,'.m','');
varName=matlab.lang.makeValidName(chalg);
chselalg=str2func(varName)
[CH]=chselalg(EexL,nodesCH)

%% Routing

if (get(handles.radiobutton3,'Value') == 0 && get(handles.radiobutton4,'Value') == 0 &&  get(handles.radiobutton5,'Value') == 0)
    %user did not select any radio button, then do this
  else
    %do this if a user selected a radio button
      switch get(get(handles.uibuttongroup6,'SelectedObject'),'Tag')
      case 'radiobutton3',  Route_type = 'direct';
      case 'radiobutton4',  Route_type = 'chleach';
      case 'radiobutton5',  Route_type = 'custom';    
      end
end

if(strcmp(Route_type,'custom'))
    Ralg1=get(handles.edit10,'String');
    Ralg=strrep(Ralg1,'.m','');
    varName=matlab.lang.makeValidName(Ralg);
    Ralgfun=str2func(varName)    
    
end

%% EnergyFunction
Efun1=get(handles.edit11,'String');
Efun=strrep(Efun1,'.m','');
varName=matlab.lang.makeValidName(Efun);
Efun=str2func(varName)
if(strcmp(Route_type,'direct'))
    A=1;
elseif(strcmp(Route_type,'chleach'))
    A=2;
else
    A=3;
end
%%



%% ThroughputFn
Thfun1=get(handles.edit12,'String');
Thfun=strrep(Thfun1,'.m','');
varName=matlab.lang.makeValidName(Thfun);
Thfun=str2func(varName)
%% Simulation Starts
round=str2double(get(handles.edit14,'String'));
roundDelay=str2double(get(handles.edit13,'String'));
%%
ipp=1;  
Alivenodes=zeros(1,round);
AvgEc=zeros(1,round);
Throughput=zeros(1,round);
d=1;
apL=1;


%%
X1=[X Xb];
Y1=[Y Yb];
matrizP=pdist2([X1 ;Y1]',[X1; Y1]');
%%


%N=nodes;
%N1=1:nodes;
while(ipp<round) 
     EneExL(ipp)=0;
     
 %%Leach
  %% Leach
       A1=randperm(N);                                                      % Randomly select Source node
       ind=A1(3);
       pathL=[];
       if(A==1)%% Direct
          dist2Leach=pdist2([Xb;Yb]',[X(ind);Y(ind)]');                     
          valp=dist2Leach;
          dist2L=dist2Leach;
          indp=ind;
          v2=1;
          dist1L=0;
          dist2L=dist2Leach;          
          
          apL=0;
          if(( dist2Leach <(Rc)) &&  EexL(ind)~=0 )
              pathL=[ind -1];                                                   % Sink Node Assign as PathL=-1
              apL=apL+1;
              path11=pathL(1:end-1);
                EexL(path11)=EexL(path11)-Efun(alpha,beta,dist1L,dist2L,A);
              EneExL(ipp)=Efun(alpha,beta,dist1L,dist2L,A).*numel(path11);  
          end                                    
            
          
            
       
       end
       
       if(A==2)%% LEACH
       indLeach=CH;                                                         % CH of LEACH
       vl=find(indLeach==ind,1);                                            % Verify node and CH are same
       v2=1;
       if(~isempty(vl))
          dist2Leach=pdist2([Xb;Yb]',[X(ind);Y(ind)]');                     %if same data direct transfer to sink
          valp=dist2Leach;
          dist2L=dist2Leach;
          indp=ind;
          v2=1;
          pathL1=[ind -1];                                                   % Sink Node Assign as PathL=-1
       else
         for ib=1:numel(indLeach)  
            dist22L(ib)=pdist2([X(indLeach(ib));Y(indLeach(ib))]',[X(ind);Y(ind)]');        % verify which CH nearest to Node
         end
            [valp,indp]=min(dist22L);        
            dist1L=pdist2([X(indLeach(indp));Y(indLeach(indp))]',[X(ind);Y(ind)]');% node to XH
            dist2L=pdist2([Xb;Yb]',[X(indLeach(indp));Y(indLeach(indp))]'); % CH to sink   
            v2=2;
             pathL1=[ind indLeach(indp) -1];
       end     
            dist1L(isempty(dist1L))=0;
            dist2L(isempty(dist2L))=0;            
           
            apL=0;
            if(( valp <(Rc)) &&  EexL(ind)~=0 )
                pathL=pathL1;
                apL=apL+1;
                path11=pathL(1:end-1);
                EexL(path11)=EexL(path11)-Efun(alpha,beta,dist1L,dist2L,A);
              EneExL(ipp)=Efun(alpha,beta,dist1L,dist2L,A).*numel(path11);  
                
            end                                    

       end
       
       
       
       
       %% Custom Adhoc Routing
       if(A==3)
           
          Source =ind;
          Dest=numel(X1);
          Rc1=Rc;%4.* 
          matrizP(matrizP>Rc1)=inf;
          [pathP,cost]=Ralgfun(Source,Dest,matrizP);
          costN=cost.*length(pathP);
          
          dist1L=costN;
          dist2L=0;
          apL=0;
          if(~isempty(pathP))
              
            if(pathP(end)~=Dest)
                 pathP=[pathP Dest];       
            end
          
          
               
          if(EexL(ind)~=0 )
              pathP(pathP==Dest)=-1;
              pathL=pathP;%[ind -1]; 
              path11=pathP;
              path11(path11==-1)=[];
              % Sink Node Assign as PathL=-1
              apL=apL+1;
              path11
              EexL(path11)=EexL(path11)-Efun(alpha,beta,dist1L,dist2L,A);
              EneExL(ipp)=Efun(alpha,beta,dist1L,dist2L,A).*numel(path11);  
          end  
          end              
       end
       %%

            %%
            
            %EexL(ind)=EexL(ind)-alpha.*dist1L -(beta+0.5).*dist2L;
         
                    
            
            %id1(ind)
            %EexL(id1(ind))
            
            
            if(EexL(ind)<=0)
               EexL(ind)=0; 
            end
            
            
 %%
 if(Sector1==1)
 if(~isempty(pathL))
     path1=pathL;
    for p =1:(size(path1,2))-1
            if(path1(p+1)==-1)
                line([X(path1(p)) Xb], [Y(path1(p)) Yb], 'Color','g','LineWidth',2, 'LineStyle','-') 
                arrow([X(path1(p)) Y(path1(p)) ], [Xb Yb ])
            else
                line([X(path1(p)) X(path1(p+1))], [Y(path1(p)) Y(path1(p+1))], 'Color','g','LineWidth',2, 'LineStyle','-') 
                arrow([X(path1(p)) Y(path1(p)) ], [X(path1(p+1)) Y(path1(p+1))])
            end
            hold on
    end 
    
 end
 end
        
        
 %% Recluster  Leach
  EexL(EexL<=0)=0;
  indg3=find(EexL<=0)
  
 if(A==2) 
    [CH]=chselalg(EexL,nodesCH)
 end
  
  %% Store Parameters
    Ec2L(ipp)=sum(EexL);                                                     % Energy Consumption
    REc2L(ipp)=sum(E1) -sum(EexL);                                           % Remaining Energy Level
            
            % Throughput Calc 
            RxData1L=apL;
            
              if(ipp>1)
                  ThroughputL(ipp)=Thfun(ThroughputL,RxData1L,ipp-1);
              else
                  ThroughputL(ipp)=RxData1L;
              end
            
            % AliveNodes Calc
            hold on
            plot(X(indg3),Y(indg3),'o','LineWidth',1,...
                     'MarkerEdgeColor','k',...
                     'MarkerFaceColor','k',...
                     'MarkerSize',8');   
                  xlabel('X in m')
                  ylabel('Y in m')
                  hold on
            AlivenodesL(ipp)=N-numel(indg3);
            %Avg Energy Consumed
            AvgEcL(ipp)=mean(EneExL); 
                  
                  
                  


ipp=ipp+1;
set(handles.text19,'string',num2str(ipp));
pause(roundDelay)
if(stop1==1)
    stop1=0;
    break;
end


end


if(strcmp(Route_type,'direct'))
    Az=Route_type;
elseif(strcmp(Route_type,'chleach'))
    Az=Route_type;
else
    inde=find(Ralg1=='.');
    Az=Ralg1(1:inde-1);
end


fname=['-' Az];
save(['Result\Alivenodes' fname '.mat'],'AlivenodesL')
save(['Result\AvgEc' fname '.mat'],'AvgEcL')
save(['Result\Ec2' fname '.mat'],'Ec2L')
save(['Result\REc2' fname '.mat'],'REc2L')
save(['Result\Throughput',fname,'.mat'],'ThroughputL')

%%
figure,
plot(1:numel(AlivenodesL)-1,AlivenodesL(1:end-1),'-*k')
xlabel('Rounds')
ylabel('AliveNodes')
%legend('Proposed','EEGBR','LEACH','PEAGISS')
title('Alivenodes')

%
figure,
plot(1:numel(AvgEcL)-1,AvgEcL(1:end-1),'-*k')
xlabel('Rounds')
ylabel('AvgEnergyConsumption')
%legend('Proposed','EEGBR','LEACH','PEAGISS')
title('AvgEnergyConsumption')
%%
figure,
plot(1:numel(Ec2L),Ec2L(1:end),'-*k')
xlabel('Rounds')
ylabel('Remaining Energy Level')
legend('Proposed','EEGBR','LEACH','PEAGISS')
title('Remaining Energy Level')
%%
figure,
plot(1:numel(REc2L),REc2L(1:end),'-*k')
xlabel('Rounds')
ylabel('Energy Consumption Level')
%legend('Proposed','EEGBR','LEACH','PEAGISS')
title('Energy Consumption Level')
%%
% Throughput  
figure,
plot(1:numel(ThroughputL)-1,ThroughputL(1:end-1),'-*k')
%legend('Proposed','EEGBR','LEACH','PEAGISS')
xlabel('Rounds') 
ylabel('Recieved-Packets')
title('Recieved-Packets')








% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global stop1

stop1=1;




% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%h3 = findobj(gcf,'tag','pushbutton3') %get handle of the push button
%feval(get(h3,'Callback'),h3,[]);

axes(handles.axes1);
hold off;
cla reset;
%load('InitialData.mat')
global X Y N max1 RaDat

rand('seed',RaDat)
N=str2double(get(handles.edit1,'String'));% Total No. of Nodes
t1=get(handles.edit2,'String');
t2=strsplit(t1,'x');
min1=0;
max1=str2double(t2(2));
X = min1+(max1-min1)*rand(1,N);
Y = min1+(max1-min1)*rand(1,N);



%%
plot(X,Y,'o','LineWidth',1,...
                    'MarkerEdgeColor','k',...
                    'MarkerFaceColor','b',...
                    'MarkerSize',8'); 
          xlabel('X in m')
          ylabel('Y in m')
          
          
for i2 = 1:N 
          
          text(X(i2), Y(i2), num2str(i2),'FontSize',10); 
          hold on;
end
hold on
axes(handles.axes1);          
xlabel('x');
ylabel('y');
guidata(hObject,handles);          
 





function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


fn=dir('Result\*.mat');
for ik=1:numel(fn)
     ind1=find(fn(ik).name=='-');
     ind2=find(fn(ik).name=='.');
     fname2{ik}=fn(ik).name(ind1+1:ind2-1);
end
fname=unique(fname2);
c=['r' 'g' 'b' 'k' 'c' 'y' 'm'];
%%
figure,
for ik=1:numel(fname)
load(['Result\Alivenodes-' fname{ik} '.mat'])
plot(1:numel(AlivenodesL)-1,AlivenodesL(1:numel(AlivenodesL)-1),['-*' c(ik)] )
hold on
end
xlabel('Rounds')
ylabel('AliveNodes')
legend(fname)
title('Alivenodes')

%
figure,
for ik=1:numel(fname)
load(['Result\AvgEc-' fname{ik} '.mat'])
plot(1:numel(AvgEcL)-1,AvgEcL(1:numel(AvgEcL)-1),['-*' c(ik)])
hold on
end
xlabel('Rounds')
ylabel('AvgEnergyConsumption')
legend(fname)
title('AvgEnergyConsumption')
%%
figure,
for ik=1:numel(fname)
load(['Result\Ec2-' fname{ik} '.mat'])    
plot(1:numel(Ec2L),Ec2L,['-*' c(ik)])
hold on
end
xlabel('Rounds')
ylabel('Remaining Energy Level')
legend(fname)
title('Remaining Energy Level')
%%

figure,
for ik=1:numel(fname)
load(['Result\REc2-' fname{ik} '.mat'])    
plot(1:numel(REc2L),REc2L,['-*' c(ik)])
hold on

end
xlabel('Rounds')
ylabel('Energy Consumption Level')
legend(fname)
title('Energy Consumption Level')
%%
% Throughput  
figure,
for ik=1:numel(fname)
load(['Result\Throughput-' fname{ik} '.mat'])
plot(1:numel(ThroughputL)-1,ThroughputL(1:numel(ThroughputL)-1),['-*' c(ik)])
hold on
end
legend(fname)
xlabel('Rounds') 
ylabel('Recieved-Packets')
title('Recieved-Packets')



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox4.
function checkbox4_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox4


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc;
%clear all;
%close all;
warning off all;
restoredefaultpath;
addpath(genpath(pwd));
global X Y N max1 RaDat
axes(handles.axes1);
hold off;
cla reset;

excelsheet=get(handles.edit15,'String');
type=1;
if (get(handles.radiobutton1,'Value') == 0 && get(handles.radiobutton2,'Value') == 0)
    %user did not select any radio button, then do this
  else
    %do this if a user selected a radio button
      switch get(get(handles.uibuttongroup1,'SelectedObject'),'Tag')
      case 'radiobutton1',  type = 1;
      case 'radiobutton2',  type = 2;
      end
end

RaDat=round(100.*rand(1));
rand('seed',RaDat)

if(type==1)
    
    N=str2double(get(handles.edit1,'String'));% Total No. of Nodes
    t1=get(handles.edit2,'String');
    t2=strsplit(t1,'x');
    min1=0;
    max1=str2double(t2(2));
    X = min1+(max1-min1)*rand(1,N);
    Y = min1+(max1-min1)*rand(1,N);

else
    
    [num,txt,raw] = xlsread(excelsheet);
    X=num(:,1);
    Y=num(:,2);
    N=numel(X);  
    
end


%%
plot(X,Y,'o','LineWidth',1,...
                    'MarkerEdgeColor','k',...
                    'MarkerFaceColor','b',...
                    'MarkerSize',8'); 
          xlabel('X in m')
          ylabel('Y in m')
          
          
for i2 = 1:N 
          
          text(X(i2), Y(i2), num2str(i2),'FontSize',10); 
          hold on;
end
hold on
axes(handles.axes1);          
xlabel('x');
ylabel('y');
guidata(hObject,handles);          
 
% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global X Y Xb Yb N max1 SN Rc



Rc=(str2double(get(handles.edit7,'String')));
%Rc=280; %sensor field Radius
t1=get(handles.edit4,'String');
t2=strsplit(t1,'x');
Xb =str2double(t2(1));
Yb =str2double(t2(2));
t1=get(handles.edit3,'String');
t2=strsplit(t1,'x');
gapX=str2double(t2(1));
gapY=str2double(t2(2));
         hold on
         plot(Xb,Yb,'s','LineWidth',1,...
                    'MarkerEdgeColor','k',...
                    'MarkerFaceColor','y',...
                    'MarkerSize',12'); 
         xlabel('X in m')
         ylabel('Y in m')
         text(Xb, Yb, 'Base','FontSize',10); 
         hold on;                    
%% Equal-zone division
zX=0:gapX:max1;
zY=0:gapY:max1;
id=zeros(1,N);          
ik1=1;
for ik=1:numel(zX)-1
    for ij=1:numel(zY)-1
    rectangle('Position',[zX(ik) zY(ij) gapX gapY]);
    points = [zX(ik) zY(ij); zX(ik)+gapX zY(ij);  zX(ik)+gapX zY(ij)+gapY ;zX(ik) zY(ij)+gapY ;zX(ik) zY(ij)];
    %figure(2)
    %plot(points(:,1),points(:,2),'-*r')
    [in,on]=inpolygon(X,Y,points(:,1),points(:,2));
    ind=find(in==1);
    SN(ik1).id=ind;
    ik1=ik1+1;
    hold on
    end
end         
axes(handles.axes1);          
xlabel('x');
ylabel('y');
guidata(hObject,handles);          



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

close all;
closereq;


%
