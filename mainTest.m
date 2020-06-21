clc;
clear all;
close all;


rng default;
stop1=0;


N=100; % No. of nodes
t1='400x400';
t2=strsplit(t1,'x');

min1=0;
max1=str2double(t2(2));
X = min1+(max1-min1)*rand(1,N);
Y = min1+(max1-min1)*rand(1,N);



%%
figure,
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
xlabel('x');
ylabel('y');

 



Rc=200; % coverage Area



%Rc=280; %sensor field Radius
% position of  sink node
t1='410x410';
t2=strsplit(t1,'x');
Xb =str2double(t2(1));
Yb =str2double(t2(2));
t1='50x50';
t2=strsplit(t1,'x');
%t2=[50 50]; % zone size
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
xlabel('x');
ylabel('y');




%%
alpha=0.01;%0.001 energy per distance% 0.001                                 %% node to CH power Ratio
beta=0.045;%0.0015                                                           %% CH to sink distance power ratio

Sector1=1;
nodes=N;% Total No. of Nodes
E=10.*ones(1,nodes);
nodesCH=2;% no of cluster head
EexL=E;
E1=E;
%% Ch selection
chalg1='chselalg.m'
chalg=strrep(chalg1,'.m','');
varName=matlab.lang.makeValidName(chalg);
chselalg=str2func(varName)
[CH]=chselalg(EexL,nodesCH)

%% Routing
RoutingId=1;
    %do this if a user selected a radio button
      switch RoutingId
      case 1,  Route_type = 'direct';
      case 2,  Route_type = 'chleach';
      case 3,  Route_type = 'custom';    
      end


if(strcmp(Route_type,'custom'))
    Ralg1='DjisktraRoute.m';
    Ralg=strrep(Ralg1,'.m','');
    varName=matlab.lang.makeValidName(Ralg);
    Ralgfun=str2func(varName)    
    
end

%% EnergyFunction
Efun1='Energyfun.m'
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
Thfun1='Throughputfun.m';
Thfun=strrep(Thfun1,'.m','');
varName=matlab.lang.makeValidName(Thfun);
Thfun=str2func(varName)
%% Simulation Starts
round=200;
roundDelay=0.05;
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
%set(handles.text19,'string',num2str(ipp));
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

title('Alivenodes')

%
figure,
plot(1:numel(AvgEcL)-1,AvgEcL(1:end-1),'-*k')
xlabel('Rounds')
ylabel('AvgEnergyConsumption')

title('AvgEnergyConsumption')
%%
figure,
plot(1:numel(Ec2L),Ec2L(1:end),'-*k')
xlabel('Rounds')
ylabel('Remaining Energy Level')
title('Remaining Energy Level')
%%
figure,
plot(1:numel(REc2L),REc2L(1:end),'-*k')
xlabel('Rounds')
ylabel('Energy Consumption Level')

title('Energy Consumption Level')
%%
% Throughput  
figure,
plot(1:numel(ThroughputL)-1,ThroughputL(1:end-1),'-*k')

xlabel('Rounds') 
ylabel('Recieved-Packets')
title('Recieved-Packets')










