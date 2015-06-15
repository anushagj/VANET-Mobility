function VANET_mobility
% Parameters for the simulations
% Safety distance (Ds) = 10 meters
% Vehicle speed Smin = 50 miles/hour = 22.35 meters/second, Smax = 70
% miles/hour = 31.29 meters/second
% Vehicle acceletation = a_min= 0 meters/seconds2, a_max = +(-) 5 meters/seconds2
% Road Traffic Volume = vol = 3000 vehicles/hour/lane
% Vehicle Arrival rate/Departure rate = 0.833
% Road Traffic Density = 100-500 vehicles/lane

minutes=2;            %no of minutes
range=50;            %comm range

global Smax; 
global Smin; 
global amax; 
global amin;
global veh_id;
global ytotal;
global target;

Smin = 22.35;
Smax = 31.29;
amin = 0;
amax = 5;
traffic_density = 100:50:500;
ytotal = 5000;
arrival_rate = 0.833;
departure_rate = 0.833;

x_parta=[];
x_partb=[];
x_partc=[];
sim_conn=0;
tot_same3=0;
tot_same=0;

Ds = 10;
b=0.75;
r=0.0070104;
sdep=(-b+sqrt(b^2+(4*r*Ds)))/(2*r);    %used in car following model

% increments of 50 traffic density
% Assuming same traffic density in all lanes
% Giving initial coordinates to nodes based on uniform distribution


for m=traffic_density
    
    %fix your target node using veh_id
    target=randi([1 4*m],1,1)
    
    %empty lanes for each density
    lane1=[];
    lane2=[];
    lane3=[];
    lane4=[];
    
    for runs=1:1:5
        
        % Initializing the lanes 
        lane1= initialize(m,ytotal,1);
        lane2= initialize(m,ytotal,2);
        lane3= initialize(m,ytotal,3);
        lane4= initialize(m,ytotal,4);
        
        % Intitializing number of neighbors
        num_neigh=0;
        
        % Iterations
        % Iterations with 100ms steps for 10 minutes 10*60 / 0.1 = 6000
        % Outer_loop iterations=600; inner_loop=each 100
        
        for t=0:1:minutes*60
            
            % Entry ramp once every sec
            if rand<arrival_rate     
                random_entry_ramp=randi([1 3],1,1);     %randomly selects one integer out of 1 to 3
                random_entry_lane=randi([1 4],1,1);
                sp=Smin + (Smax - Smin)*rand(1);      %random speed for new node entering
                acc=amin + (amax - amin)*rand(1);     %random speed for new node entering
                veh_id=veh_id+1;                      %if a new node enters,it gets the next id
                switch random_entry_lane
                    case 1
                        %insertnode takes random speed,acceleration,ramp,lane
                        lane1=insertnode(sp,acc,lane1,random_entry_ramp);        
                    case 2
                        lane2=insertnode(sp,acc,lane2,random_entry_ramp);
                    case 3
                        lane3=insertnode(sp,acc,lane3,random_entry_ramp);
                    case 4
                        lane4=insertnode(sp,acc,lane4,random_entry_ramp);
                    end
            end
            
            %exit ramp once every second
            if rand<departure_rate     
                random_exit_ramp=randi([1 3],1,1);
                random_exit_lane=randi([1 4],1,1);
                %delete node function takes ramp and lane no as arguments
                switch random_exit_lane
                    case 1
                        lane1=deletenode(lane1,random_exit_ramp);
                    case 2
                        lane2=deletenode(lane2,random_exit_ramp);
                    case 3
                        lane3=deletenode(lane3,random_exit_ramp);
                    case 4
                        lane4=deletenode(lane4,random_exit_ramp);
                end
            end
            
            for time=0:0.1:1
                
                lane=[];   %a temporary array used in lane changing model
                
                % Border effect and position update
                lane1=border(lane1);
                lane2=border(lane2);
                lane3=border(lane3);
                lane4=border(lane4);
                
                % Freeway and car following mobility
                lane1=freeway(lane1, sdep, Ds);
                lane2=freeway(lane2, sdep, Ds);
                lane3=freeway(lane3, sdep, Ds);
                lane4=freeway(lane4, sdep, Ds);
                
                %lane changing model
                %coulnt use a common function as lane2 and lane3 have two adjacent lanes
                %lane1
                i=1; 
                while(i<size(lane1,1))
                    if((lane1(i+1,1)-lane1(i,1))<10)      %only change lane if leading node is closer than 10 metres 
                    x = lane2(:,1)<lane1(i,1);            %select nodes on nxt lane less than considered node
                    [val,id]=max(lane2(x));               %select the lowest nearest node on next lane
                    if(isempty(id)==1)                    %if appending is to be done at start of next lane
                        val=lane2(1,1);
                            id=1;                          %node at start of lane
                        if((lane2(id,1)-lane1(i,1))>10)       %change lane only if difference is Ds
                            lane2=vertcat(lane1(i,:),lane2(id:end,:));   %move the vehicle to nxt lane 
                            lane1(i,:)=[];
                            i=i-1;                            %decrement id whenever node leaves the lane
                        end
                    elseif(id==size(lane2,1) && ((lane1(i,1)-val)>10))   %if appending is to be done at end of next lane
                        lane2=vertcat(lane2(1:end,:),lane1(i,:));        %move the vehicle to nxt lane 
                        lane1(i,:)=[];
                        i=i-1;
                    else
                        if(((lane1(i,1)-val)>10)&&((lane2(id+1,1)-lane1(i,1))>10))       %check if no vehicle within +/- 10 range
                            lane2=vertcat(lane2(1:id,:),lane1(i,:),lane2(id+1:end,:));   %move the vehicle to nxt lane 
                            lane1(i,:)=[];
                            i=i-1; 
                        end
                    end
                    i=i+1;                            %increment id to run the loop
                    else
                        i=i+1;                        %increment id if no changes are required
                    end
                end


                %lane2
                i=1; 
                while(i<size(lane2,1))
                    if((lane2(i+1,1)-lane2(i,1))<10)       %only change lane if leading node is closer than 10 metres
                    l_ch=round(rand(1));            % coin flip and choose adj lane
                    if l_ch==0                      
                        lane=lane1;                 % used a single variable var lane for code optmisation 
                    else
                        lane=lane3;
                    end  
                    x = lane(:,1)<lane2(i,1);       %select nodes on nxt lane less than considered node
                    [val,id]=max(lane(x));          %select the nearest node on next lane
                    if(isempty(id)==1)              %if appending is to be done at start of next lane
                        val=lane(1,1);
                            id=1;                   %node at start of lane
                        if((lane(id,1)-lane2(i,1))>=10)                %change lane only if difference is Ds
                            lane=vertcat(lane2(i,:),lane(id:end,:));   %move the vehicle to nxt lane 
                            lane2(i,:)=[];
                            i=i-1;                  %decrement id whenever node leaves the lane
                        end
                    elseif(id==size(lane,1) && ((lane2(i,1)-val)>10))     %nearest node is at the end of other lane
                        lane=vertcat(lane(1:end,:),lane2(i,:));           %move the vehicle to nxt lane 
                        lane2(i,:)=[];
                        i=i-1;
                    else
                        if(((lane2(i,1)-val)>10)&&((lane(id+1,1)-lane2(i,1))>10))    %check if no vehicle within +/- 10 range
                            lane=vertcat(lane(1:id,:),lane2(i,:),lane(id+1:end,:));  %move the vehicle to nxt lane 
                            lane2(i,:)=[];
                            i=i-1; 
                        end
                    end
                    i=i+1;                                      %increment id to run the loop
                    if l_ch==0                                   %as changes are made in a single variable 
                        lane1=lane;                              %assigning them back
                    else
                        lane3=lane;
                    end
                    else
                        i=i+1;                                  %increment id if no changes are required
                    end
                end


                %lane3
                i=1; 
                while(i<size(lane3,1))
                    if((lane3(i+1,1)-lane3(i,1))<10)        %only change lane if leading node is closer than 10 metres
                    l_ch=round(rand(1));            % coin flip and choose adj lane
                    if l_ch==0                      
                        lane=lane2;                 % used a single variable var lane for code optmisation 
                    else
                        lane=lane4;
                    end 
                    x = lane(:,1)<lane3(i,1);            %select nodes on nxt lane less than considered node
                    [val,id]=max(lane(x));               %select the nearest node on next lane
                    if(isempty(id)==1)                    %if appending is to be done at start of next lane
                        val=lane(1,1);
                            id=1;                         %node at start of lane
                        if((lane(id,1)-lane3(i,1))>10)              %nearest node is at the end of other lane
                            lane=vertcat(lane3(i,:),lane(id:end,:));   %move the vehicle to nxt lane 
                            lane3(i,:)=[];
                            i=i-1;                  %decrement id whenever node leaves the lane
                        end
                    elseif(id==size(lane,1) && ((lane3(i,1)-val)>10))       %nearest node is at the end of other lane
                        lane=vertcat(lane(1:end,:),lane3(i,:));   %move the vehicle to nxt lane 
                        lane3(i,:)=[];
                        i=i-1;
                    else
                        if(((lane3(i,1)-val)>10)&&((lane(id+1,1)-lane3(i,1))>10))   %check if no vehicle within +/- 10 range
                            lane=vertcat(lane(1:id,:),lane3(i,:),lane(id+1:end,:));   %move the vehicle to nxt lane 
                            lane3(i,:)=[];
                            i=i-1; 
                        end
                    end
                    i=i+1;                                       %increment id to run the loop
                    if l_ch==0                                   %as changes are made in a single variable 
                        lane2=lane;                              %assigning them back
                    else
                        lane4=lane;
                    end
                    else
                        i=i+1;                                 %increment id if no changes are required
                    end
                end


                %lane4
                i=1; 
                while(i<size(lane4,1))
                    if((lane4(i+1,1)-lane4(i,1))<10)       %only change lane if leading node is closer than 10 metres
                    x = lane3(:,1)<lane4(i,1);            %select nodes on nxt lane less than considered node
                    [val,id]=max(lane3(x));               %select the nearest node on next lane
                    if(isempty(id)==1)                    %if appending is to be done at start of next lane
                        val=lane3(1,1);
                            id=1;
                        if((lane3(id,1)-lane4(i,1))>10)
                            lane3=vertcat(lane4(i,:),lane3(id:end,:));   %move the vehicle to nxt lane 
                            lane4(i,:)=[];
                            i=i-1;                           %decrement id whenever node leaves the lane
                        end
                    elseif(id==size(lane3,1) && ((lane4(i,1)-val)>10))     %nearest node is at the end of other lane
                        lane3=vertcat(lane3(1:end,:),lane4(i,:));   %move the vehicle to nxt lane 
                        lane4(i,:)=[];
                        i=i-1;
                    else
                        if(((lane4(i,1)-val)>10)&&((lane3(id+1,1)-lane4(i,1))>10))   %check if no vehicle within +/- 10 range
                            lane3=vertcat(lane3(1:id,:),lane4(i,:),lane3(id+1:end,:));   %move the vehicle to nxt lane 
                            lane4(i,:)=[];
                            i=i-1; 
                        end
                    end
                    i=i+1;                                  %increment id to run the loop
                    else
                        i=i+1;                              %increment id if no changes are required
                    end
                end 
                
                %connectivity of target node
                %check for target id in each lane
                if(any(lane1(:,4)==target))     
                    xpos=0;           %used in no of neighbors calculation
                    lanex=lane1;      %using same array for code optimisation
                elseif(any(lane2(:,4)==target))
                   xpos=3;             %used in no of neighbors calculation  
                   lanex=lane2;
                elseif(any(lane3(:,4)==target))
                   xpos=6;             %used in no of neighbors calculation
                   lanex=lane3;
                elseif(any(lane4(:,4)==target))
                   xpos=9;             %used in no of neighbors calculation
                   lanex=lane4;
                else
                    disp('target not found')
                end
                id=find(lanex(:,4)==target);    %location of target node
                ypos=lanex(id,1);               %ycoord of target node
                
                %no of neighbors on each lane irrespective of what lane target is on
                ids1=lane1(find(sqrt((lane1(:,1)-ypos).^2+(0-xpos).^2)<=range),4);     
                ids2=lane2(find(sqrt((lane2(:,1)-ypos).^2+(3-xpos).^2)<=range),4);
                ids3=lane3(find(sqrt((lane3(:,1)-ypos).^2+(6-xpos).^2)<=range),4);
                ids4=lane4(find(sqrt((lane4(:,1)-ypos).^2+(9-xpos).^2)<=range),4);
                
                %total neighbors
                neigh=[ids1;ids2;ids3;ids4]';
                neigh(find(neigh==target))=[];    %as per the formula used target is also a neighbor... so deleting it
                num_neigh=num_neigh+length(neigh);    %total no of neighbors
                
                if(t==0 && time==0)              %start of each simulation
                    set3=[];
                    f=0;
                    g=[];
                    k=[];
                    same3=[];
                    time_cnt=0;
                    flag=0;
                    set_30=[];
                end
                
                %part b
                set3=union(neigh,set3);       %this array has all neighbors every iteration
                if(length(intersect(set3,neigh))>=3)    %see if it has same 3 neighbors
                    time_cnt=time_cnt+0.1;          
                    flag=1;                     
                elseif(length(intersect(set3,neigh))<3)   
                    if(flag==1)                          %check flag to avoid appending zeroes to array
                        same3=[same3,time_cnt];          % mean of this array is taken at end of each sim
                        time_cnt=0;
                        flag=0;
                    end
                end

                %part c   30 secs is selected
                f=f+0.1;                     %this variable becomes 30 secs after 300 iterations
                set_30=union(neigh,set_30) ;     %set_30 has list of neighbors for 30 secs
                l=length(intersect(set_30,neigh));      %l has no of same neighbors every 100msec
                if(f<=30)
                    if(l>=1)
                        k=[k,l];              %after 30 secs mean of k is appended to g array
                    end
                else
                    f=0;
                    g=[g,mean(k)];            %g has mean neighbors for different 30 secs
                    k=[];
                    set_30=[];
                end 
            end
        end
        if(time_cnt~=0)
            same3=[same3,time_cnt];             %at end of each sim, if any value in time_cnt
        end
        sim_conn=sim_conn+round(num_neigh/(minutes*60*10));     %average calculated once every 10 mins i.,e per each sim 
        tot_same3=tot_same3+mean(same3);
        tot_same=tot_same+mean(g);
        set3=[];
    end
    
    %after every sim,the values are appended for plotting
    v2v_conn=round(sim_conn/runs);          %average calculated for each traffice density i.e., evry 5 sim
    x_parta=[x_parta,v2v_conn];
    avg_same3=round(tot_same3/runs);
    x_partb=[x_partb,avg_same3];         %total is in secs
    avg_same=round(tot_same/runs);
    x_partc=[x_partc,avg_same];
    
    sim_conn=0;                     %assigning temp variables back to 0 at end of each sim
    tot_same3=0;
    tot_same=0;
    
end


figure(1);
plot(traffic_density,x_parta)
title('average v2v connectivity')
ylabel('v2v connectivity') % x-axis label
xlabel('traffic density') % y-axis label
legend('comm range 50 m','Location','northwest')
legend('comm range 50 m','Location','northwest')

figure(2);
plot(traffic_density,x_partb)
title('average duration of same 3 neighbors')
ylabel('average duration in secs') % x-axis label
xlabel('traffic density') % y-axis label


figure(3);
plot(traffic_density,x_partc)
title('average number of same neighbors for 30 sec')
ylabel('average number') % x-axis label
xlabel('traffic density') % y-axis label
legend('comm range 50 m','Location','northwest')

end

% Functions

% Initializing the nodes
%intialize function takes in traffic density, 5000 metres and lane no and
%returns uniformly allocated lane
function lane = initialize(m,ytotal,lane_no)   
global Smax; 
global Smin; 
global amax; 
global amin;
global veh_id;
lane=zeros(m,4);
lane(:,1) = ytotal*rand(m,1);  
lane=sortrows(lane);
lane(:,2)=Smin + (Smax - Smin)*rand(m,1);    %100 in a lane have uniformly distributed speed
lane(:,3)=amin + (amax - amin)*rand(m,1);
for i=1:m   
    lane(i,4)=(lane_no-1)*m + i;
end
veh_id=lane_no*m;                   %veh_id has total no of nodes after intialization
end

% Function to insert node at entry ramps
%function insertnode takes random speed,accelartion,ramp and lane num as
%arguments and inserts a node on that lane
function l=insertnode(s,a,lane,ramp)
    global veh_id
    entry_ramps = [1510,2510,4010];
    Y=entry_ramps(ramp);
    X = lane(:,1)<Y;                     %select nodes less than entry ramp coords
    [Val,Id]=max(lane(X));               %select the nearest one of the above
    if(isempty(Id)==1)
        l=vertcat([Y,s,a,veh_id],lane(1:end,:)); 
    elseif(Id==size(lane,1))
        l=vertcat(lane(1:end,:),[Y,s,a,veh_id]); 
    else
        l=vertcat(lane(1:Id,:),[Y,s,a,veh_id],lane(Id+1:end,:));       %add our node just after the nearest node
    end
end

% Function to remove node from exit ramp
%function deletenode takes random ramp and lane num as
%arguments and sees if there a node; if yes deletes it
function lane=deletenode(lane,ramp)
exit_ramps = [1500,2500,4000];
    global target
    Y=exit_ramps(ramp);  
    x=lane(:,1)==Y;
     if(any(x))                   %select nodes less than exit ramp coords
        id=find(x);              %select the nearest one of the above
        if(lane(id,4)~=target)      %see that target node is not deleted
            lane(id,:)=[]; 
        end  
     end
end  

% Function for border effect
function lane = border(lane)
global ytotal;
%position update
lane(:,1)=lane(:,1)+lane(:,2)*(0.1)+(0.5*lane(:,3)*0.01);  %s=vt+1/2at2
x = lane(:,1)>5000;            %select nodes which crossed 5km
if(any(x))                     %if any nodes >5km
    ids=find(x);                %find ids on lane to delete them and store their ids before deleting
    temp=lane(ids,:);           %store their speed,acceleration and ids
    lane(ids,:)=[];     
    for i=1:size(ids,1)
        npos=ytotal*rand(1);           %new random position
        y = lane(:,1)<npos;            %select nodes on nxt lane less than new generated position
        [val,id]=max(lane(y)); 
        if(isempty(id)==1)                    %if needs to be appended at start of lane
            lane=vertcat([npos,temp(i,2:end)],lane(1:end,:));    %insert into lane randomly 
        elseif(id==size(lane,1))                %if needs to be appended at end of lane
            lane=vertcat(lane(1:end,:),[npos,temp(i,2:end)]);    %insert into lane randomly
        else
            lane=vertcat(lane(1:id,:),[npos,temp(i,2:end)],lane(id+1:end,:));  %insert into lane randomly
        end
    end
end
end

% Function for Freeway Mobility Model
function lane=freeway(lane, sdep, Ds)
lane(:,2)=lane(:,2)+(-1+2*rand(1))*lane(:,3);       %velocity dependance on its previous value
len=size(lane,2);
for i=1:len-1
    if(lane(i+1)-lane(i)<=Ds)                  %if within safety distance
        if(lane(i+1,2)>lane(i,2))               %check velocities of trailing and leading
            lane(i,2)=min(lane(i+1,2),sdep);    %if car within Ds, see that speed is less than its prev
        end
    end   
end
end