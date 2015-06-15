function V2I()
    clc;
    clear all;
    workspace
    grid on;
    title('V2I Connectivity')
    xlabel('X')
    ylabel('Y')
    axis([0 1000 0 1000]);
    uiwait(msgbox('Click on two points for Road 1.', 'modal'));
    [x1, y1] = ginput(2);
    line([x1(1), x1(2)], [y1(1), y1(2)])
    uiwait(msgbox('Click on next point for Road 2.', 'modal'));
    [x2, y2] = ginput(1);
    line([x1(2), x2], [y1(2), y2])
    uiwait(msgbox('Click on next point for Road 3.', 'modal'));
    [x3, y3] = ginput(1);
    line([x2, x3], [y2, y3])

    hold on;    
    %scatter(x_1,y_1); 

    a1 = [x1(1), y1(1)];
    b1 = [x1(2), y1(2)];
    len = pdist ([x1(1), y1(1); x1(2), y1(2)],'euclidean');
    % straight line function from a to b
    func = @(xa)a1(2) + (a1(2)-b1(2))/(a1(1)-b1(1))*(xa-a1(1));
    % determine the x values
    xa = linspace(a1(1),b1(1),len/2);
    % determine the y values
    ya = func(xa);

    a2 = [x1(2), y1(2)];
    b2 = [x2, y2];
    len = pdist ([x1(2), y1(2); x2, y2],'euclidean');
    % straight line function from a to b
    func = @(xb)a2(2) + (a2(2)-b2(2))/(a2(1)-b2(1))*(xb-a2(1));
    % determine the x values
    xb = linspace(a2(1),b2(1),len/2);
    % determine the y values
    yb = func(xb);

    a3 = [x2, y2];
    b3 = [x3, y3];
    len = pdist ([x2, y2; x3, y3],'euclidean');
    % straight line function from a to b
    func = @(xc)a3(2) + (a3(2)-b3(2))/(a3(1)-b3(1))*(xc-a3(1));
    % determine the x values
    xc = linspace(a3(1),b3(1),len/2);
    % determine the y values
    yc = func(xc);

    uiwait(msgbox('Click on a point for RSU 1.', 'modal'));
    [rsu1x, rsu1y] = ginput(1);
    scatter(rsu1x, rsu1y, 'filled');
    text(rsu1x+20, rsu1y-10, 'RSU1');

    uiwait(msgbox('Click on a point for RSU 2.', 'modal'));
    [rsu2x, rsu2y] = ginput(1);
    scatter(rsu2x, rsu2y, 'filled');
    text(rsu2x-40, rsu2y-30, 'RSU2');

    % get a handle to a plot graphics object
    hPlot1 = plot(NaN,NaN, 'square');
    speed=10+(10*rand);
    pause_time=speed*0.001;

    for k=1:length(xa)
        % update the plot graphics object with the next position
        set(hPlot1,'XData',xa(k),'YData',ya(k));
        vrsu1 = pdist ([xa(k), ya(k); rsu1x, rsu1y],'euclidean');
        vrsu2 = pdist ([xa(k), ya(k); rsu2x, rsu2y],'euclidean');
        if((vrsu1 <= 100)&&(vrsu2 > 100))
            one = plot([xa(k), rsu1x],[ya(k), rsu1y], '--g');
            scatter(xa(k),ya(k),'g', 'filled');
        elseif((vrsu2 <= 100)&&(vrsu1 > 100))
            two = plot([xa(k), rsu2x],[ya(k), rsu2y], '--g');
            scatter(xa(k),ya(k),'g', 'filled');
        elseif((vrsu2 <= 100)&&(vrsu1 <= 100)) % If both are less than 100, pick the closer RSU
            if(vrsu1<=vrsu2)
                three = plot([xa(k), rsu1x],[ya(k), rsu1y], '--g');
                scatter(xa(k),ya(k),'g', 'filled');
            else
                three = plot([xa(k), rsu2x],[ya(k), rsu2y], '--g');
                scatter(xa(k),ya(k),'g', 'filled');
            end
        end
        if((vrsu1 >100)&&(vrsu2 >100))      % if neither is less than 100m, display red 
            scatter(xa(k),ya(k),'r', 'filled');
        end
        pause(pause_time);
        if exist('one', 'var')
            delete(one)
        end
        if exist('two', 'var')
            delete(two)
        end
        if exist('three', 'var')
            delete(three)
        end
    end

    for k=1:length(xb)
        % update the plot graphics object with the next position
        set(hPlot1,'XData',xb(k),'YData',yb(k));
        vrsu1 = pdist ([xb(k), yb(k); rsu1x, rsu1y],'euclidean');
        vrsu2 = pdist ([xb(k), yb(k); rsu2x, rsu2y],'euclidean');
        if(vrsu1 <= 100)
            one = plot([xb(k), rsu1x],[yb(k), rsu1y], '--g');
            scatter(xb(k),yb(k),'g', 'filled');
        elseif(vrsu2 <= 100)
            two = plot([xb(k), rsu2x],[yb(k), rsu2y], '--g');
            scatter(xb(k),yb(k),'g', 'filled');
        elseif((vrsu2 <= 100)&&(vrsu1 <= 100))      % If both are less than 100, pick the closer RSU
            if(vrsu1<=vrsu2)
                three = plot([xb(k), rsu1x],[yb(k), rsu1y], '--g');
                scatter(xb(k),yb(k),'g', 'filled');
            else
                three = plot([xb(k), rsu2x],[yb(k), rsu2y], '--g');
                scatter(xb(k),yb(k),'g', 'filled');
            end
        end
        if((vrsu1 > 100)&&(vrsu2 > 100))        % if neither is less than 100m, display red 
            scatter(xb(k),yb(k),'r', 'filled');
        end
        pause(pause_time);
        if exist('one', 'var')
            delete(one)
        end
        if exist('two', 'var')
            delete(two)
        end
        if exist('three', 'var')
            delete(three)
        end
    end

    for k=1:length(xc)
        % update the plot graphics object with the next position
        set(hPlot1,'XData',xc(k),'YData',yc(k));
        vrsu1 = pdist ([xc(k), yc(k); rsu1x, rsu1y],'euclidean');
        vrsu2 = pdist ([xc(k), yc(k); rsu2x, rsu2y],'euclidean');
        if(vrsu1 <=100)
            one = plot([xc(k), rsu1x],[yc(k), rsu1y], '--g');
            scatter(xc(k),yc(k),'g', 'filled');
        elseif(vrsu2 <=100)
            two = plot([xc(k), rsu2x],[yc(k), rsu2y], '--g');
            scatter(xc(k),yc(k),'g', 'filled');
        elseif((vrsu2 <= 100)&&(vrsu1 <= 100))      % If both are less than 100, pick the closer RSU
            if(vrsu1<=vrsu2)
                three = plot([xc(k), rsu1x],[yc(k), rsu1y], '--g');
                scatter(xc(k),yc(k),'g', 'filled');
            else
                three = plot([xc(k), rsu2x],[yc(k), rsu2y], '--g');
                scatter(xc(k),yc(k),'g', 'filled');
            end
        end
        if((vrsu1 >100)&&(vrsu2 >100))      % if neither is less than 100m, display red 
            scatter(xc(k),yc(k),'r', 'filled');
        end
        pause(pause_time);
        if exist('one', 'var')
            delete(one)
        end
        if exist('two', 'var')
            delete(two)
        end
        if exist('three', 'var')
            delete(three)
        end
    end
end