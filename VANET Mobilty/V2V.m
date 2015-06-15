function V2V()

    clc;
    clear all;
    workspace
    grid on;
    title('V2V Connectivity')
    xlabel('X')
    ylabel('Y')
    axis([0 1000 0 1000]);
    uiwait(msgbox('Click on two points for Road 1.', 'modal'));
    [x1, y1] = ginput(2);
    line([x1(1), x1(2)], [y1(1), y1(2)])
    slope1 = (y1(2)-y1(1))/(x1(2)-x1(1))
    uiwait(msgbox('Click on two points for Road 2.', 'modal'));
    [x2, y2] = ginput(2);
    line([x2(1), x2(2)], [y2(1), y2(2)])
    p1 = polyfit(x1, y1, 1);
    p2 = polyfit(x2, y2, 1);
    x_intersect = fzero(@(x) polyval(p1-p2,x),3);
    y_intersect = polyval(p1,x_intersect);

    hold on;  
    a1 = [x1(1), y1(1)];
    b1 = [x1(2), y1(2)];
    % straight line function from a1 to b1
    func = @(xa)a1(2) + (a1(2)-b1(2))/(a1(1)-b1(1))*(xa-a1(1));
    % determine the x values
    speed = randi([10 20], 1, 1);
    no_of_points = floor(abs(((x1(1) - x1(2))/speed)));
    % divide the line based on random speed
    xa = linspace(a1(1),b1(1),no_of_points);
    % determine the y values
    ya = func(xa);

    a2 = [x2(1), y2(1)];
    b2 = [x2(2), y2(2)];
    % straight line function from a2 to b2
    func = @(xb)a2(2) + (a2(2)-b2(2))/(a2(1)-b2(1))*(xb-a2(1));
    % determine the x values
    speed = randi([10 20], 1, 1);
    no_of_points = floor(abs(((x2(1) - x2(2))/speed)));
    % divide the line based on random speed
    xb = linspace(a2(1),b2(1),no_of_points);
    % determine the y values
    yb = func(xb);

    a3 = [x2(2), y2(2)];
    b3 = [x2(1), y2(1)];
    % straight line function from a to b
    func = @(xc)a3(2) + (a3(2)-b3(2))/(a3(1)-b3(1))*(xc-a3(1));
    % determine the x values
    speed = randi([10 20], 1, 1);
    no_of_points = floor(abs(((x2(1) - x2(2))/speed)));
    xc = linspace(a3(1),b3(1),no_of_points);
    % determine the y values
    yc = func(xc);

    % get a handle to a plot graphics object
    hPlot1 = plot(NaN,NaN, 'square');
    hPlot2 = plot(NaN,NaN, 'square');
    hPlot3 = plot(NaN,NaN, 'square');
    % iterate through each point on line

    for k=1:min(min(length(xa),length(xb)),min(length(xa),length(xc)))
        % update the plot graphics object with the next position
        if (k <= length(xa))
            set(hPlot1,'XData',xa(k),'YData',ya(k));
        end
        if (k <= length(xb))
            set(hPlot2,'XData',xb(k),'YData',yb(k));
        end
        if (k <= length(xc))
            set(hPlot3,'XData',xc(k),'YData',yc(k));
        end
        ab = pdist ([xa(k), ya(k);xb(k), yb(k)],'euclidean');
        bc = pdist ([xb(k), yb(k);xc(k), yc(k)],'euclidean');
        ac = pdist ([xa(k), ya(k);xc(k), yc(k)],'euclidean');
        if(ab <=100)
            one = plot ([xa(k), xb(k)], [ya(k), yb(k)], '--g');
        end
        if(bc <=100)
            two = plot ([xb(k), xc(k)], [yb(k), yc(k)], '--g');
        end
        if(ac <=100)
            three = plot ([xa(k), xc(k)], [ya(k), yc(k)], '--g');
        end
        % pause for 0.3 second
        pause(0.3);
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
end