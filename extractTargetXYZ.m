function [x,y,z] =extractTargetXYZ(xyz,BW)
working = BW; %input the binary image
[row, column] =  size(working);
index = [];
count=1;
%store the indices corresponding to test object in a
%separate matrix.
for i=1:row
    for j=1:column
        if working(i,j) == 1
            index(end+1) = count;
        end
        count=count+1;
    end
end

%% create a new XYZ matrix with only those indices that 
%correspond to the test object.
x = [];
y = [];
z = [];
for i= 1:size(index,2)
    var = index(i);
    if ~(isnan(xyz(var,1)) || isnan(xyz(var,2)) ||
        isnan(xyz(var,3)))
        x(end+1)= xyz(var,1);
        y(end+1)= xyz(var,2);
        z(end+1)= xyz(var,3);
    end
end
end
