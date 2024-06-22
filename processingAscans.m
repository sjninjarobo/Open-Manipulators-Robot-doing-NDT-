%% get the hilber transform and log compression for each A-Scan
hilbertOut=[];
rescaledGray = [];
for j=1:size(dataAScan,1)
    hilbertOut(j,:) = hilbert(dataAScan(j,1:800));
    hilbertAbs = abs(hilbertOut);
    logCompressed = log(hilbertAbs);
    Max = max(logCompressed,[],'all');
    Min = min(logCompressed,[],'all');
    %save the data in grayscale format for better
    %visualization
    rescaledGray = mat2gray(logCompressed, [0 Max]); 
end
%% creating volumetric data
xyzSurface = [xB yB zB];
normals = [-u -v -w];
%% create new volumetric points by using normal information 
% and corresponding 
allPoints=[];
intensitiesGray=[];
for i=1:size(rescaledGray,1)
    allPoints((end+1),:)=xyzSurface(i,:);
    AlineInt = rescaledGray(i,:);
    intensitiesGray(end+1)=AlineInt(1);
    normal1 = normals(i,:);
    for k=1:699
        allPoints((end+1),:)=xyzSurface(i,:) + normal1*k*0.000192;
        %From time scale of A-Scan to distance scale:
        % Distance = (speed of sound)*time_least_count_per_data_point
        %             = 6000*3.2e-08
        %             =0.000192 m
        intensitiesGray(end+1)=AlineInt(k);
    end
    disp(i)
end
intensitiesGray= intensitiesGray';
%% visualize
ptcloud = pointCloud(allPoints,'Intensity',intensitiesGray);
pcshow(ptcloud)
hold on
plot3(xB, yB, zB,'.','MarkerSize',15)
colorbar 
hold off
