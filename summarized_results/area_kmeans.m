close all;
cluster = 3;

X = [6957.15,5936.84;
    7199.13,5452.66;
    6932.05,5372.42;
    7896.51,6290.57;
    8748.83,5935.08;
    5701.82,5490.37;
    5839.99,5536.96;
    5987.15,5592.09;
    5534.22,5088.62;
    6054.24,5113.84];

stream = RandStream('mlfg6331_64');  % Random number stream
options = statset('UseParallel',1,'UseSubstreams',1,'Streams',stream);
tic; % Start stopwatch timer
[idx,C,sumd,D] = kmeans(X,cluster,'Options',options,'MaxIter',10000,'Display','final','Replicates',10);

x1 = min(X(:,1)):10:max(X(:,1));
x2 = min(X(:,2)):10:max(X(:,2));
[x1G,x2G] = meshgrid(x1,x2);
XGrid = [x1G(:),x2G(:)]; % Defines a fine grid on the plot

idx2Region = kmeans(XGrid,cluster,'MaxIter',1,'Start',C);

figure
gscatter(XGrid(:,1),XGrid(:,2),idx2Region,[0,0.75,0.75;0.75,0,0.75;0.75,0.75,0],'..');
hold on;


gscatter(X(:,1),X(:,2),idx,'bgm')
hold on
plot(C(:,1),C(:,2),'kx')
% legend('Cluster 1','Cluster 2','Cluster 3','Cluster Centroid');
