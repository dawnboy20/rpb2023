clear all; close all; clc

X=load("data.txt");

figure(1);
plot(X(:,1),X(:,2),'.');
title('Randomly Generated Data');

%K-mean clustering
opts = statset('Display','final');
[idx,C] = kmeans(X,2,'Distance','cityblock',...
    'options',opts);

% Plot
figure(2);
plot(X(idx==1,1),X(idx==1,2),'r.','MarkerSize',12)
hold on
plot(X(idx==2,1),X(idx==2,2),'b.','MarkerSize',12)
plot(C(:,1),C(:,2),'kx',...
    'MarkerSize',15,'LineWidth',3)
legend('Cluster 1','Cluster 2','Centroids',...
    'Location','NW')
title('Cluster Assignments and Centroids')
hold off

