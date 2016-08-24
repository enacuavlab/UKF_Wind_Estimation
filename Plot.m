
%AOA vs. AOA est
uvw=squeeze(uvw)';
AOAest= atan (uvw(:,3)./uvw(:,1));
plot(aoa(:,2));
hold on
plot(AOAest);

