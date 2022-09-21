[sqrtdeltav,meanv,rmsev] = deal(zeros(1,b.n));

for j=1:b.n
    for i=1:300
        sqrtdeltav(i,j) = ( vCtrl(i,j) - vvCtrl(i,j) )^2;
    end
    meanv(j) = mean(sqrtdeltav(:,j));
    rmsev(j) = sqrt(meanv(j));
end


