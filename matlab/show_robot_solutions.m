iiwa = loadrobot("kukaIiwa14");
config = homeConfiguration(iiwa);

for j=1:4
    subplot(2,2,j);
    for i=1:7
        config(i).JointPosition = solutionSet(j,i);
    end
    ax = show(iiwa,config);
    ax.XLim = [-0.5, 0.5];
    ax.YLim = [-0.5, 1.0];
    ax.ZLim = [0, 1.0];
end
