function plot_trim()
% Recreate report figures from saved simulation data without rerunning it.
root = fileparts(mfilename('fullpath'));
out = root;
D = load(fullfile(out,'trim.mat'),'t','O','result');
names = D.result.OutputNames;
units = D.result.OutputUnits;
fig = figure('Visible','off','Color','w','Position',[80 80 1250 860]);
guard = onCleanup(@() close(fig));
tiledlayout(3,2,'TileSpacing','compact');
selected = [1,2,3,5,6,4];
for j=1:6
    k=selected(j);
    ax=nexttile;
    plot(ax,D.t,D.O(:,k),'Color',[0 0.35 0.7],'LineWidth',1.5);
    hold(ax,'on');
    yline(ax,D.result.InitialOutputs(k),'--','Initial',...
        'Color',[0.35 0.35 0.35],'LabelHorizontalAlignment','right');
    set(ax,'Color','w','XColor','k','YColor','k','GridColor',[0.6 0.6 0.6],...
        'FontSize',10);
    grid(ax,'on');
    xlabel(ax,'Time (s)','Color','k');
    ylabel(ax,sprintf('%s (%s)',names(k),units(k)),'Color','k');
end
sgtitle('Group 8: 60 s nonlinear flight, fixed initial controls, zero gust','Color','k');
exportgraphics(fig,fullfile(out,'trim.png'),'Resolution',180,'BackgroundColor','white');
exportgraphics(fig,fullfile(out,'trim.pdf'),'ContentType','vector','BackgroundColor','white');
end
