function saveax(ax, filename, figsize)
    % Saves the axes ax to filename, cropping whitespace
    % ax may be an array, for objects which require it, such as colorbars
    figdummy = figure('Visible','off'); % Invisible figure
    clf;
    figdummy.Position(3) = figsize(3);
    figdummy.Position(4) = figsize(4);
    newAxes = copyobj(ax,figdummy);
    set(newAxes,'Position',get(groot,'DefaultAxesPosition')); 
    set(figdummy,'CreateFcn','set(gcbf,''Visible'',''on'')');
    for ii=1:length(ax)
        newAxes(ii).Position = ax(ii).Position;
    end
    saveas(figdummy, filename);
    system(sprintf("pdfcrop %s %s > NUL", filename, filename));
    delete(figdummy);
end
