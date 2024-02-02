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
    % Check if pdfcrop is installed
    [status, result] = system('pdfcrop -h');
    if status == 0
        % Crop the pdf
        system(sprintf("pdfcrop %s %s", filename, filename));
    else
        warning("pdfcrop not found, whitespace not cropped - it is safe to ignore/comment out this warning");
    end
    delete(figdummy);
end
