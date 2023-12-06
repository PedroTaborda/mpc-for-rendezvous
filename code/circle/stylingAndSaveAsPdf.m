function stylingAndSaveAsPdf(fig, filename)
    % set latex interpreter for most text objects
    h = findobj(fig, "Type", "axes");
    set(h, "TickLabelInterpreter", "latex");
    %set(h, "XLabelInterpreter", "latex");
    %set(h, "YLabelInterpreter", "latex");
    %set(h, "ZLabelInterpreter", "latex");
    %set(h, "TitleInterpreter", "latex");

    %h = findobj(fig, "Type", "legend");
    %set(h, "Interpreter", "latex");

    % set line thickness for all lines already drawn
    h = findobj(fig, "Type", "line");
    set(h, "LineWidth", 1.5);

    set(fig, "DefaultAxesFontSize", 12);
    set(fig, "DefaultTextFontSize", 12);
    set(fig, "DefaultAxesFontName", "Arial");

    saveas(fig, filename, "pdf");
    system(sprintf("pdfcrop %s %s", filename, filename));
end