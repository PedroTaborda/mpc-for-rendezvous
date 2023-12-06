function validationFigStylingAndSave(fig, filename)
    % set line thickness for all lines already drawn
    h = findobj(fig, "Type", "line");
    set(h, "LineWidth", 1.5);

    set(fig, "DefaultAxesFontSize", 8);
    set(fig, "DefaultTextFontSize", 8);

    h = findobj(fig, "Type", "axes");
    set(h, "FontSize", 8);
    h = findobj(fig, "Type", "text");
    set(h, "FontSize", 8);

    saveas(fig, filename, "pdf");
    system(sprintf("pdfcrop %s %s", filename, filename));
end