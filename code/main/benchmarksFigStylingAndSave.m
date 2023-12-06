function benchmarksFigStylingAndSave(fig, filename)
%     h = findobj(fig, "Type", "axes");
%     set(h, "FontSize", 8);
%     h = findobj(fig, "Type", "text");
%     set(h, "FontSize", 8);

    saveas(fig, filename, "pdf");
    system(sprintf("pdfcrop %s %s", filename, filename));
end