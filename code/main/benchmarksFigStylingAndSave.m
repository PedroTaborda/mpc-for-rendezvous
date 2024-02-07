function benchmarksFigStylingAndSave(fig, filename)
%     h = findobj(fig, "Type", "axes");
%     set(h, "FontSize", 8);
%     h = findobj(fig, "Type", "text");
%     set(h, "FontSize", 8);

    saveas(fig, filename, "pdf");    
    % Check if pdfcrop is installed
    [status, result] = system('pdfcrop -h');
    if status == 0
        % Crop the pdf
        system(sprintf("pdfcrop %s %s", filename, filename));
    else
        warning("pdfcrop not found, whitespace not cropped - it is safe to ignore/comment out this warning");
    end
end