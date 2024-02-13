function benchmarksFigStylingAndSave(fig, filename)

    set(fig, 'Units', 'Inches');
    pos = get(fig,'Position');
    set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    
    print(fig,filename,'-dpdf','-r0')
    set(fig, 'Units', 'pixels');
    return;
end