function ScriptForFigToPdfFun(nf,filename,pdfYes)
h = figure(nf);
% plot(1:10);
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
if pdfYes ==1
    print(h,filename,'-painters','-dpdf','-r0')
    %     export_fig(h,filename,'Render','-painters','-pdf')
end
print(h,filename,'-painters','-depsc','-r0')
% export_fig(h,filename,'Render','-painters','-eps')
