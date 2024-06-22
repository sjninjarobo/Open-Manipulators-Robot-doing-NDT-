function BW = extractTarget(image2)
L = superpixels(image2,500);
imshow(image2)
f = drawpolygon(gca,'Color','g'); % create foreground mask
foreground = createMask(f,image2);
b1 = drawpolygon(gca,'Color','r'); % create 4 background mask
b2 = drawpolygon(gca,'Color','b');
b3 = drawpolygon(gca,'Color','m');
b4 = drawpolygon(gca,'Color','k');
background = createMask(b1,image2) + createMask(b2,image2) + 
createMask(b3,image2) +createMask(b4,image2);
BW = lazysnapping(image2,L,foreground,background);
end
