
%%
files = ["C:\Users\assad\Documents\recording_files\DS1\DS1_20250930\DS1_20250930.mat",
    "C:\Users\assad\Documents\recording_files\DS1\DS1_20250927\DS1_20250927.mat",
    "C:\Users\assad\Documents\recording_files\DS1\DS1_20250926\DS1_20250926.mat",
    "C:\Users\assad\Documents\recording_files\DS1\DS1_20250925\DS1_20250925.mat",
    "C:\Users\assad\Documents\recording_files\DS1\DS1_20250924\DS1_20250924.mat"];
%%
files = ["C:\Users\assad\Documents\recording_files\DS1\DS1_20250913\DS1_20250913.mat"]
%files = ["C:\Users\assad\Documents\recording_files\DS1\DS1_20251001\DS1_20251001.mat"];
allilis = [];
for i = 1:length(files)
    test = load(files(i));
    eventmarkers = test.obj.EventMarkers;
    lickmarker = 9;
  
    licks = eventmarkers(:,1) == lickmarker;
    licktimes = eventmarkers(licks, 2);
    start = licktimes(1);
    licktimes = licktimes - start;
    
    licktimes = licktimes / 600;
    
    ilis = diff(licktimes);
    ilis = ilis(ilis<35);
    ilis = ilis(ilis>1);
    allilis = [allilis; ilis];
end

allilis = round(allilis, 2);
figure();
histogram(allilis, 'NumBins',30)

figure();
logili = log(allilis);
histogram(logili, 'NumBins',30)
meanlogili = exp(mean(logili));
sdlogili = std(logili); 

modeili = mode(allilis);
medili = median(allilis);
meanili = mean(allilis);
