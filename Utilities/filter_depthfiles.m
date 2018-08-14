sigmar = 100;
wins = 2;
sigmas = 5;
filteriter = 1;
metric = 1000.0;

baseDir = '/home/cvlab/floorplan/reconstructions_filtered/mmcr/depth/';
baseDirNew = '/home/cvlab/floorplan/reconstructions_filtered/mmcr/depth_filtered/';
files = dir(fullfile(baseDir, '*.png'));
num_files = size(files);

for idx = 1 : num_files(1);
    D = imread([baseDir, files(idx).name]);
   % D = calc_depth(double(D));
    D = filterdepth(D, sigmar, wins, sigmas, filteriter, metric);
    Dint = uint16(round(D));
    imwrite(Dint,[baseDirNew, files(idx).name,])
end

    
%D = imread("/home/cvlab/floorplan/reconstructions/mmcr/depth/Depth0000.png")