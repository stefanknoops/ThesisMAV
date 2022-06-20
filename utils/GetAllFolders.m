    
function folders = GetAllFolders()
    d = dir('../Experiments/');
    isub = [d(:).isdir]; %# returns logical vector
    nameFolds = {d(isub).name}';
    nameFolds(ismember(nameFolds,{'.','..','__pycache__'})) = [];
    
    
    folders = strings(size(nameFolds,1),1);
    for i=1:size(nameFolds)
        folders(i) = string(nameFolds(i));
    end
    
    folders.T;