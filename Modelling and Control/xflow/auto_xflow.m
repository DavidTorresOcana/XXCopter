% Script de automatización de proceso de extracción


%% 1. Obtención de lista de ficheros en carpeta /data

    addpath(pwd);
    dir_list = get_xflow_dirs;
    size_dir_list = size(dir_list,2);
    counter_data = 1;
    cd data


%% 2. Extracción de datos
    for i = 1:size_dir_list
        cd (dir_list{i})
        file_list_struct = get_xflow_files;

        % Bruteforce para casos con simetría
        axis_matrix = [[-1 0 0];[0 -1 0];[0 0 1]]; % Matriz cambio de base
        sim_flag = 0;
        time_proc = 0.4;

        for j = 1:size(file_list_struct,1)
            xflow_data(counter_data) = get_xflow_data(file_list_struct(j).name,axis_matrix,sim_flag,time_proc);
            counter_data = counter_data+1;
        end
        
        cd ..

    end


%% 3. Al excel o a un array !! TBD


