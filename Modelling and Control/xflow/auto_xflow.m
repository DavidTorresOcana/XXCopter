% Script de automatizaci�n de proceso de extracci�n


%% 1. Obtenci�n de lista de ficheros en carpeta /data
    file_list_struct = get_xflow_files;

%% 2. Extracci�n de datos

% Bruteforce para casos con simetr�a
    axis_matrix = [[-1 0 0];[0 -1 0];[0 0 1]]; % Matriz cambio de base
    sim_flag = 0;
    time_proc = 0.4;

for i = 1:size(file_list_struct,1)
    xflow_data(i) = get_xflow_data(file_list_struct(i).name,axis_matrix,sim_flag,time_proc)
end


%% 3. Al excel o a un array !! TBD


