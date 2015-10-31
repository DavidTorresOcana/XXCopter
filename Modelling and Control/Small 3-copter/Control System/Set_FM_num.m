function Set_FM_num(num)

Block_search=find_system(['Tricopter_Controlled/GCS params'], 'Name', 'FM_source' );

set_param(char(Block_search),'Value',num2str(num));

end