function Set_theta_comm(num)

Block_search=find_system(['Tricopter_Controlled/GCS params'], 'Name', 'theta_comm' );

set_param(char(Block_search),'Value',num2str(num));

end