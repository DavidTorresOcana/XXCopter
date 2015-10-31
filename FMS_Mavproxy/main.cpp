#include "mavlink_bridge_header.h"
#include "mavlink/common/mavlink.h"
#include <vector>
#include <iostream>
#include <pthread.h>

unsigned int MAX_PARAM=10;

typedef	std::vector<mavlink_param_value_t> Parameters;

static void communication_receive(Parameters &  params)
{
	mavlink_message_t msg;
	mavlink_status_t status;

	unsigned int cnt=0;

	while(cnt<MAX_PARAM)
	{
		uint8_t c;
		RS232_PollComport(comport,&c,1);

		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			std::cout<<"Message: "<<(int)msg.sysid<<" "<<(int)msg.compid<<" "<<(int)msg.msgid<<std::endl;
			switch(msg.msgid)
			{
			case MAVLINK_MSG_ID_HEARTBEAT:
				break;
			case MAVLINK_MSG_ID_PARAM_VALUE:
				mavlink_param_value_t tmp;
				mavlink_msg_param_value_decode(&msg,&tmp);
				params.push_back(tmp);
				cnt++;
				break;
			default:
				break;
			}
		}
	}
}

void * rx_thread_function(void * pv){
	Parameters * ps = (Parameters*)pv;
	communication_receive(*ps);
	pthread_exit((void *) ps);
}

int main(){

	mavlink_system.sysid = 100;
	mavlink_system.compid = 50;

	RS232_OpenComport(comport,57600,"8N1");

	int system_id = 1;
	int component_id = 1;

	Parameters params;
	pthread_t rx_thread;
	pthread_create(&rx_thread,NULL,rx_thread_function,&params);

	mavlink_msg_param_request_list_send(MAVLINK_COMM_0, system_id,component_id);

	void * pv;
	pthread_join(rx_thread,&pv);
	params = *(Parameters *)pv;

	for (unsigned int i=0;i<params.size();++i){
		std::cout<<params[i].param_id<<" "<<params[i].param_value<<std::endl;
	}

	RS232_CloseComport(comport);
}


