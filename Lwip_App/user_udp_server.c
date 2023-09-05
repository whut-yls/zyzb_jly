#include "api.h"
#include "user_udp_server.h"

void StartUdpServerTask(void const *arg)
{
	struct netconn* conn;
	struct netbuf *buf,*tx_buf;
	static void *data;
	unsigned short int len;
	err_t err,recv_err;
	
	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_UDP);
	if (conn!=NULL)
	{
		/* Bind connection to well known port number 7. */
		err = netconn_bind(conn, NULL, 5002);
		if (err == ERR_OK)
		{
			while (1)
			{
					if(( recv_err = netconn_recv(conn, &buf)) == ERR_OK)
					{
						do
						{
							netbuf_data(buf, &data, &len);
							 err = netconn_sendto(conn, buf, (const ip_addr_t *)&(buf->addr), buf->port);
								if(err != ERR_OK) {
									LWIP_DEBUGF(LWIP_DBG_ON, ("netconn_send failed: %d\n", (int)err));
								} else {
									LWIP_DEBUGF(LWIP_DBG_ON, ("got %s\n", buffer));
								}
							
						}while (netbuf_next(buf) >= 0);
						netbuf_delete(buf);
					}
			}	
		}
		else
		{
			netconn_delete(conn);
		}
	}	
}
