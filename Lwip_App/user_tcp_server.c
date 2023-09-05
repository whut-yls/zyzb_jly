#include "api.h"
#include "string.h"
#include "user_tcp_server.h"
static unsigned char recvNetData1[RECV_BUF_MAX-1];
static int recvNetLen1;
struct netconn* gNetServerSock;
/**
功能：从一个打开的套接字句柄上发送数据
返回：成功返回实际写入的字节数 失败返回-1
**/
static int transport1_sendPacketBuffer(struct netconn* newconn,unsigned char* send,short int sendLen)
{
	unsigned int w_len;
	
	if(newconn==NULL)
	{
		return -1;
	}
	/*发送一帧数据**/			
	if(ERR_OK==netconn_write_partly(newconn, send, sendLen,NETCONN_COPY, &w_len))
	{
		return w_len;
	}else{
		return -1;
	}
}

void StartTcpServerTask(void const *arg)
{
	struct netconn* conn;
	struct netconn* newconn;
	struct netbuf *buf;
	static void *data;
	unsigned short int readLen;
	err_t err,recv_err;
	err_t accept_err;
	
	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);
	if (conn!=NULL)
	{
		/* Bind connection to well known port number 7. */
		err = netconn_bind(conn, NULL, 5002);
		if (err == ERR_OK)
		{
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);
			while (1)
			{
					recvNetLen1=0;
				/* Grab new connection. */
				accept_err = netconn_accept(conn, &newconn);
				newconn->recv_timeout=5000;
				/* Process the new connection. */
				if (accept_err == ERR_OK)
				{
					while (( recv_err = netconn_recv(newconn, &buf)) == ERR_OK)
					{
						do
						{
							netbuf_data(buf, &data, &readLen);
							/**拷贝数据到recv,更新recvLen**/
							memcpy(recvNetData1,data,readLen);
							recvNetLen1+=readLen;
							if(recvNetLen1>RECV_BUF_MAX)
							{
							break;
							}
							/**会写测试**/
//							netconn_write(newconn, data, len, NETCONN_COPY);
						}while (netbuf_next(buf) >= 0);
						netbuf_delete(buf);
						/*数据处理*/
						transport1_sendPacketBuffer(newconn,recvNetData1,recvNetLen1);
					}
					/* Close connection and discard connection identifier. */
					netconn_close(newconn);
					netconn_delete(newconn);
				}else{ //其它错误
					/* Close connection and discard connection identifier. */
					netconn_close(newconn);
					netconn_delete(newconn);
				}
			}	
		}
		else
		{
			netconn_delete(newconn);
		}
	}
}
