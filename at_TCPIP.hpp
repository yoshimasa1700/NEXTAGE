#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef u_int SOCKET;
typedef struct in_addr IN_ADDR;
typedef struct in_addr *LPIN_ADDR;
typedef struct hostent *LPHOSTENT;
typedef struct sockaddr *LPSOCKADDR;
typedef struct sockaddr_in SOCKADDR_IN;
#define Sleep(micro) usleep(micro*1000)
#define INVALID_SOCKET (SOCKET)(~0)
#define SOCKET_ERROR (-1)
#define closesocket close
#define FALSE false
#define TRUE true

class at_TCPIP
{
public:
	bool cnct;
private:
	SOCKET s;	
public :
	bool TCPConnect(char *server,u_short port)
	{
		cnct=false;
		s = socket(PF_INET,SOCK_STREAM,0);
		if(s==INVALID_SOCKET) return false;
		LPHOSTENT ipHost = gethostbyname(server);
		if(ipHost == NULL) return false;
		SOCKADDR_IN sockadd;
		memset(&sockadd,0,sizeof(sockadd));
		sockadd.sin_family = AF_INET;
		sockadd.sin_port = htons(port);
		sockadd.sin_addr = *((LPIN_ADDR)*ipHost->h_addr_list);
		if(connect(s,(sockaddr*)&sockadd,sizeof(sockadd))!=0) return false;
		cnct=true;
		return true;
	}
	bool TCPSend(char* sendWord)
	{
		int nRtn;
		int i;
		if(!cnct)return FALSE;
		//ポインタで渡された送信文字列はsizeofが使えないため
		//文字数をカウントします
		for(i=0;sendWord[i]!='\0';i++);
		nRtn = send(s,sendWord,i,0);//送信
		if(nRtn==SOCKET_ERROR || nRtn==0) return FALSE; else return TRUE;
	}
	int TCPRead(char*buff,int bfSize)
	{
		// データを受信するまでブロック
		{
			fd_set readfds;
			FD_ZERO( &readfds ) ;
			FD_SET( s , &readfds );

			struct timeval timeout;
			timeout.tv_sec =  2;  // 待ち時間
			timeout.tv_usec = 0;

			// select Function
			// ソケットの状態を設定する。ここでは読取り用のソケットに対してタイムアウトを指定する。
			int n = select( 
				0,             // 未使用っぽい
				&readfds,      // ソケットが読取り可能かをチェックするようにする
				NULL,
				NULL,
				&timeout       // タイムアウト時間
				);
			if( n == SOCKET_ERROR )
			{
				return -1;
			}
			// タイムアウト発生
			if( n == 0 )
			{
				return -1;
			}
		}

		int nRtn=false;
		nRtn = recv(s,buff,bfSize,0);//受信
		if(nRtn==SOCKET_ERROR || nRtn==0) return -1;
		else{
			buff[nRtn]='\0';
			return nRtn;
		}
	}
	bool TCPRead2(unsigned char*buff,int width,int height)
	{
		int i;
		int cnt=0;
		for(i=0;i<height;i++){
			while(cnt-i*width<width){
				if(int ccnt = recv(s,(char*)buff+cnt, width,0)>0){
					cnt+=ccnt;
				}
				Sleep(10);
			}
		}
		if(cnt>0){
			buff[cnt]='\n';
			return TRUE;
		}
		else
			return FALSE;
	}
	void TCPClose()
	{
		closesocket(s);
	}
};
