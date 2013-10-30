#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "at_TCPIP.hpp"

#define Sleep(micro) usleep(micro*1000)

#define LIMNUM 21 // 関節数

class at_Nextage
{
private:
	at_TCPIP *s;
	char strAgls[65536];
public:
	bool cnct;
	struct LIMBS
        {
            //int ID;    // 名前
            //int lim;   // 関節
            double agl;// 角度
            double ini_agl;// 角度
            double th_agl;// 角速度スレッショルド
            //int dir;   // 回転方向
            //double x;  // 座標
            //double y;  // 座標
            //double z;  // 座標
        }limb[LIMNUM];
public:
	at_Nextage(char *server,u_short port){
		s = new at_TCPIP();
		sprintf(strAgls,"allAgl");
		for (int i = 0; i < LIMNUM; i++){
			limb[i].agl = 0;
			limb[i].th_agl = 0.02;
			//limb[i].dir = 1;
		}
		for (int i = 0; i < LIMNUM; i++){
			limb[i].ini_agl = 0;
			//limb[i].th_agl = 0.02;
			//limb[i].dir = 1;
		}
		limb[ 0].ini_agl = 0;// 腰ヨー
		limb[ 1].ini_agl = 0;// 頭部ヨー
		limb[ 2].ini_agl = 0;// 頭部ピッチ
		limb[ 3].ini_agl = 0;// 右肩ヨー
		limb[ 4].ini_agl = 0;// 右肩ピッチ
		limb[ 5].ini_agl =-130;// 右肘ピッチ？
		limb[ 6].ini_agl = 15;// 右肘捻り
		limb[ 7].ini_agl = 0;// 右手首ピッチ
		limb[ 8].ini_agl = 0;// 右手首エンドエフェクタ
		limb[ 9].ini_agl = 0;// 左肩ヨー
		limb[10].ini_agl = 0;// 左肩ピッチ
		limb[11].ini_agl =-130;// 左肘ピッチ？
		limb[12].ini_agl =-15;// 左肘捻り
		limb[13].ini_agl = 0;// 左手首ピッチ
		limb[14].ini_agl = 0;// 左手首エンドエフェクタ
		/*
		limb[ 1].th_agl = 2.0;// 頭部ヨー
		limb[ 2].th_agl = 2.0;// 頭部ピッチ
		limb[ 3].th_agl = 2.0;// 右肩ヨー
		limb[ 4].th_agl = 2.0;// 右肩ピッチ
		limb[ 5].th_agl = 2.0;// 右肘ピッチ？
		limb[ 9].th_agl = 2.0;// 左肩ヨー
		limb[10].th_agl = 2.0;// 左肩ピッチ
		limb[11].th_agl = 2.0;// 左肘ピッチ？*/
		cnct=s->TCPConnect(server,port);
	}
	bool calib(){
		if(!cnct)return false;
		s->TCPSend((char*)"cal_off\n");
		Sleep(100);
		s->TCPSend((char*)"pow_off\n");
		Sleep(100);
		s->TCPSend((char*)"pow_on\n");
		Sleep(500);
		s->TCPSend((char*)"cal_on\n");
		for(int i=0;i<10;i++)Sleep(1000);
		s->TCPSend((char*)"cal_off\n");
		Sleep(1000);
		return true;
	}
	bool init(){
		if(!cnct)return false;
		s->TCPSend((char*)"cmd_all_off\n");
		Sleep(100);
		s->TCPSend((char*)"pow_off\n");
		Sleep(100);
		s->TCPSend((char*)"pow_on\n");
		Sleep(1000);
		s->TCPSend((char*)"cmd_all_on\n");
		Sleep(1000);
		return true;
	}
	bool sendAgl(int id,double agl){
		char str[128];
		limb[id].agl=fmin(fmax(agl,limb[id].agl-limb[id].th_agl),limb[id].agl+limb[id].th_agl);
		sprintf(str,"agl %d %1.2f\n",id,limb[id].agl+limb[id].ini_agl);
		s->TCPSend(str);
		Sleep(15);
		if(id==5||id==11)
		printf(str,"a %d %1.2f\n",id,limb[id].agl+limb[id].ini_agl);
		if(fabs(limb[id].agl-agl)>limb[id].th_agl)return false;
		return true;
	}
	bool addAgl(int id,double agl){
		limb[id].agl=fmin(fmax(agl,limb[id].agl-limb[id].th_agl),limb[id].agl+limb[id].th_agl);
		double rad_agl=(limb[id].agl+limb[id].ini_agl)*3.14/180.0;
//printf("目標\n%d\t%lf\n",id,limb[id].agl);
		sprintf(strAgls,"%s %1.6f",strAgls,rad_agl);
		if(limb[id].agl!=agl)return false;
		return true;
	}
	bool sendAllAgl(bool flg_now){//flg_nowのときは即実行、それ以外はstartAngle時に実行
		sprintf(strAgls,"%s\n",strAgls);
		if(!flg_now){strAgls[0]='a';strAgls[1]='d';strAgls[2]='d';}
		s->TCPSend(strAgls);
		sprintf(strAgls,"allAgl");
		return true;
	}
	bool startAnime(){
		s->TCPSend("startAnime\n\0");
		return true;
	}
};
