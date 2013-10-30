#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <math.h>

#include "at_Nextage20130401.hpp"
#include <time.h>

/*豊吉くん
#include "/opt/softkinetic/DepthSenseSDK/include/DepthSense.hxx"
#include <opencv2/opencv.hpp>
#include <HFMD_core/CRForest.h>
#include <HFMD_core/util.h>
#include <HFMD_core/CDataset.h>
#include <gflags/gflags.h>
#include "./CCalibDS325.h"
*/



using namespace OpenRAVE;
using namespace std;

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define usleep(micro) Sleep(micro/1000)
#endif

#define NEXTAGEIP "150.82.201.200" //NEXTAGE側のIPアドレス

/*---------------------------NEXHAND用---------------------------*/
/*概要：サーボ番号
 *中央部分根元	1
 *中央部分中間	2
 *中央部分指先	3
 *右指部分根元	4
 *右指部分中間	5
 *右指部分指先	6
 *左指部分根元	7
 *左指部分中間	8
 *左指部分指先	9
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define COM_PORT		"/dev/ttyUSB0"		// 通信ポートの指定
#define BAUDRATE B115200
#define SPEED 100
#define HANDdof 9	//ハンドの軸数
/*NEXHAND初期姿勢の違いを埋める値*/
#define HAND1 0
#define HAND2 -900
#define HAND3 0
#define HAND4 0
#define HAND5 -900
#define HAND6 0
#define HAND7 0
#define HAND8 -900
#define HAND9 0
/*NEXHAND各id*/
#define HAND1id 15
#define HAND2id 16
#define HAND3id 16
#define HAND4id 17
#define HAND5id 18
#define HAND6id 18
#define HAND7id 19
#define HAND8id 20
#define HAND9id 20

struct termios oldtio, newtio; /* 通信ポートを制御するためのインターフェイス */

/*----------------------------------------------------------------------------*/
/*	概要：通信ポートを閉じる
 *
 *	関数：	int CommClose( int fd )
 *	引数：
 *		int fd		通信ポートのハンドル
 *	戻り値：
 *		1				成功
 */
int CommClose( int fd )
{
	//for linux
	tcsetattr(fd, TCSANOW, &oldtio);  /* 退避させた設定に戻す */
	close(fd);                       /* COM1のシリアルポートを閉じる */
	return 1;
}

int CommOpen( char *pport )
{
	int fd;		// 通信用ハンドル
	//int baudRate = 115200;	
	// 通信ポートを開く
	if((fd=open(COM_PORT, O_RDWR | O_NOCTTY))== -1)
	{
		/* O_RDWR:読み書き両用 O_NOCTTY:tty制御をしない */
		perror(COM_PORT);
		/*exit(-1);*/
		printf("HAND connect Err\nHand does not move.\n");
		return fd;
	}
	tcgetattr(fd, &oldtio);          /* 現在のシリアルポートの設定を退避させる */
	bzero(&newtio, sizeof(newtio));  /* 新しいポートの設定の構造体をクリア */
	newtio.c_cflag= (BAUDRATE | CS8 | CLOCAL | CREAD);
	/* CRTSCTS:フロー制御有り CS8:8ビット、ノンパリティ、ストップビット１
	CLOCAL:モデムの状態信号を無視 CREAD:受信可能にする */
	newtio.c_iflag=IGNPAR;          /* IGNPAR:パリティエラーの文字は無視 */
	newtio.c_oflag=0;               /* rawモード */
	newtio.c_lflag=0;               /* 非カノニカル入力 */
	newtio.c_cc[VTIME]=0;           /* キャラクタ間タイマは未使用 */
	newtio.c_cc[VMIN]=1;            /* MC文字受け取るまでブロックする */
	tcflush(fd,TCIFLUSH);           /* ポートのクリア */
	tcsetattr(fd, TCSANOW, &newtio); /* ポートの設定を有効にする */
	// 通信バッファクリア
	tcflush(fd, TCIOFLUSH);
FuncEnd:
	return fd;
}
/*----------------------------------------------------------------------------*/
/*	概要：サーボを移動させる
 *
 *	関数：int RSMove( int fd, short sPos, unsigned short sTime )
 *	引数：
 *		int			fd		通信ポートのハンドル
 *		short			sPos		移動位置
 *		unsigned short	sTime		移動時間
 *	戻り値：
 *		0以上			成功
#include <stdio.h>					// 標準ヘッダー
 *		0未満			エラー
 */
int RSMove( int fd, short sPos, unsigned short sTime, int sid )
{
	unsigned char	sendbuf[28];
	unsigned char	sum;
	int				i;
	int				rets;
	//unsigned long	len;
	// ハンドルチェック
	if( !fd ){return -1;}
	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));
	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;			// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;			// ヘッダー2
	sendbuf[2]  = (unsigned char)sid;			// サーボID
	sendbuf[3]  = (unsigned char)0x00;			// フラグ
	sendbuf[4]  = (unsigned char)0x1E;			// アドレス(0x1E=30)
	sendbuf[5]  = (unsigned char)0x04;			// 長さ(4byte)
	sendbuf[6]  = (unsigned char)0x01;			// 個数
	sendbuf[7]  = (unsigned char)(sPos&0x00FF);		// 位置
	sendbuf[8]  = (unsigned char)((sPos&0xFF00)>>8);	// 位置
	sendbuf[9]  = (unsigned char)(sTime&0x00FF);		// 時間
	sendbuf[10] = (unsigned char)((sTime&0xFF00)>>8);	// 時間
	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < 11; i++ ){sum = (unsigned char)(sum ^ sendbuf[i]);}
	sendbuf[11] = sum;								// チェックサム
	// 通信バッファクリア
	//PurgeComm( fd, PURGE_RXCLEAR );
	tcflush( fd, TCIOFLUSH );
	// 送信
	//rets = WriteFile( fd, &sendbuf, 12, &len, NULL );
	rets = write( fd, &sendbuf, 12);
	return rets;
}
/*----------------------------------------------------------------------------*/
/*	概要：サーボのコンプライアンスを指定する
 *
 *	関数：int RSSetComp( int fd,unsigned short compCW,unsigned short compCCW, int sid )
 *  compCW:時計回り方向の硬さ（０から２５５、大きいほど柔らかい）
 *  compCCW:反時計回り方向の硬さ（０から２５５、大きいほど柔らかい）
 *  sid サーボID（１から２５５、255の時は全サーボに摘要される）
 */
int RSSetComp( int fd, unsigned short compCW,unsigned short compCCW, int sid )
{
	unsigned char	sendbuf[28];
	unsigned char	sum=0;
	int i;
	int	ret;
	// ハンドルチェック
	if( !fd ){return -1;}
	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));
	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;			// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;			// ヘッダー2
	sendbuf[2]  = (unsigned char)sid;			// サーボID
	sendbuf[3]  = (unsigned char)0x00;			// フラグ
	sendbuf[4]  = (unsigned char)0x18;			// アドレス(0x1E=30)
	sendbuf[5]  = (unsigned char)0x04;			// 長さ(4byte)
	sendbuf[6]  = (unsigned char)0x01;			// 個数
	sendbuf[7]  = (unsigned char)(0x05);			// コンプライアンスマージン
	sendbuf[8]  = (unsigned char)(0x05);			// コンプライアンスマージン
	sendbuf[9]  = compCW;					// コンプライアンススロープ
	sendbuf[10] = compCCW;					// コンプライアンススロープ
	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < 11; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[11] = sum;								// チェックサム
	// 通信バッファクリア
	tcflush( fd, TCIOFLUSH );
	// 送信
	ret = write( fd, &sendbuf, 12);
	return ret;
}
/*----------------------------------------------------------------------------*/
/*	概要：サーボのトルクをON/OFFする
 *
 *	関数：int RSTorqueOnOff( int fd, short sMode )
 *	引数：
 *		int			fd		通信ポートのハンドル
 *		short		sMode		1:トルクON
 *										0:トルクOFF
 *	戻り値：
 *		0以上			成功
 *		0未満			エラー
 */
int RSTorqueOnOff( int fd, short sMode, int sid )
{
	unsigned char	sendbuf[28];
	unsigned char	sum;
	int		i;
	int		rets;
	// ハンドルチェック
	if( !fd ){return -1;}
	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));
	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;		// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;		// ヘッダー2
	sendbuf[2]  = (unsigned char)sid;		// サーボID
	sendbuf[3]  = (unsigned char)0x00;		// フラグ
	sendbuf[4]  = (unsigned char)0x24;		// アドレス(0x24=36)
	sendbuf[5]  = (unsigned char)0x01;		// 長さ(4byte)
	sendbuf[6]  = (unsigned char)0x01;		// 個数
	sendbuf[7]  = (unsigned char)(sMode&0x00FF);	// ON/OFFフラグ
	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < 8; i++ ){sum = (unsigned char)(sum ^ sendbuf[i]);}
	sendbuf[8] = sum;								// チェックサム
	// 通信バッファクリア
	tcflush( fd, TCIOFLUSH );
	// 送信
	rets = write( fd, &sendbuf, 9);
	return rets;
}
/*----------------------------------------------------------------------------*/
/*	概要：制御用main
 *
 *	関数：int main( )
 *	引数：	なし
 *	戻り値：
 *		0				エラー
 *		0でない値		成功
 */
void moveFinger( int fd, int deg ,int i)
{
			RSMove( fd, deg, SPEED, i);
}
/*---------------------------NEXHAND用-------------------ここまで*/
void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer=RaveCreateViewer(penv,viewername);
    penv->AddViewer(viewer);
    viewer->main(true);
}
int main(int argc, char ** argv)
{
/*---------------------------NEXHAND用準備---------------------------*/
	int	fd = 0;		// 通信用ハンドル
	int	rets = 0;	// リターン用
	int	c;		// 入力待ち用
	// 正常終了時値の設定
	rets = 1;
	// 通信ポートを開く
	printf( "COM PORT OPEN [%s]\n", COM_PORT );
	fd = CommOpen( COM_PORT );
	printf( "fd=%d\n",fd );
	if( !fd )
	{
		printf( "ERROR:Com port open error\n" );
		printf( "COM PORT CLOSE [%s]\n", COM_PORT );
	  CommClose( fd );;
	}
	if( fd!=-1 )
	{//サーボのトルクON
		printf( "SEND Torque ON\n" );
		for( int i = 1; i <= HANDdof; i++ )
		{
			rets = RSTorqueOnOff( fd, 1, i );
			if( rets < 0 )
			{
				printf( "ERROR:Torque ON failed[%x]\n", rets );
				printf( "Hit Return Key\n" );
				c = getchar( );
			}	
		}
	}
/*---------------------------NEXHAND用準備-------------------ここまで*/
	clock_t start,end;//時間確認用
	FILE *fpp;
	double tensou=0.00;
////////////////////////////////////////////////
	int m,n,Tdof,Tgrasp=0,Tpres;
	char lBuf[512], *p;
	FILE *fp;
  char *ret;
	start = clock();//データ読み込み時
	//ファイルオープン
	if((fp=fopen("grasp_NEXTAGE_ORNAMENT_0904.dat","r"))==NULL)
	{
		printf("Can't open\n");
		exit(EXIT_FAILURE);
	}
	//把持数をファイルより確認
	while(fgets(lBuf,sizeof(lBuf),fp)!=NULL)
	{
		if((ret = strstr(lBuf,"totalgrasp:")) !=NULL)
		{
			ret= strchr( ret, ':' );
			ret++;
			sscanf( ret, " %d", &Tgrasp);
		}
  }
	if(Tgrasp==0){printf( "totalgrasp:は発見出来ません\n");}
	rewind(fp);
	//１把持あたりの配列数をファイルより確認
	fgets( lBuf, sizeof( lBuf ), fp );/*一行だけ取り出す*/
	if(sscanf(lBuf, " %d %d", &Tdof,&Tpres) != EOF)
	{
////		printf("totaldof:%d\n",Tdof);
	}

	//indices用配列作成
	int igraspdir[3],
			igrasppos[3],
			igrasproll,
			igraspstandoff,
			igrasppreshape[Tpres],
			igrasptrans[12],
			imanipulatordirection[3],
			forceclosure,
			grasptrans_nocol[12],
			performance,
			GDistance;

 //その分だけ配列を生成+各把持の最後に比較用の値(把持値と定義)を用意
	double data[Tgrasp][Tdof+1];

	fgets( lBuf, sizeof( lBuf ), fp );
	if((p= strchr( lBuf, ' ' )) == NULL)//間の空白を利用して値を取り出す
	{
		fgets( lBuf, sizeof( lBuf ), fp );
		p = strchr( lBuf, ' ' );
	}
	
	for( m=0; m < Tgrasp; m++ )
	{
		for( n=0; n < Tdof; n++ )
		{
			// データの取り込み
			sscanf( p, "  %lf", &data[m][n] );
			p=p+sizeof(&data[m][n]);//取り出した値だけ行を進める

			if((p= strchr( p, '.' )) == NULL)//次の行に行くかの確認
			{
			 	fgets( lBuf, sizeof( lBuf ), fp );
				p = strchr( lBuf, '.' );
			}
			p=p-2;
		}
	p = strchr( lBuf, '[' );//新しい把持の値の前にある[を無視するため
	p=p+1;
	}
	if((ret = strstr(lBuf,"igraspdir")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		for( m=0; m < 3; m++ )
		{
			sscanf( ret, " %d", &igraspdir[m]);
			ret= strchr( ret, ',' );
			ret++;
		}
  }
  else{printf( "igraspdirは発見出来ません\n");}

	if((ret = strstr(lBuf,"igrasppos")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		for( m=0; m < 3; m++ )
		{
			sscanf( ret, " %d", &igrasppos[m]);
			ret= strchr( ret, ',' );
			ret++;
		}
  }
  else{printf( "igrasprollは発見出来ません\n");}

	if((ret = strstr(lBuf,"igrasproll")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		sscanf( ret, " %d", &igrasproll);
  }
  else{printf( "igrasprollは発見出来ません\n");}

	if((ret = strstr(lBuf,"igraspstandoff")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		sscanf( ret, " %d", &igraspstandoff);
  }
  else{printf( "igraspstandoffは発見出来ません\n");}

	if((ret = strstr(lBuf,"igrasppreshape")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		for( m=0; m < Tpres; m++ )
		{
			sscanf( ret, " %d", &igrasppreshape[m]);
			ret= strchr( ret, ',' );
			ret++;
		}
  }
  else{printf( "igrasppreshapeは発見出来ません\n");}

	if((ret = strstr(lBuf,"igrasptrans")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		for( m=0; m < 12; m++ )
		{
			sscanf( ret, " %d", &igrasptrans[m]);
			ret= strchr( ret, ',' );
			ret++;
		}
  }
  else{printf( "igrasptransは発見出来ません\n");}

	if((ret = strstr(lBuf,"imanipulatordirection")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		for( m=0; m < 3; m++ )
		{
			sscanf( ret, " %d", &imanipulatordirection[m]);
			ret= strchr( ret, ',' );
			ret++;
		}
  }
  else{printf( "imanipulatordirectionは発見出来ません\n");}

	if((ret = strstr(lBuf,"forceclosure")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		sscanf( ret, " %d", &forceclosure);
  }
  else{printf( "forceclosureは発見出来ません\n");}

	if((ret = strstr(lBuf,"grasptrans_nocol")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		for( m=0; m < 12; m++ )
		{
			sscanf( ret, " %d", &grasptrans_nocol[m]);
			ret= strchr( ret, ',' );
			ret++;
		}
  }
  else{printf( "grasptrans_nocolは発見出来ません\n");}

	if((ret = strstr(lBuf,"performance")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		sscanf( ret, " %d", &performance);
  }
  else{printf( "performanceは発見出来ません\n");}

	if((ret = strstr(lBuf,"GDistance")) != NULL )
	{
		ret= strchr( ret, '[' );
		ret++;
		sscanf( ret, " %d", &GDistance);
  }
  else{printf( "GDistanceは発見出来ません\n");}

	fclose(fp);
/////////////////////////////////////////
	end = clock();//データ読み込み時
	printf("%.2f秒(データ読み込み)\n",(double)(end-start)/CLOCKS_PER_SEC);//データ読み込み時

	string scenefilename = "data/NEXTAGEenv/nextage_ORNAMENT_20130901.env.xml";
	string viewername = "qtcoin";
	RaveInitialize(true);
	EnvironmentBasePtr penv = RaveCreateEnvironment();

	CollisionReportPtr _report;//Grasperの代役用に追加
	_report.reset(new CollisionReport());//Grasperの代役用に追加

	boost::thread thviewer(boost::bind(SetViewer,penv,viewername)); // create the viewer
	usleep(200000); // wait for the viewer to init
	penv->Load(scenefilename);
	usleep(100000); // wait for the viewer to init

	vector<RobotBasePtr> vrobots;
	penv->GetRobots(vrobots);
	RobotBasePtr probot = vrobots.at(0);
	vector<dReal> vlower,vupper,v(probot->GetDOF()),vINI(probot->GetDOF());
	list< vector< dReal > > vlog;
	list< vector< dReal > > vhand;
	probot->GetDOFLimits(vlower,vupper);

	// set all dofs as active
	vector<int> vindices(probot->GetDOF());
	for(size_t i = 0; i < vindices.size(); ++i){vindices[i] = i;}
	probot->SetActiveDOFs(vindices);

	//時間保存用ファイル生成
	if((fpp=fopen("NEXTAGE_TIME_ORNAMENT.txt","w"))==NULL)//138把持は取り込み可能
	{
		printf("Can't open\n");
		exit(EXIT_FAILURE);
	}
	fprintf(fpp, "把持リスト作成\t軌道生成\tデータ転送\t実機動作\t(全て秒)\n");//項目
	fclose(fpp);
	//時間保存用ファイル生成終了
	///IK用処理
	// find a manipulator chain to move
	for(size_t i = 0; i < probot->GetManipulators().size(); ++i)
	{
		if( probot->GetManipulators()[i]->GetName().find("arm") != string::npos )
		{
			probot->SetActiveManipulator(probot->GetManipulators()[i]);
			break;
		}
	}
	RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
	// load inverse kinematics using ikfast
	ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
	penv->AddModule(pikfast,"");
	stringstream ssin,ssout;
	vector<dReal> vsolution;
	ssin << "LoadIKFastSolver " << probot->GetName() << " " << (int)IKP_Transform6D;
	if( !pikfast->SendCommand(ssout,ssin) ){RAVELOG_ERROR("failed to load iksolver\n");}
	if( !pmanip->GetIkSolver()){penv->Destroy();return 1;}
	///IK用処理終了

	///CDemoModule確認用
//printf("a\n");
	ModuleBasePtr pcdemomodule = RaveCreateModule(penv,"CDemoModule");
	penv->AddModule(pcdemomodule,"");//この時にmodule内のmainが実行される。ここは、mainを実行しないならば必要ないかも。
	ssin.clear();
	ssin << "numbodies";
	if(!pcdemomodule->SendCommand(ssout,ssin)){RAVELOG_ERROR("failed to load CDemoModule\n");}
	else
	{
		int numbodies;
		ssout >> numbodies;
    RAVELOG_INFO("number of bodies are: %d\n",numbodies);
	}
//	printf("b\n");
//  c=getchar();
	ssin.clear();
	ssin << "load";
	if(!pcdemomodule->SendCommand(ssout,ssin)){RAVELOG_ERROR("failed to load CDemoModule load\n");}


	///CDemoModule確認用終了

	//NEXHAND起動準備
	probot->GetActiveDOFValues(v);// ロボットの角度をvに代入
	double deg=0.0;
	deg=HAND1-180.0*(v[HAND1id])/3.1415*10.0;
	moveFinger( fd, deg ,1);
	deg=HAND2+180.0*(v[HAND2id])/3.1415*10.0;
	moveFinger( fd, deg ,2);
	deg=HAND3+180.0*(v[HAND3id])/3.1415*10.0;
	moveFinger( fd, deg ,3);
	deg=HAND4+180.0*(v[HAND4id])/3.1415*10.0;
	moveFinger( fd, deg ,4);
	deg=HAND5+180.0*(v[HAND5id])/3.1415*10.0;
	moveFinger( fd, deg ,5);
	deg=HAND6+180.0*(v[HAND6id])/3.1415*10.0;
	moveFinger( fd, deg ,6);
	deg=HAND7+180.0*(v[HAND7id])/3.1415*10.0;
	moveFinger( fd, deg ,7);
	deg=HAND8+180.0*(v[HAND8id])/3.1415*10.0;
	moveFinger( fd, deg ,8);
	deg=HAND9+180.0*(v[HAND9id])/3.1415*10.0;
	moveFinger( fd, deg ,9);
	// ※ 下の一文をハンド起動時に実行すると関節が柔らかくなる
	RSSetComp( fd, 50, 50, 255 ); //すべてのサーボのコンプライアンスを下げる
	RSSetComp( fd, 20, 20, 1 ); //根元のサーボのコンプライアンスを下げる
	RSSetComp( fd, 20, 20, 4 ); //根元のサーボのコンプライアンスを下げる
	RSSetComp( fd, 20, 20, 7 ); //根元のサーボのコンプライアンスを下げる
	usleep(10000);
	//NEXHAND起動準備終了
	///target位置(事前定義)
	KinBodyPtr targetbody;
	string name="ORNAMENT";//掴む物体
	targetbody = penv->GetKinBody(name);
	Transform targetbodycenter = targetbody->GetTransform();
	///target位置(事前定義)終了
	start = clock();//事前計算時
///[①④は事前計算なので一度計算したらもう計算しない様にする]最初にgraspingの時と同じ姿勢の時を考えてそれで①④の値を出しておく
//把持値の準備
	double NOENV[Tgrasp];
	for( m=0; m < Tgrasp; m++){NOENV[m]=1.00*exp(data[m][forceclosure]);}

	KinBodyPtr subbody1 = penv->GetKinBody("ORNAMENT");
	Transform subbodycenter1 = subbody1->GetTransform();
	{
		EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
		subbodycenter1.trans = Vector(0.0,subbodycenter1.trans.y,subbodycenter1.trans.z);
		subbody1->SetTransform(subbodycenter1);
	}
	RaveVector<dReal> RotAx;
	end = clock();//事前計算時
	printf("%.2f秒(事前計算)\n",(double)(end-start)/CLOCKS_PER_SEC);//事前計算時
	// TCPIPソケットの生成
	at_Nextage *nxtg;
	// TCIP接続
	nxtg = new at_Nextage((char*)NEXTAGEIP,9876);
	if(!nxtg->cnct){printf("TCP connect Err\n");}
	nxtg->calib();
	nxtg->init();
	probot->GetActiveDOFValues(vINI);// vINIにロボットの角度(初期値)を与える
	int Serecter=1;
while(1) //メインプログラム
{
	printf("1:従来手法\t2:提案手法(中心に近い把持)\n");
	scanf("%d",&Serecter);
	if(!(Serecter==1 || Serecter==2)){printf("1か2を入力してください\n");continue;}
	fpp=fopen("NEXTAGE_TIME_ORNAMENT.txt","a+");
	RobotBase::RobotStateSaver saver(probot); // save the state
	{
		EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
		subbodycenter1.trans = Vector(0.0,subbodycenter1.trans.y,subbodycenter1.trans.z);
		subbody1->SetTransform(subbodycenter1);
	}
	for( m=0; m < Tgrasp; m++ )//初期化
	{
		data[m][Tdof+1]=NOENV[m];
		if(Serecter==2){data[m][Tdof+1]=data[m][Tdof+1]*exp((0.07989-data[m][GDistance])/0.07989);}
	}
	targetbodycenter = subbodycenter1;
	start = clock();//把持リスト計算時

	Transform robotcenter = probot->GetTransform();
///Robot(肩)位置
	Transform robot_sh = probot->GetTransform();
	robot_sh.trans.x=robotcenter.trans.x/*腰周り軸-0.1450*-sin(v[0])*/;
	robot_sh.trans.y=robotcenter.trans.y-0.1450/*腰周り軸*cos(v[0])*/;
	robot_sh.trans.z=robotcenter.trans.z+0.4205;
//printf(">>robot_sh:x=%lf,y=%lf,z=%lf\n",robot_sh.trans.x,robot_sh.trans.y,robot_sh.trans.z);
///Robot(肩)位置終了
///cosθ計算
	Vector A,B;
	A=Vector(targetbodycenter.trans.x-robot_sh.trans.x , targetbodycenter.trans.y-robot_sh.trans.y , targetbodycenter.trans.z-robot_sh.trans.z);//肩→targetまでのベクトル
	float dir_cos=0.0;
	for( m=0; m < Tgrasp; m++ )
	{
		RotAx=Vector(data[m][igraspdir[0]],data[m][igraspdir[1]],data[m][igraspdir[2]]);//m番目の接近方向ベクトルを代入
		RotAx=geometry::quatRotate(targetbodycenter.rot,RotAx);//targetbodycenterの回転に合わせた値の変更
		B=RotAx;//ある把持でのハンドの接近方向ベクトル(物体の回転込み)
		dir_cos=(A.x*B.x+A.y*B.y+A.z*B.z)/(sqrt(A.x*A.x+A.y*A.y+A.z*A.z)+sqrt(B.x*B.x+B.y*B.y+B.z*B.z));
		data[m][Tdof+1]=data[m][Tdof+1]*exp(dir_cos);//把持値への代入
	}///cosθ計算終了

	ModuleBasePtr ptaskmanip = RaveCreateModule(penv,"TaskManipulation"); // create the module
	penv->AddModule(ptaskmanip,probot->GetName()); // load the module

	int dd[Tgrasp];
	{
		EnvironmentMutex::scoped_lock lock(penv->GetMutex());
		dReal conewidth = 0.25f*PI;
		int nDistMapSamples = 60000;

		Vector vmapcenter=Vector(targetbodycenter.trans.x,targetbodycenter.trans.y,targetbodycenter.trans.z);//掴む物体の中心
		RobotBase::RobotStateSaver saver1(probot);
		KinBody::KinBodyStateSaver saver2(targetbody);
		probot->Enable(false);
		targetbody->Enable(true);
		vector<CollisionReport::CONTACT> vpoints;
		RAY r;
		KinBody::Link::TRIMESH tri;
		CollisionReport::CONTACT pp;
		dReal ffar = 1.0f;
		penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts|CO_Distance);
		vpoints.reserve(nDistMapSamples);
		dReal counter = ffar/sqrt((dReal)nDistMapSamples/12);
		for(int k = 0; k < 6; k++)
		{
			for(dReal i = -ffar/2.0f; i < ffar/2.0f; i+=counter)
			{
				for(dReal j = -ffar/2.0f; j < ffar/2.0f; j+=counter)
				{
					switch(k){
						case 0:
							r.pos = Vector(vmapcenter.x-ffar,vmapcenter.y+i,vmapcenter.z+j);
							r.dir = Vector(1000,0,0);
							break;
						case 1:
							r.pos = Vector(vmapcenter.x+ffar,vmapcenter.y+i,vmapcenter.z+j);
							r.dir = Vector(-1000,0,0);
							break;
						case 2:
							r.pos = Vector(vmapcenter.x+i,vmapcenter.y-ffar,vmapcenter.z+j);
							r.dir = Vector(0,1000,0);
							break;
						case 3:
							r.pos = Vector(vmapcenter.x+i,vmapcenter.y+ffar,vmapcenter.z+j);
							r.dir = Vector(0,-1000,0);
							break;
						case 4:
							r.pos = Vector(vmapcenter.x+i,vmapcenter.y+j,vmapcenter.z-ffar);
							r.dir = Vector(0,0,1000);
							break;
						case 5:
							r.pos = Vector(vmapcenter.x+i,vmapcenter.y+j,vmapcenter.z+ffar);
							r.dir = Vector(0,0,-1000);
							break;
					}
					if( penv->CheckCollision(r, KinBodyConstPtr(targetbody), _report) )
					{
						pp.norm = -_report->contacts.at(0).norm;   
						pp.pos = _report->contacts.at(0).pos;   
						pp.depth = 0;
		 				vpoints.push_back(pp);
					}
				}
			}
		}
		penv->GetCollisionChecker()->SetCollisionOptions(0);
		targetbody->Enable(false);
		dReal fCosTheta = RaveCos(conewidth);
		int N;
		if(conewidth < 0.01f){N = 1;}
		penv->GetCollisionChecker()->SetCollisionOptions(CO_Distance);
		// set number of rays to randomly sample
		if( conewidth < 0.01f ){N = 1;}
		else{N=(int)ceil(conewidth*(64.0f/(PI/12.0f)));}// sample 64 points when at pi/12
		for(int i = 0; i < (int)vpoints.size(); ++i)
		{
			Vector vright = Vector(1,0,0);
			if( RaveFabs(vpoints[i].norm.x) > 0.9){vright.y = 1;}
			vright -= vpoints[i].norm * vright.dot3(vpoints[i].norm);
			vright.normalize3();
			Vector vup = vpoints[i].norm.cross(vright);
			dReal fMinDist = 2;
			for(int j = 0; j < N; ++j)
			{
				// sample around a cone
				dReal fAng = fCosTheta + (1-fCosTheta)*RaveRandomFloat();
				dReal R = RaveSqrt(1 - fAng * fAng);
				dReal U2 = 2 * PI * RaveRandomFloat();
				r.dir = 1000.0f*(fAng * vpoints[i].norm + R * RaveCos(U2) * vright + R * RaveSin(U2) * vup);
				r.pos = vpoints[i].pos;
				if( penv->CheckCollision(r, _report) ) 
				{
					if( _report->minDistance < fMinDist ){fMinDist = _report->minDistance;}
				}
			}
			vpoints[i].depth = fMinDist;
		}
		penv->GetCollisionChecker()->SetCollisionOptions(0);

		probot->Enable(true);
		targetbody->Enable(true);
//depth計算式
		for( m=0; m < Tgrasp; m++ )
		{
			RotAx=Vector(data[m][igrasppos[0]],data[m][igrasppos[1]],data[m][igrasppos[2]]);//m番目の把持点を代入
			RotAx=geometry::quatRotate(targetbodycenter.rot,RotAx);//targetbodycenterの回転に合わせた値の変更
			double last_res=1000;
			int num=0;
			for(int i = 0; i < (int)vpoints.size(); ++i)
			{
				double res=0;
				res=sqrt(pow(RotAx.x-(vpoints[i].pos.x-vmapcenter.x),2.0)+pow(RotAx.y-(vpoints[i].pos.y-vmapcenter.y),2.0)+pow(RotAx.z-(vpoints[i].pos.z-vmapcenter.z),2.0));
				if(last_res>res)
				{
					last_res=res;
					num=i;
				}
			}
		//data[m]に一番近いdepthの値は(vpoints[num].depth)
			data[m][Tdof+1]=data[m][Tdof+1]*exp(vpoints[num].depth);//把持値への代入

		}

///把持値による並べ替え
		for( m=0; m < Tgrasp; m++){dd[m]=-1;}
		for( m=0; m < Tgrasp; m++)
		{
			int saigo=-1;
			float last_hazi=0.000000;
			for(int i=0;i<Tgrasp;i++)
			{
				int cc=0;
				for(int j=0;j<Tgrasp;j++)	{if(i == dd[j]){cc=-1;}}
				if(cc==0 && last_hazi<data[i][Tdof+1])
				{
					last_hazi=data[i][Tdof+1];
					saigo=i;
				}
			}
			dd[m]=saigo;
		}
	}
	end = clock();//把持リスト計算時
	printf("%.2f秒(把持リスト計算)\n",(double)(end-start)/CLOCKS_PER_SEC);//把持リスト計算時
	fprintf(fpp, "%.2f\t",(double)(end-start)/CLOCKS_PER_SEC);//把持選択時

	ModuleBasePtr pbasemanip = RaveCreateModule(penv,"basemanipulation"); // create the module(直線軌道＆戻る軌道用)
  penv->AddModule(pbasemanip,probot->GetName()); // load the module

	int CN=0;
	m=0;
	start = clock();//把持選択時
	while(1) 
	{
		{
			EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
			for( m=0; m < Tgrasp; m++ )
			{
				stringstream cmdin,cmdout;
				cmdin << "GraspPlanning ";
				cmdin << "target ORNAMENT ";
				cmdin << "grasps ";
				cmdin << "1 " ;
				cmdin <<  0+3+3+1+1+Tpres+12+3+12 << " " ;//一把持当たりの配列数
				for( n=0; n < 3; n++ )
				{cmdin << data[dd[m]][igraspdir[n]] << " ";}
				for( n=0; n < 3; n++ )
				{cmdin << data[dd[m]][igrasppos[n]] << " ";}
				cmdin << data[dd[m]][igrasproll] << " ";
				cmdin << -data[dd[m]][igraspstandoff] << " ";
				for( n=0; n < Tpres; n++ )
				{cmdin << data[dd[m]][igrasppreshape[n]] << " ";}
				for( n=0; n < 12; n++ )
				{cmdin << data[dd[m]][igrasptrans[n]] << " ";}
				for( n=0; n < 3; n++ )
				{cmdin << data[dd[m]][imanipulatordirection[n]] << " ";}
				for( n=0; n < 12; n++ )
				{cmdin << data[dd[m]][grasptrans_nocol[n]] << " ";}
				cmdin << "igraspdir ";
				cmdin <<  0 << " " ;
				cmdin << "igrasppos ";
				cmdin <<  0+3 << " " ;
				cmdin << "igrasproll ";
				cmdin <<  0+3+3 << " " ;
				cmdin << "igraspstandoff ";
				cmdin <<  0+3+3+1 << " " ;
				cmdin << "igrasppreshape ";
				cmdin <<  0+3+3+1+1 << " " ;
				cmdin << "igrasptrans ";
				cmdin <<  0+3+3+1+1+Tpres << " " ;
				cmdin << "imanipulatordirection ";
				cmdin <<  0+3+3+1+1+Tpres+12 << " " ;
				cmdin << "grasptrans_nocol ";
				cmdin <<  0+3+3+1+1+Tpres+12+3 << " " ;
// start the planner and run the robot
				if( !ptaskmanip->SendCommand(cmdout,cmdin)) 
				{
					if(m==(Tgrasp-1)){printf("失敗\n");CN=1;break;}
					else{continue;}
				}
				end = clock();//把持選択時
				printf("%.2f秒(把持選択+軌道決定)\n",(double)(end-start)/CLOCKS_PER_SEC);//把持選択時
				fprintf(fpp, "%.2f\t",(double)(end-start)/CLOCKS_PER_SEC);//把持選択時
				printf("準備完了\tEnterで把持実行\n");
				c=getchar();
				c=getchar();
				printf("把持開始\n");
				break;
			}
		}

			int count=0;
			vlog.clear();
			vhand.clear();
			start = clock();//データ転送時
  		while(!probot->GetController()->IsDone())
  		{
				vector<dReal> v_(v.size());
				probot->GetActiveDOFValues(v);// ロボットに角度を設定
				for(size_t i = 0; i < v.size(); ++i){v_[i]=v[i];}
				vlog.push_back(v_);
				vhand.push_back(v_);
				usleep(5000);
  		}
  		while(!vlog.empty())
  		{
   			bool comp_mov=false;
   			while(!comp_mov)
   			{
    			list< vector<dReal> >::iterator v_ =vlog.begin(); 
    			comp_mov=true;
    			for(size_t i = 0; i < v.size(); ++i)
    			{
    	 			double agl=180.0* ((*v_)[i])/3.1415;

    	 			comp_mov &= nxtg->addAgl(i,agl);
    			}
   				nxtg->sendAllAgl(false);
count++;
					usleep(1000);
				}
				vlog.pop_front();
			}
			end = clock();//データ転送時
			printf("%.2f秒(データ転送)\n",(double)(end-start)/CLOCKS_PER_SEC);//データ転送時
			fprintf(fpp, "%.2f\t",(double)(end-start)/CLOCKS_PER_SEC);//データ転送時
			tensou=(double)(end-start)/CLOCKS_PER_SEC;//転送時間待機
			usleep(tensou*1000);//転送時間待機
			nxtg->startAnime();

			printf("%.2f秒(実機動作)\n",(double)5*count/1000);//実機動作時
			fprintf(fpp, "%.2f\n",(double)5*count/1000);//実機動作時
    	for(int ci = 0; ci < count; ++ci)//NEXHAND同期動作
    	{
  			while(!vhand.empty())
  			{
    			list< vector<dReal> >::iterator v_ =vhand.begin(); 
					double deg=0.0;
					deg=HAND1-180.0*((*v_)[HAND1id])/3.1415*10.0;
					moveFinger( fd, deg ,1);
					deg=HAND4+180.0*((*v_)[HAND4id])/3.1415*10.0;
					moveFinger( fd, deg ,4);
					deg=HAND7+180.0*((*v_)[HAND7id])/3.1415*10.0;
    	 		moveFinger( fd, deg ,7);
					vhand.pop_front();
				}
			usleep(1000*5);//(実機動作時間)
		}
		
		probot->GetActiveDOFValues(v);// ロボットの角度をvに代入
		printf("実機動作完了\n");
		if(CN!=1)//把持計画失敗時は省略する
		{	
			{//クローズフィンガー 
				EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
				stringstream cmdin,cmdout;
				cmdin << "CloseFingers ";
				// start the planner and run the robot
				if( !ptaskmanip->SendCommand(cmdout,cmdin) ) {continue;}
			}
			// unlock the environment and wait for the robot to finish
			while(!probot->GetController()->IsDone()) {usleep(1000);}
// NEXTAGE実機動作
			probot->GetActiveDOFValues(v);// ロボットの角度をvに代入
			deg=HAND1-180.0*(v[HAND1id])/3.1415*10.0;
			moveFinger( fd, deg ,1);
			deg=HAND2+180.0*(v[HAND2id])/3.1415*10.0+50.0;
			moveFinger( fd, deg ,2);
			deg=HAND3+180.0*(v[HAND3id])/3.1415*10.0+50.0;
			moveFinger( fd, deg ,3);
			deg=HAND4+180.0*(v[HAND4id])/3.1415*10.0;
			moveFinger( fd, deg ,4);
			deg=HAND5+180.0*(v[HAND5id])/3.1415*10.0+50.0;
			moveFinger( fd, deg ,5);
			deg=HAND6+180.0*(v[HAND6id])/3.1415*10.0+50.0;
			moveFinger( fd, deg ,6);
			deg=HAND7+180.0*(v[HAND7id])/3.1415*10.0;
			moveFinger( fd, deg ,7);
			deg=HAND8+180.0*(v[HAND8id])/3.1415*10.0+50.0;
			moveFinger( fd, deg ,8);
			deg=HAND9+180.0*(v[HAND9id])/3.1415*10.0+50.0;
			moveFinger( fd, deg ,9);
// NEXTAGE実機動作終了
			printf("把持完了\tEnetrで物体持ち上げ\n");
			c=getchar();
			probot->Grab(targetbody);//指と掴んだ物体を同じように動かさせる
			printf("上昇開始\n");
			double stepL=0.0030;
			while(1)//直線に一定距離
			{
				if(stepL<0.0001){printf("上昇失敗\n");break;}
				int mx=0,my=0,mz=1;
				{
					EnvironmentMutex::scoped_lock lock(penv->GetMutex()); //lock environment
			//find a new manipulator position and feed that into the planner. If valid, robot will move to it safely.
					ssin.str("");
					ssin.clear();
					ssin << "MoveHandStraight direction ";
					ssin << mx <<" ";
					ssin << my <<" ";
					ssin << mz <<" ";  
					ssin <<"minsteps 95 "; 
					ssin <<"maxsteps 105 "; 
					ssin <<"steplength "<< stepL; 
					// start the planner and run the robot
					RAVELOG_INFO("%s\n",ssin.str().c_str());
					if( !pbasemanip->SendCommand(ssout,ssin) ){stepL=stepL/2.000;continue;}
				}
    		// unlock the environment and wait for the robot to finish

int REcount=0;
			vlog.clear();
			vhand.clear();
			start = clock();//データ転送時
  		while(!probot->GetController()->IsDone())
  		{
		 		vector<dReal> v_(v.size());
				probot->GetActiveDOFValues(v);// ロボットに角度を設定
				for(size_t i = 0; i < v.size(); ++i){v_[i]=v[i];}
				vlog.push_back(v_);
				vhand.push_back(v_);
				usleep(10000);
  		}
  		while(!vlog.empty())
  		{
   			bool comp_mov=false;
   			while(!comp_mov)
   			{
    			list< vector<dReal> >::iterator v_ =vlog.begin(); 
    			comp_mov=true;
    			for(size_t i = 0; i < v.size(); ++i)
    			{
    	 			double agl=180.0* ((*v_)[i])/3.1415;
   	  			comp_mov &= nxtg->addAgl(i,agl);
   	 			}
   				nxtg->sendAllAgl(false);
REcount++;
					usleep(1000);
				}
				vlog.pop_front();
			}
			end = clock();//データ転送時
			tensou=(double)(end-start)/CLOCKS_PER_SEC;//転送時間待機
			usleep(tensou*1000);//転送時間待機
			nxtg->startAnime();
			printf("%.2f秒(上昇動作)\n",(double)5*REcount/1000);//実機動作時
    	for(int ci = 0; ci < REcount; ++ci)//NEXHAND同期動作
    	{
  			while(!vhand.empty())
  			{
    			list< vector<dReal> >::iterator v_ =vhand.begin(); 
					double deg=0.0;
					deg=HAND1-180.0*((*v_)[HAND1id])/3.1415*10.0;
					moveFinger( fd, deg ,1);
					deg=HAND4+180.0*((*v_)[HAND4id])/3.1415*10.0;
					moveFinger( fd, deg ,4);
					deg=HAND7+180.0*((*v_)[HAND7id])/3.1415*10.0;
    	 		moveFinger( fd, deg ,7);
					vhand.pop_front();
				}
			usleep(1000*5);//(実機動作時間)			
			}
			usleep(2000);
			break;
		}
		printf("上昇完了\tEnetrで初期姿勢へ移行\n");
		c=getchar();
		probot->Release(targetbody);
		usleep(50000);
		printf("ハンドオープン\n");
		{//オープンフィンガー 
			EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
			stringstream cmdin,cmdout;
			cmdin << "ReleaseFingers ";
			// start the planner and run the robot
			if( !ptaskmanip->SendCommand(cmdout,cmdin) ) {continue;}
		}
		// unlock the environment and wait for the robot to finish
		while(!probot->GetController()->IsDone()) {usleep(1000);}
// NEXTAGE実機動作
		probot->GetActiveDOFValues(v);// ロボットの角度をvに代入
		deg=HAND1-180.0*(v[HAND1id])/3.1415*10.0;
		moveFinger( fd, deg ,1);
		deg=HAND2+180.0*(v[HAND2id])/3.1415*10.0;
		moveFinger( fd, deg ,2);
		deg=HAND3+180.0*(v[HAND3id])/3.1415*10.0;
		moveFinger( fd, deg ,3);
		deg=HAND4+180.0*(v[HAND4id])/3.1415*10.0;
		moveFinger( fd, deg ,4);
		deg=HAND5+180.0*(v[HAND5id])/3.1415*10.0;
		moveFinger( fd, deg ,5);
		deg=HAND6+180.0*(v[HAND6id])/3.1415*10.0;
		moveFinger( fd, deg ,6);
		deg=HAND7+180.0*(v[HAND7id])/3.1415*10.0;
		moveFinger( fd, deg ,7);
		deg=HAND8+180.0*(v[HAND8id])/3.1415*10.0;
		moveFinger( fd, deg ,8);
		deg=HAND9+180.0*(v[HAND9id])/3.1415*10.0;
		moveFinger( fd, deg ,9);
// NEXTAGE実機動作終了
	}//if(CN!=1)把持計画失敗時は省略するここまで
	break;
	}
	usleep(10000);
	printf("初期姿勢移行開始\n");
	while(1)
	{				
    {    	 
     EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
     stringstream cmdin,cmdout;
		 cmdin.str("");
		 cmdin.clear();
     cmdin << "MoveActiveJoints goal ";
     for(size_t i = 0; i < vINI.size(); ++i) {cmdin << vINI[i] << " ";}
	     // start the planner and run the robot
     RAVELOG_INFO("%s\n",cmdin.str().c_str());
     if( !pbasemanip->SendCommand(cmdout,cmdin) ){continue;}
    }
    // unlock the environment and wait for the robot to finish
			int REcount=0;
			vlog.clear();
			vhand.clear();
			start = clock();//データ転送時
  		while(!probot->GetController()->IsDone())
  		{
		 		vector<dReal> v_(v.size());
				probot->GetActiveDOFValues(v);// ロボットに角度を設定
				for(size_t i = 0; i < v.size(); ++i){v_[i]=v[i];}
				vlog.push_back(v_);
vhand.push_back(v_);
				usleep(10000);
  		}
  		while(!vlog.empty())
  		{
   			bool comp_mov=false;
   			while(!comp_mov)
   			{
    			list< vector<dReal> >::iterator v_ =vlog.begin(); 
    			comp_mov=true;
    			for(size_t i = 0; i < v.size(); ++i)
    			{
    	 			double agl=180.0* ((*v_)[i])/3.1415;
   	  			comp_mov &= nxtg->addAgl(i,agl);
   	 			}
   				nxtg->sendAllAgl(false);
REcount++;
					usleep(1000);
				}
				vlog.pop_front();
			}
			end = clock();//データ転送時
			tensou=(double)(end-start)/CLOCKS_PER_SEC;//転送時間待機
			usleep(tensou*1000);//転送時間待機
			nxtg->startAnime();
			//printf("戻る%d回\n",REcount);//実機動作時
			printf("%.2f秒(戻る動作)\n",(double)5*REcount/1000);//実機動作時
    	for(int ci = 0; ci < REcount; ++ci)//NEXHAND同期動作
    	{
  			while(!vhand.empty())
  			{
    			list< vector<dReal> >::iterator v_ =vhand.begin(); 
					double deg=0.0;
					deg=HAND1-180.0*((*v_)[HAND1id])/3.1415*10.0;
					moveFinger( fd, deg ,1);
					deg=HAND4+180.0*((*v_)[HAND4id])/3.1415*10.0;
					moveFinger( fd, deg ,4);
					deg=HAND7+180.0*((*v_)[HAND7id])/3.1415*10.0;
    	 		moveFinger( fd, deg ,7);
					vhand.pop_front();
				}
			usleep(1000*5);//(実機動作時間省略)
			}
		usleep(2000);
		break;
	}

	fclose(fpp);
	{//元の位置に戻す
		EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
		targetbodycenter.trans = Vector(0.0,targetbodycenter.trans.y,targetbodycenter.trans.z);
		subbody1->SetTransform(targetbodycenter);
	}
	printf("1巡完了\n\n");
}//メインプログラムここまで
thviewer.join(); // wait for the viewer thread to exit
penv->Destroy(); // destroy

return rets;
}
