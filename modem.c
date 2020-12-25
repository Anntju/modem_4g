#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <termios.h>
#include <errno.h>
#include <getopt.h>
#include <string.h>
#include <net/if.h>
#include <linux/socket.h>
#include <sys/ioctl.h> 
#include <netinet/in.h>
#include <arpa/inet.h>
#include "log.h"
#include "atchannel.h"
#include "at_tok.h"
#include "modem.h"

#define DEV_NAME  "/dev/ttyUSB2"
#define DEFALUT_PORT_NAME "wwan0"
#define COMMAND_1  "udhcpc -i wwan0"
#define COMMAND_2  "ping www.baidu.com -c 4" 

static int speed_arr[] = { 
	B921600, B460800, B230400, B115200, B57600, B38400, B19200, 
	B9600, B4800, B2400, B1200, B300, 
};

static int name_arr[] = {
	921600, 460800, 230400, 115200, 57600, 38400,  19200,  
	9600,  4800,  2400,  1200,  300,  
};
 
int current_dbg_level = LOG_ERROR;
static pthread_mutex_t s_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_state_cond = PTHREAD_COND_INITIALIZER;
static const char * s_device_path = NULL;
static RIL_RadioState sState = RADIO_STATE_UNAVAILABLE; 
static int s_lac = 0;
static int s_cid = 0;
/* trigger change to this with s_state_cond */
static int s_closed = 0;

static void setRadioState(RIL_RadioState newState);
static SIM_Status getSIMStatus();
static POLL_Status pollSIMState (int repeat_time);


static void waitForClose()
{
    pthread_mutex_lock(&s_state_mutex);

    while (s_closed == 0) {
        pthread_cond_wait(&s_state_cond, &s_state_mutex);
    }

    pthread_mutex_unlock(&s_state_mutex);
}


/** do post-AT+CFUN=1 initialization */
static void onRadioPowerOn()
{
     sleepMsec(1000);
     pollSIMState(1);
}


static void setRadioState(RIL_RadioState newState)
{
    
    RIL_RadioState oldState;
    RLOGD("setRadioState(%d)\n", newState);
    pthread_mutex_lock(&s_state_mutex);

    oldState = sState;

    if (s_closed > 0) {
        // If we're closed, the only reasonable state is
        // RADIO_STATE_UNAVAILABLE
        // This is here because things on the main thread
        // may attempt to change the radio state after the closed
        // event happened in another thread
        newState = RADIO_STATE_UNAVAILABLE;
    }

    if (sState != newState || s_closed > 0) {
        sState = newState;

        pthread_cond_broadcast (&s_state_cond);
    }

    pthread_mutex_unlock(&s_state_mutex);


    /* do these outside of the mutex */
    if (sState != oldState) {
      //  RIL_onUnsolicitedResponse (RIL_UNSOL_RESPONSE_RADIO_STATE_CHANGED,
      //                              NULL, 0);
        // Sim state can change as result of radio state change
      //  RIL_onUnsolicitedResponse (RIL_UNSOL_RESPONSE_SIM_STATUS_CHANGED,
      //                              NULL, 0);

        /* FIXME onSimReady() and onRadioPowerOn() cannot be called
         * from the AT reader thread
         * Currently, this doesn't happen, but if that changes then these
         * will need to be dispatched on the request thread
         */
        if (sState == RADIO_STATE_ON) {
            onRadioPowerOn();
        }
    }
}
/** returns 1 if line starts with prefix, 0 if it does not */
/*int strStartsWith(const char *line, const char *prefix)
{
    for ( ; *line != '\0' && *prefix != '\0' ; line++, prefix++) {
        if (*line != *prefix) {
            return 0;
        }
    }

    return *prefix == '\0';
}
**/
/**
 * Called by atchannel when an unsolicited line appears
 * This is called on atchannel's reader thread. 
 */
static void onUnsolicited (const char *s, const char *sms_pdu)
{
/*
    if (strStartsWith(s, "+CFUN: 0")) {
        setRadioState(RADIO_STATE_OFF);
    }
*/
}

/* Called on command or reader thread */
static void onATReaderClosed()
{
    RLOGI("AT channel closed\n");
    at_close();
    s_closed = 1;
    setRadioState (RADIO_STATE_UNAVAILABLE);
}

/* Called on command thread */
static void onATTimeout()
{
    RLOGI("AT channel timeout; closing\n");
    at_close();
    s_closed = 1;

    /* FIXME cause a radio reset here */
    setRadioState (RADIO_STATE_UNAVAILABLE);
}


/** Returns SIM_NOT_READY on error */
static SIM_Status getSIMStatus()
{
    ATResponse *p_response = NULL;
    int err;
    int ret;
    char *cpinLine;
    char *cpinResult;

    RLOGD("getSIMStatus\n");
    err = at_send_command_singleline_deftimeout("AT+CPIN?", "+CPIN:", &p_response);
	RLOGD("getSIMStatus  err=%d -----------function:%s line:%d  --------\n",err,__FUNCTION__,__LINE__);
	if (err != 0) {
        ret = SIM_NOT_READY;
        goto done;
    }

    switch (at_get_cme_error(p_response)) {
        case CME_SUCCESS:
            RLOGD("getSIMStatus  CME_SUCCESS\n" );
            break;
        case CME_SIM_NOT_INSERTED:
            ret = SIM_ABSENT;
            RLOGD("getSIMStatus  SIM_ABSENT \n" );
            goto done;
        /**/
        case CME_SIM_FAILURE:
            ret = SIM_FAILURE;
            RLOGD("getSIMStatus  CME_SIM_FAILURE \n" );
            goto done;    /* */ 
        default:
            ret = SIM_NOT_READY;
            RLOGD("getSIMStatus default set SIM_NOT_READY \n");
            goto done;
    }

    /* CPIN? has succeeded, now look at the result */
    cpinLine = p_response->p_intermediates->line;
    err = at_tok_start (&cpinLine);
    if (err < 0) {
        ret = SIM_NOT_READY;
        goto done;
    }
 
    err = at_tok_nextstr(&cpinLine, &cpinResult);
    if (err < 0) {
        ret = SIM_NOT_READY;
        goto done;
    }

    if (0 == strcmp (cpinResult, "SIM PIN")) {
        ret = SIM_PIN;
        goto done;
    } else if (0 == strcmp (cpinResult, "SIM PUK")) {
        ret = SIM_PUK;
        goto done;
    } else if (0 == strcmp (cpinResult, "PH-NET PIN")) {
        return SIM_NETWORK_PERSONALIZATION;
    } else if (0 != strcmp (cpinResult, "READY"))  {
        /* we're treating unsupported lock types as "sim absent" */
        ret = SIM_ABSENT;
        goto done;
    }

    at_response_free(p_response);
    p_response = NULL;
    cpinResult = NULL;

    ret = SIM_READY;
done:
    at_response_free(p_response);
    RLOGD("getSIMStatus -----------function:%s line:%d  --------\n",__FUNCTION__,__LINE__);
    return ret;
}

 
static POLL_Status pollSIMState (int repeat_time)
{
      
    int count = 0;

    //if (sState != RADIO_STATE_UNAVAILABLE) {
    //    // no longer valid to poll
    //    return;
    //}
    do{
		count++;
		 
		switch(getSIMStatus()) {
			case SIM_READY:
				RLOGI("SIM_READY\n");
			//	RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_SIM_STATUS_CHANGED, NULL, 0);
			return POLL_OK;
			case SIM_NOT_READY:
				RLOGI("SIM NOT READY,TRY AGAIN!\n");
				continue;
			case SIM_ABSENT:
			case SIM_PIN:
			case SIM_PUK:
			case SIM_NETWORK_PERSONALIZATION:
            case SIM_FAILURE:
			default:
				RLOGI("SIM ABSENT or LOCKED\n");
			//	RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_SIM_STATUS_CHANGED, NULL, 0);
			    return POLL_ERROR_SIM;
			
		}
	}while(count < repeat_time);

	return POLL_FAILD;
}

 
void set_speed(int fd, int speed)
{
	int   i;
	int   status;
	struct termios   Opt;
	tcgetattr(fd, &Opt);

	for(i=0; i<sizeof(speed_arr)/sizeof(int);i++)
	{
		if(speed == name_arr[i])
		{
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if(status != 0)
			{
                perror("tcsetattr fd1");
            }
            return;
		}
		tcflush(fd,TCIOFLUSH);
  	 }

	if (i == 12)
	{ 
		RLOGE("\tSorry, please set the correct baud rate!\n\n");
	}
}
/*
	*@brief   设置串口数据位，停止位和效验位
	*@param  fd     类型  int  打开的串口文件句柄*
	*@param  databits 类型  int 数据位   取值 为 7 或者8*
	*@param  stopbits 类型  int 停止位   取值为 1 或者2*
	*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
static POLL_Status set_Parity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;
	if(tcgetattr(fd,&options) != 0) 
	{
		perror("SetupSerial 1");
		return(POLL_FAILD);
	}
	options.c_cflag &= ~CSIZE ;
	switch (databits) /*设置数据位数*/ 
	{
    	case 7:
    		options.c_cflag |= CS7;
    	break;
    	case 8:
    		options.c_cflag |= CS8;
    	break;
    	default:
    		RLOGE("error！Unsupported data size\n");
    	    return (POLL_FAILD);
	}
	
	switch (parity) 
	{
    	case 'n':
    	case 'N':
    		options.c_cflag &= ~PARENB;            /* Clear parity enable */
    		options.c_iflag &= ~INPCK;             /* Enable parity checking */
    	break;
    	case 'o':
    	case 'O':
    		options.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/
    		options.c_iflag |= INPCK;              /* Disnable parity checking */
    	break;
    	case 'e':
    	case 'E':
    		options.c_cflag |= PARENB;             /* Enable parity */
    		options.c_cflag &= ~PARODD;            /* 转换为偶效验*/ 
    		options.c_iflag |= INPCK;              /* Disnable parity checking */
    	break;
    	case 'S':	
    	case 's':
    		options.c_cflag &= ~PARENB;
    		options.c_cflag &= ~CSTOPB;
    	break;
    	default:
    		RLOGE("error！Unsupported parity\n");
    		return (POLL_FAILD);
	}
 	/* 设置停止位*/  
  	switch (stopbits) 
  	{
       	case 1:
        	options.c_cflag &= ~CSTOPB;
      	break;
     	case 2:
      		options.c_cflag |= CSTOPB;
      	break;
     	default:
      		RLOGE("error！Unsupported stop bits\n");
      		return (POLL_FAILD);
 	}

  	if (parity != 'n')
  	{
    	options.c_iflag |= INPCK;
    }
  	options.c_cc[VTIME] = 10; // 1 seconds
    options.c_cc[VMIN] = 0;

	options.c_lflag &= ~(ECHO | ICANON);

  	tcflush(fd,TCIFLUSH);
  	if (tcsetattr(fd,TCSANOW,&options) != 0) 
  	{
    	perror("SetupSerial 3");
  		return (POLL_FAILD);
 	}
	return (POLL_OK);
}

//static void rintequestSignalStrength(void *data __unused, size_t datalen __unused, RIL_Token t)
static int requestSignalStrength(RIL_SignalStrength *r_response)
{
    ATResponse *p_response = NULL;
    int err;
    char *line;
    int count = 0; 
    int NumOfElements=sizeof(RIL_SignalStrength)/sizeof(int);
    int response[NumOfElements];
    
    RLOGD("requestSignalStrength");  
    memset(response, 0, sizeof(response));

    err = at_send_command_singleline_deftimeout("AT+CSQ", "+CSQ:", &p_response);

    if (err < 0 || p_response->success == 0) {
     //   RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
        goto error;
    }

    line = p_response->p_intermediates->line;
     
    err = at_tok_start(&line);
    if (err < 0) goto error;
     
    for (count = 0; count < NumOfElements; count++) {
        err = at_tok_nextint(&line, &(response[count]));
        if (err < 0 && count < NumOfElements) goto error;
    }

   // RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(response));
    r_response->signalStrength = response[0];
    r_response->bitErrorRate = response[1]; 
    RLOGD("r_response->signalStrength=%d  r_response->bitErrorRate =%d  ",r_response->signalStrength,r_response->bitErrorRate);
    at_response_free(p_response);
    return 0;

error:
    RLOGE("requestSignalStrength must never return an error when radio is on");
   // RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(p_response);
    return err;
}
 
/**
 * check SignalStrength CSQ
 */

static POLL_Status pollSignalStrength (int repeat_time)
{
    int ret;
    int count = 0;
    RIL_SignalStrength R_signalstrength={0,0};
    
    //if (sState != RADIO_STATE_UNAVAILABLE) {
        // no longer valid to poll
    //    return;
    //}
    do{
		count++;
		 
        memset(&R_signalstrength, 0, sizeof(RIL_SignalStrength));
        ret = requestSignalStrength(&R_signalstrength);
        
        if( ret < 0 || (0 == R_signalstrength.signalStrength)) {
            RLOGI(" FAILED TO GET signalStrength or signalStrength is too low\n");
            continue;
        }
        else if((99 == R_signalstrength.signalStrength) || (199 == R_signalstrength.signalStrength)) {
            RLOGI("signalStrength : not known or not detectable\n");
           // continue;
            return POLL_OK;    
        }
        else{
            RLOGI("signalStrength = %d\n",R_signalstrength.signalStrength);
            return POLL_OK;    
        }
           
	}while(count < repeat_time);

	return POLL_FAILD;
}

static void requestRegistrationState(int *regStat)                                       
{
    int err;
    ATResponse *p_response = NULL;  
    char *line;
    int commas, *resp = NULL, skip;
    char *p;
    
    RLOGD("requestRegistrationState");  
    err = at_send_command_singleline_deftimeout("AT+CGREG?", "+CGREG:", &p_response);

    if (err != 0) goto error;

    line = p_response->p_intermediates->line;
      
    err = at_tok_start(&line);
    if (err < 0) goto error;
     
    /* count number of commas */
    commas = 0;
    for (p = line ; *p != '\0' ;p++) {
        if (*p == ',') commas++;
    }
     
    resp = (int *)calloc(commas + 1, sizeof(int));
    if (!resp) goto error;
    switch (commas) {
       case 0: /* +CREG: <stat> */
            err = at_tok_nextint(&line, &resp[0]);
            if (err < 0) goto error;
            resp[1] = -1;
            resp[2] = -1;
        break;
        case 1: /* +CREG: <n>, <stat> */
            err = at_tok_nextint(&line, &skip);
            if (err < 0) goto error;
            err = at_tok_nextint(&line, &resp[0]);
            if (err < 0) goto error;
            resp[1] = -1;
            resp[2] = -1;
            if (err < 0) goto error;
        break;
        case 2: /* +CREG: <stat>, <lac>, <cid> */
            err = at_tok_nextint(&line, &resp[0]);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &resp[1]);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &resp[2]);
            if (err < 0) goto error;
        break;
        case 3: /* +CREG: <n>, <stat>, <lac>, <cid> */
            err = at_tok_nextint(&line, &skip);
            if (err < 0) goto error;
            err = at_tok_nextint(&line, &resp[0]);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &resp[1]);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &resp[2]);
            if (err < 0) goto error;
        break;
        default:
            goto error;
    }
    s_lac = resp[1];
    s_cid = resp[2];
    *regStat = resp[0];
   
    free(resp); 
    at_response_free(p_response);
    return;
error:
    free(resp);
    RLOGE("requestRegistrationState must never return an error when radio is on");
    //RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(p_response);
    return;
}
 
static POLL_Status pollRegistrationState (int repeat_time)
{
    int count = 0;
    int RegistrationState;
    //if (sState != RADIO_STATE_UNAVAILABLE) {
        // no longer valid to poll
    //    return;
    //}
    do{
		count++;

        memset(&RegistrationState, 0, sizeof(RIL_SignalStrength));
        requestRegistrationState(&RegistrationState);
       
        if((RIL_REG_HOME == RegistrationState) || (RIL_REG_ROAMING == RegistrationState)) {
            RLOGI(" Modem registered\n");   
            return POLL_OK;    
        }
        else{
            RLOGI("Modem not registered.RegistrationState = %d\n",RegistrationState);
            continue;   
        }
           
	}while(count < repeat_time);
  
	return POLL_FAILD;
}

static void setRadioShutdown(void)
{
    int err;
    ATResponse *p_response = NULL;
    
    RLOGD("setRadioShutdown\n");
    if (sState != RADIO_STATE_OFF) {
        err = at_send_command("AT+CFUN=0", &p_response);
        setRadioState(RADIO_STATE_UNAVAILABLE);
    }

    at_response_free(p_response);
   // RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;
}

static void setRadioOn(void)
{
    int err;
    ATResponse *p_response = NULL;

    RLOGD("setRadioOn\n");
    if (sState == RADIO_STATE_OFF) {
        err = at_send_command("AT+CFUN=1", &p_response);
        setRadioState(RADIO_STATE_ON);
    }

    at_response_free(p_response);
  //  RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;
}

/**
 * Initialize everything 
 */
static int initialize( )
{
    int ret;

    at_handshake();
     /*  Extended errors show in number*/
    at_send_command("AT+CMEE=1", NULL);

do_init:
    ret = pollSIMState(40);
	if (ret == POLL_FAILD) {
		RLOGE ("pollSIMState:sim not ready\n" );
		goto do_reset;
        }
    else if(ret == POLL_ERROR_SIM) {
        RLOGE("pollSIMState: SIM ABSENT or LOCKED \n");
        goto getout_error;
        }
          
    ret = pollSignalStrength(100);
	if (ret != POLL_OK) {
		RLOGE ("AT error %d on pollSIMState\n", ret);
		goto do_reset;
        } 
    /**/ 
    ret = pollRegistrationState(100); //100 times
	if (ret != POLL_OK) {
		RLOGE ("AT error %d on pollRegistrationState\n", ret);
		goto do_reset;
        } 
	else {
		RLOGI ("The module initialization is complete\n");
		return 0;	
    }	
   
do_reset:
    setRadioShutdown();
	sleepMsec(5000);
	setRadioOn();
	goto do_init;

getout_error:
	return -1;
	
}

static int NDISDial(void)
{
    int err = 0;
    ATResponse *p_response = NULL;
     
    err = at_send_command("AT$QCRMCALL=1,1", &p_response);

    if (err < 0 || p_response->success == 0) {
        goto error;
    }
    
    RLOGD("NDISDial SUCCESS\n");
    at_response_free(p_response);
    return 0;

error:
    RLOGD("err=%d\n",err);
    RLOGE("NDISDial Failed\n");
   // RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(p_response);
    return -1; 
 
}
/*
when $QCRMCALL is DOWN :
AT$QCRMCALL?
OK

when $QCRMCALL is UP:
AT$QCRMCALL?
$QCRMCALL: 1, V4

OK
*/
static int check_Dial_status(void)
{
    int err = 0;
    ATResponse *p_response = NULL;
    char *line;
    char *ip_type;
    int status;

    RLOGD("check_Dial_status");  
  
    err = at_send_command_singleline("AT$QCRMCALL?", "$QCRMCALL:", &p_response);

    if (err != 0) goto error;

    line = p_response->p_intermediates->line;
      
    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &status);
    if (err < 0) goto error;

    err = at_tok_nextstr(&line, &ip_type);
    if (err < 0)  goto error;

    RLOGI("dail status=%d ip_type=%s\n",status,ip_type);

    //if (0 == strcmp (ip_type, "V4")) {}
    ip_type = NULL;
    RLOGD("check_Dial_status SUCCESS\n");
    at_response_free(p_response);
    return 0;

error:
    RLOGD("err=%d\n",err);
    RLOGE("check_Dial_status Failed OR Dial status is down\n");
   // RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    ip_type = NULL;
    at_response_free(p_response);
    return -1; 
 
}

static int get_inf_ip(const char *eth_inf,char *svr_ip)
{

	int sock;
	struct sockaddr_in sin;
	struct ifreq ifr;
	char *ip;
	
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock == -1)
	{
		perror("socket");
		return -1;	
	}
	
	strncpy(ifr.ifr_name, eth_inf, IFNAMSIZ);
	ifr.ifr_name[IFNAMSIZ - 1] = 0;
	 
	if ( ioctl(sock, SIOCGIFADDR, &ifr) < 0)
	{
		perror("ioctl");
		goto error;	
	}
	memcpy(&sin, &ifr.ifr_addr, sizeof(sin));	
	ip = inet_ntoa(sin.sin_addr);
	  printf("%s  strlen(ip)=%d \n",ip,strlen(ip));
	 memcpy(svr_ip,ip,strlen(ip)+1); 
	 	  printf("svr_ip=%s   \n",svr_ip);
 
	close(sock);
	return 0;
error:
	close(sock);
	return -1;	

}
static int dhcp_get_ip(void)
{
    int err = 0;
    char ip[20]="";
  

    system(COMMAND_1);
    sleepMsec(2000);
    err =   get_inf_ip(DEFALUT_PORT_NAME, ip);
    printf("IP =%s   \n",ip);
    if(err < 0){
        return -1;
    }

    return 0;     
}

static int test_network(void)
{
    int ret = 0;
    ret = system(COMMAND_2);
    if(ret != 0){
        RLOGE("PING failed\n");
        return -1;
    }

    return 0;
}

int at_main_loop(void)
{
    int ret;
    int fd = -1;
  
   	s_device_path = DEV_NAME;
	RLOGI("Opening tty device %s\n", s_device_path);
  
    at_set_on_reader_closed(onATReaderClosed);
    at_set_on_timeout(onATTimeout);

   // for (;;) {
        fd = -1;
        while  (fd < 0) {
            
            fd = open (s_device_path, O_RDWR);
#if 1               
			if ( fd >= 0 ){
				set_speed(fd, 9600);
				set_Parity(fd,8,1,'N');
			
			}
#endif            
            if (fd < 0){
                RLOGI("Opening AT interface. retrying...");
                sleepMsec(10000);
                /* never returns */
            }
        }

        s_closed = 0;
		//how to deal with the unsolicited information?
        //ret = at_open(fd, onUnsolicited);
		ret = at_open(fd, NULL);
        if (ret < 0) {
            RLOGE ("AT error %d on at_open\n", ret);
            return -1;
        }
 
        ret=initialize();
        if (ret < 0){//SIM not inserted or ...
            RLOGE("Initialize modem error , SIM not inserted or... \n"); 
            return -1;
        } 
       // sleepMsec(1000);
		ret = check_Dial_status();
        if (ret < 0){//dial status is down or failed to get status
            ret = NDISDial();
            if (ret < 0){
                RLOGE("NDIS Dial error %d \n", ret);
               // return 0;
            }  
            else{
                return 0;
            }
        }
        else{
            return 0;
        }

       // sleepMsec(1000);
        
       // waitForClose();
      //  RLOGI("Re-opening after close");
   // }

    return 0;
}

#if 0
int main (int argc, char **argv)
{
    int ret; 
    
init:
    
    at_main_loop();

    ret = dhcp_get_ip();
    if(0 != ret){
        RLOGE("DHCP error %d \n", ret);
        return 0; 
    }

    for (;;) {
        ret = test_network();
        if(0 != ret){
            RLOGE("test network error. Try to init network.\n");
            goto init;
        }
        printf("test_sleep\n");
        sleepMsec(1000*60*1); //check network every 10 min
    }

    return 0;
}
#else
int main (int argc, char **argv)
{
    int ret; 
    int fd = -1;
    int opt;

    while ( -1 != (opt = getopt(argc, argv, "d:h"))) {
        switch (opt) {
            case 'h':
                printf("-d : set log level 0-4.\n 0:off 1:error 2:warn 3:info 4:debug\n");
                return 0;
            break;
			case 'd':
				current_dbg_level = atoi(optarg);
                if(current_dbg_level > LOG_DEBUG)
                    current_dbg_level = LOG_DEBUG;
                RLOGE("log level:%d (0:off 1:error 2:warn 3:info 4:debug)\n",current_dbg_level);
            break;
            default:
                current_dbg_level = LOG_ERROR;
                RLOGE("log level:%d (0:off 1:error 2:warn 3:info 4:debug)\n",current_dbg_level);
        }
    }

init: 
    ret = at_main_loop();
    if(0 != ret){
        RLOGE("at error %d \n", ret);
        return 0; 
    }
    ret = dhcp_get_ip();
    if(0 != ret){
        RLOGE("DHCP error %d \n", ret);
        return 0; 
    }

    for (;;) {
        ret = test_network();
        if(0 != ret){
            RLOGE("test network error. Try to init network.\n");
            goto init;
        }
        //printf("test_sleep\n");
        sleepMsec(1000*60*10); //check network every 10 min
    }

    return 0;

}

#endif
