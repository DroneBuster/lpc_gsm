#include "ch.h"
#include "hal.h"

#include <sim900d.h>

#define RESP_BUF_SIZE 64

void reset_sim900d(void) {
	palClearPad(GPIO2, GPIO2_RES_LEA);
	chThdSleepMilliseconds(1000);
	palSetPad(GPIO2, GPIO2_RES_LEA);
}

/* Command line ending must be CR+LF
 * SIM900D after sending command will return command it self then CR + LF line
 * ending. Response format [sent text][0x0D][0x0A][0x0D][0x0A][text][0x0D][0x0A]
 */
rsp_result commandWaitOKRsp(BaseChannel* chn, uint8_t* comm, uint8_t n, uint32_t timeout) {
	chnWrite(chn, comm, n);
	systime_t millis = MS2ST(chTimeNow())+ timeout;

	/*while(MS2ST(chTimeNow()) <= millis){ //Synchronizing with sent message
		uint8_t c = chnGetTimeout(chn, TIME_IMMEDIATE);
		if(c == comm[matchNumber]){
			matchNumber++;
			if(matchNumber == n - 1)
				break;
		}
		else
			matchNumber = 0;
		chThdSleepMilliseconds(5); //Do not block other threads entirely
	}*/

	uint8_t prevC = 0;

	while(MS2ST(chTimeNow()) <= millis) {
		uint8_t cr = chnGetTimeout(chn, TIME_IMMEDIATE);
		if(prevC == 'O' && cr == 'K')
			return SIM900D_RSP_OK;
		else
			prevC = cr;
		chThdSleepMilliseconds(5);
	}


	return SIM900D_RSP_ERROR;
}
/*
 * Wait for particular response
 */

rsp_result waitRsp(BaseChannel* chn, uint8_t rsp[], uint8_t n, uint32_t timeout) {
	systime_t time = MS2ST(chTimeNow());
	systime_t millis = time + timeout;

	uint8_t matchNr = 0; //Number of matches

	while(MS2ST(chTimeNow()) <= millis){
		uint8_t c = chnGetTimeout(chn, TIME_IMMEDIATE);
		if(c == rsp[matchNr]){
			matchNr++;
			if(matchNr == n - 1)
				return SIM900D_RSP_OK;
		}
		else
			matchNr = 0;
		chThdSleepMilliseconds(10);
	}
	if(MS2ST(chTimeNow()) > millis)
		return SIM900D_RSP_TIMEOUT;

	return SIM900D_RSP_ERROR;
}
/*
 * Init SIM900D
 */

uint8_t init_sim900d(BaseChannel* chn) {
	uint8_t buf[] = "\r\nRDY\r\n";
	do{
		uint8_t buff[64]; //flush
		chnReadTimeout(chn, buff, 64, TIME_IMMEDIATE);
		reset_sim900d();
		chThdSleepMilliseconds(2000);
	} while(waitRsp(chn, buf, sizeof(buf), 5000) != SIM900D_RSP_OK);
	chThdSleepMilliseconds(17000);
	uint8_t buff[64]; //flush
	chnReadTimeout(chn, buff, 64, TIME_IMMEDIATE);

	uint8_t buf0[] = "at+cipmode=1\r\n";
	if(commandWaitOKRsp(chn, buf0, sizeof(buf0), 10000) != SIM900D_RSP_OK) return 0;
	chThdSleepMilliseconds(10000);
	uint8_t buf1[] = "at+cstt=\"bangapro\"\r\n";
	if(commandWaitOKRsp(chn, buf1, sizeof(buf1), 10000) != SIM900D_RSP_OK) return 0;
	chThdSleepMilliseconds(10000);
	uint8_t buf2[] = "at+ciicr\r\n";
	if(commandWaitOKRsp(chn, buf2, sizeof(buf2), 10000) != SIM900D_RSP_OK) return 0;
	chThdSleepMilliseconds(10000);
	uint8_t buf4[] = "at+cifsr\r\n";
	chnWrite(chn, buf4, sizeof(buf4));
	chThdSleepMilliseconds(10000);
	uint8_t buf3[] = "AT+CIPSERVER=1,7561\r\n";
	if(commandWaitOKRsp(chn, buf3, sizeof(buf3), 10000) != SIM900D_RSP_OK) return 0;
	return 1;
}
