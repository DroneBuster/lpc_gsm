#ifndef SIM900D_H_
#define SIM900D_H_

typedef enum{
	SIM900D_RSP_ERROR,
	SIM900D_RSP_TIMEOUT,
	SIM900D_RSP_OK,
} rsp_result;


#define SIM900D_RESPONSE_FOUND 0

typedef struct{
	uint8_t msg_id;
} sim900d_msg;

#ifdef __cplusplus
extern "C" {
#endif
void reset_sim900d(void);
uint8_t init_sim900d(BaseChannel* chn);
rsp_result commandWaitOKRsp(BaseChannel* chn, uint8_t* comm, uint8_t n, uint32_t timeout);
rsp_result waitRsp(BaseChannel* chn, uint8_t rsp[], uint8_t n, uint32_t timeout);
bool_t parseChare(uint8_t c, sim900d_msg* msg);
#ifdef __cplusplus
}
#endif

#endif /* SIM900D_H_ */
