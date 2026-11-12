"""Exercise the actual CAN driver with mock HAL/RTOS; no hardware access."""
import argparse
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
PRELUDE = r'''
#include <cstdint>
#include <iostream>
#include <stdexcept>
using bool_t=bool; using BaseType_t=int; using HAL_StatusTypeDef=int;
struct CAN_TypeDef {}; CAN_TypeDef can1,can2,other;
#define CAN1 (&can1)
#define CAN2 (&can2)
#define USE_CAN1
#define USE_CAN2
struct CAN_HandleTypeDef {CAN_TypeDef* Instance;};
struct CAN_TxHeaderTypeDef {uint32_t StdId;}; struct CAN_RxHeaderTypeDef {uint32_t StdId;};
struct can_rx_message_t {CAN_RxHeaderTypeDef header;uint8_t data[8];uint32_t receivedAtMs;uint8_t receivedAtValid,sourceBus;};
using CAN_Call_Back=void(*)(can_rx_message_t*);
struct CAN_Manage_Object_t {CAN_HandleTypeDef* hcan;CAN_Call_Back rxCallbackFunction;};
struct CAN_FilterTypeDef {uint32_t FilterMode,FilterScale,FilterIdHigh,FilterIdLow,FilterMaskIdHigh,FilterMaskIdLow,FilterFIFOAssignment,FilterActivation,SlaveStartFilterBank,FilterBank;};
using osMessageQueueId_t=void*;
struct osMessageQueueAttr_t {const char* name;};
constexpr int HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2,pdFALSE=0,CAN_FILTERMODE_IDMASK=0,CAN_FILTERSCALE_32BIT=0,CAN_FilterFIFO0=0,CAN_FILTER_ENABLE=1,CAN_RX_FIFO0=0,CAN_IT_RX_FIFO0_MSG_PENDING=1;
bool inject=false,failAlloc=false,full=false;int starts=0,notifications=0,creates=0,stops=0,callbacks=0,drops=0,txWrites=0;
int filterResult=0,startResult=0,notifyResult=0,txResult=0,rxResult=0;
can_rx_message_t queued{},callbackFrame{};
uint32_t now=123;
uint32_t HAL_GetTick(){return now;}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*);
int HAL_CAN_Start(CAN_HandleTypeDef*){++starts;return startResult;}
int HAL_CAN_Stop(CAN_HandleTypeDef*){++stops;return HAL_OK;}
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef* c,int){++notifications;if(inject)HAL_CAN_RxFifo0MsgPendingCallback(c);return notifyResult;}
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,CAN_FilterTypeDef*){return filterResult;}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*,uint8_t*,uint32_t* box){
 if(!box)throw std::runtime_error("NULL mailbox");
 if(txResult==HAL_OK){*box=4;++txWrites;}return txResult;
}
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,int,CAN_RxHeaderTypeDef*,uint8_t* d){d[0]=0x9c;return rxResult;}
void* osMessageQueueNew(int,int,const osMessageQueueAttr_t*){++creates;return failAlloc?nullptr:reinterpret_cast<void*>(0x1000);}
int xQueueIsQueueFullFromISR(void* q){if(!q)throw std::runtime_error("NULL queue");return full;}
int xQueueReceiveFromISR(void*,can_rx_message_t*,BaseType_t*){++drops;return 1;}
int xQueueSendToBackFromISR(void* q,can_rx_message_t* m,BaseType_t*){if(!q)throw std::runtime_error("NULL queue");queued=*m;return 1;}
void portYIELD_FROM_ISR(BaseType_t){}
'''
TESTS = r'''
int checks=0;
void check(bool ok,const char* label){if(!ok)throw std::runtime_error(label);++checks;std::cout<<"PASS "<<label<<'\n';}
void callback(can_rx_message_t* m){++callbacks;callbackFrame=*m;}
void reset(){is_queue_initialized=false;canRxQueueHandle=nullptr;inject=failAlloc=full=false;starts=notifications=creates=stops=callbacks=drops=txWrites=0;filterResult=startResult=notifyResult=txResult=rxResult=0;queued={};}
int main(){
 CAN_HandleTypeDef c{CAN1},d{CAN2},bad{&other};
 reset();inject=true;check(CAN_Init(&c,callback)==HAL_OK&&callbacks==1&&creates==1,"pending RX has queue before notification");
 check(queued.receivedAtValid==1&&queued.receivedAtMs==123&&queued.sourceBus==1&&callbackFrame.receivedAtMs==123,"queue and callback share ISR receive timestamp");
 check(CAN_Init(&d,callback)==HAL_OK&&creates==1&&queued.sourceBus==2,"CAN2 shares queue and preserves source identity");
 full=true;HAL_CAN_RxFifo0MsgPendingCallback(&d);check(drops==1&&queued.data[0]==0x9c,"full queue drops oldest and enqueues received frame");
 reset();failAlloc=true;check(CAN_Init(&c,callback)==HAL_ERROR&&!is_queue_initialized&&starts==0&&notifications==0,"allocation failure blocks start");
 failAlloc=false;check(CAN_Init(&c,callback)==HAL_OK&&creates==2,"allocation failure permits retry");
 reset();filterResult=HAL_ERROR;check(CAN_Init(&c,callback)==HAL_ERROR&&starts==0&&notifications==0,"filter failure propagates before start");
 reset();startResult=HAL_BUSY;check(CAN_Init(&c,callback)==HAL_BUSY&&notifications==0,"start failure propagates before notification");
 reset();notifyResult=HAL_ERROR;check(CAN_Init(&c,callback)==HAL_ERROR&&stops==1,"notification failure attempts stop");
 reset();check(CAN_Init(nullptr,callback)==HAL_ERROR&&CAN_Init(&bad,callback)==HAL_ERROR&&creates==0,"null or unconfigured controller rejected");
 HAL_CAN_RxFifo0MsgPendingCallback(&c);check(callbacks==0,"early callback before queue is harmless");
 CAN_Init(&c,callback);rxResult=HAL_ERROR;HAL_CAN_RxFifo0MsgPendingCallback(&c);check(callbacks==0,"RX failure never publishes a frame");
 uint8_t data[8]{};CAN_TxHeaderTypeDef h;
 check(CAN_Send_Data(&c,&h,data)==HAL_OK&&txWrites==1,"HAL can write mailbox output on successful send");
 txResult=HAL_ERROR;check(CAN_Send_Data(&c,&h,data)==HAL_ERROR&&txWrites==1,"mailbox-full error propagated without fake success");
 txResult=HAL_BUSY;check(CAN_Send_Data(&c,&h,data)==HAL_BUSY,"HAL busy preserved");
 std::cout<<checks<<" mocked HAL/RTOS checks passed; no target timing validation.\n";
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    out = parser.parse_args().output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    body = (ROOT/'GSRL/Driver/src/drv_can.c').read_text(encoding='utf-8-sig')
    body = body.replace('#include "drv_can.h"', '').replace('#include "queue.h"', '')
    source = out/'can_driver_test.cpp'
    source.write_text(PRELUDE+body+TESTS, encoding='utf-8')
    exe = out/('can_driver_test.exe' if os.name == 'nt' else 'can_driver_test')
    for command, log in [([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wno-missing-field-initializers', str(source), '-o', str(exe)], 'build.log'), ([str(exe)], 'results.log')]:
        result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace')
        (out/log).write_text(result.stdout+result.stderr, encoding='utf-8')
        print(result.stdout+result.stderr, end='')
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
