"""Compile actual Motor/LK source with a fake clock; no hardware access.

Run: python tests/host/test_lk_broadcast.py --output <build-directory>
Requires a native C++17 compiler (CXX, or g++ on PATH). Source extraction keeps
STM32 HAL/RTOS headers out of the host build; target compilation is also required.
"""
import argparse
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def method(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


PRELUDE = r'''
#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <algorithm>
using std::isfinite; using std::isnan; using fp32 = float;
constexpr fp32 MATH_PI = 3.14159265358979323846f;
constexpr uint32_t CAN_ID_STD=0, CAN_ID_EXT=4, CAN_RTR_DATA=0, CAN_RTR_REMOTE=2, DISABLE=0;
struct CAN_TxHeaderTypeDef { uint32_t StdId,ExtId,IDE,RTR,DLC,TransmitGlobalTime; };
struct CAN_RxHeaderTypeDef { uint32_t StdId,ExtId,IDE,RTR,DLC,Timestamp,FilterMatchIndex; };
struct can_rx_message_t { CAN_RxHeaderTypeDef header; uint8_t data[8]; uint32_t receivedAtMs; uint8_t receivedAtValid, sourceBus; };
class Controller {};
static uint32_t nowMs=0;
uint32_t HAL_GetTick() { return nowMs; }
'''

TESTS = r'''
static int checks=0;
void check(bool ok, const char* label) {
    if(!ok) throw std::runtime_error(label);
    ++checks; std::cout << "PASS " << label << '\n';
}
using L=MotorLKBroadcast;
can_rx_message_t reply(uint8_t cmd, uint16_t encoder=0, uint8_t id=1) {
    can_rx_message_t r{};r.header.StdId=0x140+id;r.header.DLC=8;
    r.data[0]=cmd;r.data[1]=26;r.data[6]=encoder&255;r.data[7]=encoder>>8;return r;
}
int main() {
    L m(1,nullptr,0,6); Motor& base=m;
    check(!base.isMotorConected()&&!m.isMotorOn()&&!m.isMotorStateKnown(),"startup unknown/offline through base API");
    for(int i=0;i<1000;i++) base.getMotorControlData();
    check(!base.isMotorConected(),"buffer getters never create a connection");
    auto r=reply(0x9C,34059);r.data[4]=0x98;r.data[5]=0xFE;
    check(m.decodeCanRxMessageFromISR(&r),"state2 accepted");
    check(std::fabs(m.getCurrentAngularVelocity()+MATH_PI/3)<1e-5f,"signed speed and gearbox conversion preserved");
    check(base.isMotorConected()&&m.isState2Fresh()&&!m.isState1Fresh()&&!m.isMotorStateKnown(),"feedback kinds have independent validity");
    check(std::fabs(m.getCurrentRevolutions()-34059.f/65536)<1e-5f,"first high encoder has absolute single-turn initial value");
    check(m.isRevolutionsValid(),"first position valid");
    nowMs=99;check(base.isMotorConected(),"99ms fresh");
    nowMs=100;check(!base.isMotorConected()&&!m.isState2Fresh()&&!m.isRevolutionsValid(),"100ms expires without calling control getter");
    auto s=reply(0x9A);s.data[2]=0x18;s.data[3]=9;
    m.decodeCanRxMessageFromISR(&s);
    check(m.isState1Fresh()&&m.isMotorOn()&&!m.isState2Fresh(),"status1 cannot refresh angle or speed");
    check(std::fabs(m.getBusVoltage()-23.28f)<1e-4f,"voltage decode");
    r=reply(0x9C,500);nowMs=101;m.decodeCanRxMessageFromISR(&r);
    check(!m.isRevolutionsValid(),"gap invalidates multi-turn history after reconnect");
    base.resetCurrentRevolutionsToZero();
    check(m.isRevolutionsValid()&&m.getCurrentRevolutions()==0,"explicit fresh re-anchor restores displacement validity");
    nowMs=1100;check(!m.isState1Fresh()&&!m.isMotorStateKnown(),"state1 and motor state expire");
    for(uint16_t encoder: {uint16_t(0),uint16_t(32767),uint16_t(32768),uint16_t(34059),uint16_t(65535)}) {
        L a(1,nullptr);s=reply(0x9A);a.decodeCanRxMessageFromISR(&s);
        r=reply(0x9C,encoder);a.decodeCanRxMessageFromISR(&r);
        check(std::fabs(a.getCurrentRevolutions()-encoder/65536.f)<1e-5f,"state1 before first angle cannot invent wrap");
    }
    L wraps(1,nullptr);r=reply(0x9C,65530);wraps.decodeCanRxMessageFromISR(&r);
    nowMs++;r=reply(0x9C,5);wraps.decodeCanRxMessageFromISR(&r);
    check(std::fabs(wraps.getCurrentRevolutions()-(1+5/65536.f))<1e-5f,"forward crossing counts once");
    s=reply(0x9A);wraps.decodeCanRxMessageFromISR(&s);
    check(std::fabs(wraps.getCurrentRevolutions()-(1+5/65536.f))<1e-5f,"state1 does not change turns");
    nowMs++;r=reply(0x9C,65530);wraps.decodeCanRxMessageFromISR(&r);
    check(std::fabs(wraps.getCurrentRevolutions()-65530/65536.f)<1e-5f,"reverse crossing counts once");
    L queued(1,nullptr);Motor& qb=queued;
    can_rx_message_t batch[]={reply(0x9C,65530),reply(0x9A),reply(0x9C,5)};
    for(auto& frame:batch){frame.receivedAtValid=1;frame.receivedAtMs=nowMs;}
    check(qb.decodeCanRxMessageFromQueue(batch,3)&&queued.isState1Fresh()&&queued.getCurrentRevolutions()>1,"queue preserves order and both reply kinds through base API");
    check(!qb.decodeCanRxMessageFromQueue(nullptr,1),"null queue rejected");
    L backlog(1,nullptr);auto old=reply(0x9C,100);
    old.receivedAtValid=1;old.receivedAtMs=nowMs-100;
    check(backlog.decodeCanRxMessageFromQueue(&old,1)&&!backlog.isMotorConected()&&!backlog.isState2Fresh(),"queued old feedback retains receive age");
    auto fresh=reply(0x9C,200);fresh.receivedAtValid=1;fresh.receivedAtMs=nowMs;
    check(backlog.decodeCanRxMessageFromQueue(&fresh,1)&&backlog.isMotorConected()&&!backlog.isRevolutionsValid(),"receive-time gap invalidates queued turn history");
    check(!backlog.decodeCanRxMessageFromQueue(&old,1)&&backlog.isState2Fresh(),"out-of-order queued frame cannot overwrite current state");
    fresh.receivedAtValid=0;
    check(!backlog.decodeCanRxMessageFromQueue(&fresh,1),"queue without receive timestamp fails closed");
    fresh.receivedAtValid=1;fresh.receivedAtMs=nowMs+1;
    check(!backlog.decodeCanRxMessageFromQueue(&fresh,1),"future receive timestamp rejected");
    L zeroTick(1,nullptr);nowMs=0;fresh.receivedAtMs=0;
    check(zeroTick.decodeCanRxMessageFromQueue(&fresh,1)&&zeroTick.isState2Fresh(),"timestamp zero is valid with explicit flag");
    L rollover(1,nullptr);nowMs=0xfffffff0;r=reply(0x9C);rollover.decodeCanRxMessageFromISR(&r);
    nowMs=20;check(rollover.isMotorConected(),"tick wrap remains fresh");
    nowMs=84;check(!rollover.isMotorConected(),"tick wrap timeout boundary");
    for(int kind=0;kind<5;kind++) {
        auto bad=reply(0x9C);
        if(kind==0)bad.header.DLC=7;
        if(kind==1)bad.header.IDE=CAN_ID_EXT;
        if(kind==2)bad.header.RTR=CAN_RTR_REMOTE;
        if(kind==3)bad.header.StdId=0x181;
        if(kind==4)bad.data[0]=0xff;
        check(!rollover.decodeCanRxMessageFromISR(&bad)&&!rollover.isMotorConected(),"invalid frame cannot refresh connection");
    }
    L a(1,nullptr),b(2,nullptr),c(3,nullptr),d(4,nullptr),dup(1,nullptr),invalid(0,nullptr);
    a.setMotorCommand(L::READ_STATE_1);b.setMotorCommand(L::READ_STATE_2);
    c.setMotorCommand(L::MOTOR_OFF);d.setMotorCommand(L::MOTOR_STOP);
    L* group[]={&a,&b,&c,&d};CAN_TxHeaderTypeDef h{};uint8_t data[8];
    check(L::buildControlFrame(group,4,15,h,data)&&h.StdId==0x288&&h.DLC==8&&data[0]==0x9a&&data[2]==0x9c&&data[4]==0x80&&data[6]==0x81,"four ID mixed commands serialize in little-endian slots");
    check(L::buildControlFrame(group,1,1,h,data)&&data[2]==0&&data[6]==0,"single declared node leaves only absent slots zero");
    a.hardwareAngleClosedloopControl(.1f);b.hardwareAngleClosedloopControl(.2f);
    check(!L::buildControlFrame(group,1,3,h,data)&&h.StdId==0&&data[0]==0,"missing present node cannot create zero-position command");
    check(L::buildControlFrame(group,2,3,h,data)&&h.StdId==0x282,"complete declared position group accepted");
    b.setMotorCommand(L::READ_STATE_2);
    check(a.getMergedControlData(b)==nullptr&&!a.isLastMergeComplete(),"legacy merge fails closed on mode mismatch");
    check(!L::buildControlFrame(group,2,3,h,data)&&data[0]==0,"checked frame rejects mismatch before publishing");
    group[1]=&dup;check(!L::buildControlFrame(group,2,3,h,data),"duplicate IDs rejected");
    group[1]=nullptr;check(!L::buildControlFrame(group,2,3,h,data),"null node rejected");
    group[0]=&invalid;check(!L::buildControlFrame(group,1,1,h,data),"invalid ID rejected");
    check(!L::buildControlFrame(nullptr,1,1,h,data)&&!L::buildControlFrame(group,0,1,h,data)&&!L::buildControlFrame(group,5,1,h,data),"invalid array sizes rejected");
    group[0]=&a;check(!L::buildControlFrame(group,1,0,h,data)&&!L::buildControlFrame(group,1,16,h,data),"invalid active mask rejected");
    group[0]=&a;group[1]=&b;group[2]=&c;group[3]=&d;
    a.openloopControl(100);b.openloopControl(0);c.openloopControl(-100);d.openloopControl(0);
    const uint8_t torqueExample[]={0x64,0,0,0,0x9c,0xff,0,0};
    check(L::buildControlFrame(group,4,15,h,data)&&h.StdId==0x280&&!std::memcmp(data,torqueExample,8),"official torque byte example through openloop API");
    a.hardwareAngularVelocityClosedloopControl(0);b.hardwareAngularVelocityClosedloopControl(2*MATH_PI);
    c.hardwareAngularVelocityClosedloopControl(0);d.hardwareAngularVelocityClosedloopControl(-4*MATH_PI);
    const uint8_t speedExample[]={0,0,0x68,1,0,0,0x30,0xfd};
    check(L::buildControlFrame(group,4,15,h,data)&&h.StdId==0x281&&!std::memcmp(data,speedExample,8),"official speed byte example at gearbox one");
    a.hardwareAngleClosedloopControl(MATH_PI);b.hardwareAngleClosedloopControl(0);
    c.hardwareAngleClosedloopControl(0);d.hardwareAngleClosedloopControl(-MATH_PI/2);
    const uint8_t angleExample[]={0x50,0x46,0,0,0,0,0xd8,0xdc};
    check(L::buildControlFrame(group,4,15,h,data)&&h.StdId==0x282&&!std::memcmp(data,angleExample,8),"official position byte example at gearbox one");
    a.openloopControl(5000);b.openloopControl(-5000);c.openloopControl(NAN);d.openloopControl(0);
    const uint8_t saturated[]={0xd0,7,0x30,0xf8,0,0,0,0};
    check(L::buildControlFrame(group,4,15,h,data)&&!std::memcmp(data,saturated,8),"torque saturation and NaN zero encoding");
    for(auto motor:group)motor->openloopControl(0);
    const uint8_t zero[8]{};
    check(L::buildControlFrame(group,1,1,h,data)&&h.StdId==0x280&&!std::memcmp(data,zero,8),"single active node zero probe produces only zero payload");
    // Issue66: all four targets are distinct. Array order must not become motor ID order.
    int order[]={0,1,2,3};
    const uint8_t fourSigned[]={0x64,0x00,0x38,0xff,0x2c,0x01,0x70,0xfe};
    const uint8_t fourSpeeds[]={1,0,0xfe,0xff,3,0,0xfc,0xff};
    const uint8_t fourCommands[]={0x9a,0,0x9c,0,0x80,0,0x81,0};
    do {
        L* shuffled[]={group[order[0]],group[order[1]],group[order[2]],group[order[3]]};
        const float signedValues[]={100,-200,300,-400};
        const float degrees[]={1,-2,3,-4};
        for(int i=0;i<4;i++)group[i]->openloopControl(signedValues[i]);
        check(L::buildControlFrame(shuffled,4,15,h,data)&&h.StdId==0x280&&!std::memcmp(data,fourSigned,8),"four torque slots independent of all 24 object permutations");
        for(int i=0;i<4;i++)group[i]->hardwareAngularVelocityClosedloopControl(degrees[i]*MATH_PI/180);
        check(L::buildControlFrame(shuffled,4,15,h,data)&&h.StdId==0x281&&!std::memcmp(data,fourSpeeds,8),"four speed slots independent of object order");
        for(int i=0;i<4;i++)group[i]->hardwareAngleClosedloopControl(degrees[i]*MATH_PI/180);
        check(L::buildControlFrame(shuffled,4,15,h,data)&&h.StdId==0x282&&!std::memcmp(data,fourSigned,8),"four position slots independent of object order");
        a.setMotorCommand(L::READ_STATE_1);b.setMotorCommand(L::READ_STATE_2);
        c.setMotorCommand(L::MOTOR_OFF);d.setMotorCommand(L::MOTOR_STOP);
        check(L::buildControlFrame(shuffled,4,15,h,data)&&h.StdId==0x288&&!std::memcmp(data,fourCommands,8),"four mixed commands independent of object order");
    } while(std::next_permutation(order,order+4));
    nowMs=10000;
    L f1(1,nullptr),f2(2,nullptr),f3(3,nullptr),f4(4,nullptr);
    L* feedback[]={&f1,&f2,&f3,&f4};
    for(uint8_t id:{3,1,4,2}) {
        auto frame=reply(0xA1,id*1000,id);
        for(int i=0;i<4;i++)check(feedback[i]->decodeCanRxMessageFromISR(&frame)==(i+1==id),"each A1 reply accepted only by its own motor ID");
    }
    for(int i=0;i<4;i++)check(std::fabs(feedback[i]->getCurrentRevolutions()-(i+1)*1000.f/65536)<1e-5f,"interleaved replies retain four distinct positions");
    auto fault=reply(0x9a,0,3);fault.data[7]=0x80;
    for(auto motor:feedback)motor->decodeCanRxMessageFromISR(&fault);
    check(f3.getErrorState()==0x80&&f3.isState1Fresh()&&!f1.isState1Fresh()&&!f2.isState1Fresh()&&!f4.isState1Fresh(),"motor3 fault does not contaminate other motors status");
    nowMs=10090;auto onlyOne=reply(0xA1,1001,1);f1.decodeCanRxMessageFromISR(&onlyOne);
    nowMs=10100;
    for(int i=0;i<4;i++)check(feedback[i]->isMotorConected()==(i==0),"one active motor cannot keep three silent motors online");
    a.openloopControl(100);c.openloopControl(-100);
    L* sparse[]={&c,&a};
    check(L::buildControlFrame(sparse,2,5,h,data)&&!std::memcmp(data,torqueExample,8),"noncontiguous IDs 1 and 3 retain correct sparse slots");
    L ms(2,nullptr,0,1,L::FEEDBACK_BASE_0x140,L::ENCODER_16BIT,L::MS);
    check(a.getMotorFamily()==L::MF_MG&&ms.getMotorFamily()==L::MS,"existing constructors retain MF/MG; MS explicitly selected");
    for(float input:{850.f,851.f,INFINITY,-850.f,-851.f,-INFINITY,NAN,0.49f,-0.49f,0.5f,-0.5f}) {
        ms.openloopControl(input);
        int expected=std::isnan(input)?0:input>=850?850:input<=-850?-850:int(std::round(input));
        const auto bytes=ms.getMotorControlData();
        check(ms.getMotorControlHeader()->StdId==0x280&&
              int16_t(uint16_t(bytes[2])|(uint16_t(bytes[3])<<8))==expected&&
              bytes[0]==0&&bytes[1]==0&&bytes[4]==0&&bytes[5]==0&&bytes[6]==0&&bytes[7]==0,
              "MS voltage clamp/rounding/nonfinite handling preserves its ID2 slot");
    }
    a.openloopControl(2500);ms.openloopControl(-1000);
    L* mixedFamily[]={&ms,&a};
    const uint8_t mixedExpected[]={0xd0,0x07,0xae,0xfc,0,0,0,0};
    check(L::buildControlFrame(mixedFamily,2,3,h,data)&&!std::memcmp(data,mixedExpected,8),"MF/MG and MS share 0x280 with independent physical meanings and clamps");
    for(uint8_t command:{0xA0,0x9C}) {
        for(int raw:{-1000,1000}) {
            auto power=reply(command,1234,2);power.data[2]=raw&255;power.data[3]=(uint16_t(raw)>>8);
            check(ms.decodeCanRxMessageFromISR(&power)&&ms.isState2Fresh()&&ms.getOutputFeedbackRaw()==raw,
                  "MS A0 and state2 retain signed raw power without ampere conversion");
        }
    }
    L invalidFamily(1,nullptr,0,1,L::FEEDBACK_BASE_0x140,L::ENCODER_16BIT,static_cast<L::LKMotorFamily>(255));
    L* invalidGroup[]={&invalidFamily};
    invalidFamily.openloopControl(100);auto invalidReply=reply(0xA1);
    check(!invalidFamily.isMotorFamilyValid()&&!L::buildControlFrame(invalidGroup,1,1,h,data)&&h.DLC==0,
          "invalid family rejects atomic frame build");
    check(!invalidFamily.decodeCanRxMessageFromISR(&invalidReply)&&!invalidFamily.isMotorConected(),"invalid family cannot become online");
    check(invalidFamily.getMergedControlData(ms)==nullptr&&ms.getMergedControlData(invalidFamily)==nullptr,"invalid family rejected on both legacy merge sides");
    for(auto resolution:{L::ENCODER_14BIT,L::ENCODER_15BIT,L::ENCODER_16BIT}) {
        L variant(4,nullptr,7,1,L::FEEDBACK_BASE_0x180,resolution,L::MS);
        auto frame=reply(0x9C,uint16_t((1u<<resolution)/2+7),4);frame.header.StdId=0x184;
        check(variant.decodeCanRxMessageFromISR(&frame)&&std::fabs(variant.getCurrentAngle()-MATH_PI)<1e-5f,
              "explicit 0x180 base and each encoder resolution preserve offset-adjusted half-turn");
        frame.header.StdId=0x144;
        check(!variant.decodeCanRxMessageFromISR(&frame),"configured alternate feedback base rejects default-base ID");
    }
    for(uint8_t command:{0x80,0x81,0x88,0x9A,0x9B,0x9C}) {
        ms.setMotorCommand(static_cast<L::LKBroadcastCommand>(command));
        auto bytes=ms.getMotorControlData();
        check(ms.getMotorControlHeader()->StdId==0x288&&bytes[2]==command&&bytes[3]==0,
              "all six documented mixed commands serialize for MS");
    }
    // Avoid exact half-LSB targets here: radian conversion in float may land on either side.
    for(float raw:{-INFINITY,-40000.f,-32768.f,-32767.75f,-32767.f,32766.75f,32767.f,40000.f,INFINITY,NAN}) {
        int expected=std::isnan(raw)?0:raw>=32767?32767:raw<=-32768?-32768:int(std::round(raw));
        a.hardwareAngularVelocityClosedloopControl(raw*MATH_PI/180.f);
        auto bytes=a.getMotorControlData();
        check(int16_t(uint16_t(bytes[0])|(uint16_t(bytes[1])<<8))==expected,"speed full int16 boundaries do not wrap");
        a.hardwareAngleClosedloopControl(raw*MATH_PI/18000.f);
        bytes=a.getMotorControlData();
        check(int16_t(uint16_t(bytes[0])|(uint16_t(bytes[1])<<8))==expected,"position full int16 boundaries do not wrap");
    }
    std::cout << checks << " host checks passed; no hardware or motion validation.\n";
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    header = (ROOT/'GSRL/Device/inc/dvc_motor.hpp').read_text(encoding='utf-8-sig')
    cpp = (ROOT/'GSRL/Device/src/dvc_motor.cpp').read_text(encoding='utf-8-sig')
    start = header.index('class Motor\n')
    base = header[start:header.index('\n/**', start)]
    start = header.index('class MotorLKBroadcast :')
    lk = header[start:header.index('\n/* Exported constants', start)]
    signatures = [
        'const uint8_t *Motor::getMotorControlData()',
        'const CAN_TxHeaderTypeDef *Motor::getMotorControlHeader() const',
        'bool Motor::decodeCanRxMessageFromISR(', 'bool Motor::decodeCanRxMessageFromQueue(',
        'bool Motor::isMotorConected() const', 'fp32 Motor::getCurrentAngle() const',
        'fp32 Motor::getCurrentAngularVelocity() const', 'fp32 Motor::getCurrentRevolutions() const',
        'int16_t Motor::getCurrentTorqueCurrent() const', 'int8_t Motor::getTemperature() const',
        'void Motor::resetCurrentRevolutionsToZero()', 'Motor::Motor(',
        'fp32 Motor::updateCurrentRevolutions()', 'inline void Motor::increaseMotorFeedbackErrorCount()',
        'inline void Motor::clearMotorFeedbackErrorCount()',
        'void Motor::openloopControl(',
    ]
    unit = PRELUDE + base + lk + '\n'.join(method(cpp, sig) for sig in signatures)
    unit += cpp[cpp.index('MotorLKBroadcast::MotorLKBroadcast('):] + TESTS
    source = out/'lk_broadcast_test.cpp'
    source.write_text(unit, encoding='utf-8')
    executable = out/('lk_broadcast_test.exe' if os.name == 'nt' else 'lk_broadcast_test')
    for command, log in [([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2', '-Wall', '-Wextra', str(source), '-o', str(executable)], 'build.log'),
                         ([str(executable)], 'results.log')]:
        result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace')
        (out/log).write_text(result.stdout+result.stderr, encoding='utf-8')
        print(result.stdout+result.stderr, end='')
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
