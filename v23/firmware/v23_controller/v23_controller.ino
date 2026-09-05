// OpenCR + two AS5047P sensors + XM430. No motion or zeroing on connection.
// The shared core generates the reference; the servo's second profile is disabled.
#include <SPI.h>
#include <Dynamixel2Arduino.h>
#include "control_core.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
extern "C" int32_t CDC_Itf_Write(uint8_t*,uint32_t);

using namespace ControlTableItem;
Dynamixel2Arduino dxl(Serial3,84);
v23::Core core;
v23::StartGate gate;
constexpr uint8_t MOTOR_ID=1, PHI_CS=10, ANK_CS=9;
constexpr double PHI_SIGN=1, ANK_SIGN=1, MOTOR_SIGN=1;
constexpr double TICKS=4096.0/360.0;
bool motor=false, torque=false, calibrated=false, dry=true;
int hang_stage=0;double hp0=0,ha0=0,hd0=0;
double phi_zero=0,ank_zero=0,home=0,phi_offset=0,ank_offset=0;
uint16_t pr=0,ar=0; int32_t position=0,velocity=0;
v23::Sample sample={0,0,0,0,0};
uint32_t next_us=0,last_sample=0,seq=0,drops=0; int err=0;
char rx[96]; unsigned rxn=0; bool overflow=false;
// Buffered USB writes cannot block the control loop; seq exposes dropped D rows.
char tx[16][640]; uint16_t txlen[16]={}; unsigned th=0,tt=0,used=0;
void enqueue(const char* s) {
  if(used==16) { ++drops; return; }
  txlen[th]=snprintf(tx[th],sizeof(tx[th]),"%s\n",s);
  if(txlen[th]>=sizeof(tx[th]))txlen[th]=sizeof(tx[th])-1;
  th=(th+1)%16; ++used;
}
void flushTx() {
  if(!used || !Serial) return;
  // OpenCR 1.5.3 Serial.write retries for up to 100ms. This driver call does not.
  int32_t sent=CDC_Itf_Write((uint8_t*)tx[tt],txlen[tt]);
  if(sent<=0)return;
  memmove(tx[tt],tx[tt]+sent,txlen[tt]-sent);txlen[tt]-=sent;
  if(!txlen[tt]) { tt=(tt+1)%16;--used; }
}
void event(const char* name) {
  char b[120];snprintf(b,sizeof(b),"E,%lu,%s,%lu,%.8f",(unsigned long)millis(),name,(unsigned long)seq,sample.delta);enqueue(b);
}
void metadata() {
  char b[630];const auto& c=core.cfg;
  snprintf(b,sizeof(b),"# CFG {\"schema\":1,\"p2r\":%.12g,\"wphi\":%.12g,\"wphid\":%.12g,\"wbetad\":%.12g,\"offset\":%.9g,\"gain\":%.9g,\"trigger\":%.9g,\"relax\":%.9g,\"return_speed\":%.9g,\"dead\":%.9g,\"limit\":%.9g,\"step_cap\":%.9g,\"vmax\":%.9g,\"amax\":%.9g,\"tolerance\":%.9g,\"rest\":%.9g,\"tau\":%.9g,\"phi_zero\":%.4f,\"ank_zero\":%.4f,\"home\":%.4f,\"phi_offset\":%.3f,\"ank_offset\":%.3f}",
    c.p2r,c.wphi,c.wphid,c.wbetad,c.offset,c.gain,c.trigger,c.relax,c.return_speed,c.dead,c.limit,c.step_cap,c.vmax,c.amax,c.tolerance,c.rest,c.tau,phi_zero,ank_zero,home,phi_offset,ank_offset);
  enqueue(b);
}
void header() {
  enqueue("# D,seq,t_ms,phi,ank,del_now,alpha,beta,dphi,dbeta,Ahat,hold,ref,vref,phase,start,event,err,dt_us,tx_drop,phi_raw,ank_raw,dxl_raw,dry");
  metadata();
}
double wrap(double x) { while(x>=180)x-=360;while(x< -180)x+=360;return x; }
bool encoder(uint8_t cs,uint16_t& angle) {
  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE1));
  digitalWrite(cs,LOW);delayMicroseconds(1);SPI.transfer16(0xFFFF);
  digitalWrite(cs,HIGH);delayMicroseconds(1);digitalWrite(cs,LOW);delayMicroseconds(1);
  uint16_t r=SPI.transfer16(0xFFFF);digitalWrite(cs,HIGH);SPI.endTransaction();
  unsigned parity=0;for(unsigned k=0;k<16;++k)parity^=(r>>k)&1;
  angle=r&0x3FFF;return parity==0 && !(r&0x4000);
}
bool readSensors() {
  err=0;if(!encoder(PHI_CS,pr))err|=1;if(!encoder(ANK_CS,ar))err|=2;
  if(motor) {
    position=dxl.readControlTableItem((uint8_t)PRESENT_POSITION,MOTOR_ID,(uint32_t)2);
    if(dxl.getLastLibErrCode()!=0 || dxl.getLastStatusPacketError()!=0)err|=4;
    velocity=dxl.readControlTableItem((uint8_t)PRESENT_VELOCITY,MOTOR_ID,(uint32_t)2);
    if(dxl.getLastLibErrCode()!=0 || dxl.getLastStatusPacketError()!=0)err|=4;
  } else err|=4;
  sample.phi=wrap(PHI_SIGN*wrap(pr*360.0/16384-phi_zero)+phi_offset);
  sample.ank=wrap(ANK_SIGN*wrap(ar*360.0/16384-ank_zero)+ank_offset);
  sample.delta=sample.local_delta=MOTOR_SIGN*(position-home)/TICKS;
  sample.local_velocity=MOTOR_SIGN*velocity*1.374;
  return !err;
}
void stop(bool off) {
  gate.stop();core.reset(sample.delta);
  if(off) { if(motor)dxl.torqueOff(MOTOR_ID);torque=false; }
  else if(motor && torque) dxl.setGoalPosition(MOTOR_ID,position);
  event(off?"TORQUE_OFF":"STOP");
}
void fault(const char* reason) {
  if(gate.state==v23::FAULT)return;
  stop(true);gate.fault();event(reason);
}
void cue() {
#ifdef BDPIN_BUZZER
  tone(BDPIN_BUZZER,2400,100);
#endif
  event("GO_CUE");
}
bool active() {return gate.state==v23::RUNNING || gate.state==v23::COUNTDOWN;}
void command(char* line) {
  if(!strcmp(line,"x")) {stop(true);return;}
  if(!strcmp(line,"stop")) {stop(false);return;}
  if(!strcmp(line,"hdr")) {header();return;}
  if(!strcmp(line,"go")) {if(!gate.request())event("NOT_READY");else event("COUNTDOWN");return;}
  if(active()) {event("BUSY_STOP_FIRST");return;}
  if(!strcmp(line,"zero")) {
    if(torque || !readSensors()) {event("ZERO_REJECTED");return;}
    phi_zero=pr*360.0/16384;ank_zero=ar*360.0/16384;home=position;
    phi_offset=ank_offset=0;calibrated=true;core.reset();gate.stop();header();event("ZERO_UPRIGHT");return;
  }
  if(!strcmp(line,"zero_hang")) {
    if(torque || !readSensors()) {event("ZERO_REJECTED");return;}
    double p=pr*360.0/16384,a=ar*360.0/16384;
    if(hang_stage==0) {hp0=p;ha0=a;hd0=position;hang_stage=1;event("HANG_FIRST");return;}
    phi_zero=hp0+wrap(p-hp0)/2;ank_zero=ha0+wrap(a-ha0)/2;home=(hd0+position)/2;
    phi_offset=0;ank_offset=180;hang_stage=0;calibrated=true;core.reset();gate.stop();header();event("ZERO_HANG");return;
  }
  if(!strcmp(line,"prepare")) {
    if(!calibrated || !readSensors() || fabs(sample.delta)>core.cfg.limit) {event("PREPARE_REJECTED");return;}
    core.reset(sample.delta);
    if(!dry) {
      if(!dxl.setGoalPosition(MOTOR_ID,position) || !dxl.torqueOn(MOTOR_ID)) {fault("TORQUE_ERROR");return;}
      torque=true;
    }
    gate.prepare();metadata();event("PREPARE");return;
  }
  char key[24];double val;char tail;
  if(sscanf(line,"%23s %lf %c",key,&val,&tail)!=2 || !isfinite(val)) {event("BAD_COMMAND");return;}
  if(!strcmp(key,"dry") && (val==0 || val==1)) {stop(true);dry=val!=0;event(dry?"DRY_ON":"DRY_OFF");return;}
  auto& c=core.cfg;bool ok=true;
  if(!strcmp(key,"gain") && val>=.5 && val<=30)c.gain=val;
  else if(!strcmp(key,"p2r") && val>0 && val<1)c.p2r=val;
  else if(!strcmp(key,"wphi") && val>0 && val<10)c.wphi=val;
  else if(!strcmp(key,"wphid") && val>=0 && val<2)c.wphid=val;
  else if(!strcmp(key,"wbetad") && val>=0 && val<2)c.wbetad=val;
  else if(!strcmp(key,"offset") && fabs(val)<=5)c.offset=val;
  else if(!strcmp(key,"trigger") && val>=.1 && val<=3)c.trigger=val;
  else if(!strcmp(key,"relax") && val>=0 && val<c.trigger)c.relax=val;
  else if(!strcmp(key,"rest") && val>=.03 && val<=.3)c.rest=val;
  else if(!strcmp(key,"return_speed") && val>=0 && val<=6)c.return_speed=val;
  else if(!strcmp(key,"phi_offset") && fabs(val)<=180)phi_offset=val;
  else if(!strcmp(key,"ank_offset") && fabs(val)<=180)ank_offset=val;
  else ok=false;
  if(ok) {core.reset(sample.delta);gate.stop();metadata();event("CONFIG");}else event("BAD_PARAMETER");
}
void serialPoll() {
  // Bound parsing work, including malformed input. Emergency x is a whole command.
  for(int n=0;n<96 && Serial.available();++n) {
    char c=Serial.read();
    if(c=='\n' || c=='\r') {if(!overflow && rxn){rx[rxn]=0;command(rx);}rxn=0;overflow=false;}
    else if(rxn<sizeof(rx)-1)rx[rxn++]=c;else overflow=true;
  }
}
void setup() {
  Serial.begin(1000000);
  pinMode(PHI_CS,OUTPUT);pinMode(ANK_CS,OUTPUT);digitalWrite(PHI_CS,HIGH);digitalWrite(ANK_CS,HIGH);SPI.begin();
#ifdef BDPIN_DXL_PWR_EN
  pinMode(BDPIN_DXL_PWR_EN,OUTPUT);digitalWrite(BDPIN_DXL_PWR_EN,HIGH);delay(300);
#endif
  dxl.begin(1000000);dxl.setPortProtocolVersion(2.0);motor=dxl.ping(MOTOR_ID);
  if(motor) {
    motor=dxl.torqueOff(MOTOR_ID) && dxl.setOperatingMode(MOTOR_ID,OP_EXTENDED_POSITION)
      && dxl.writeControlTableItem(RETURN_DELAY_TIME,MOTOR_ID,0)
      && dxl.writeControlTableItem(PROFILE_VELOCITY,MOTOR_ID,0)
      && dxl.writeControlTableItem(PROFILE_ACCELERATION,MOTOR_ID,0)
      && dxl.writeControlTableItem(CURRENT_LIMIT,MOTOR_ID,350);
  }
  enqueue("# v23 0.1: DRY by default; explicit zero/prepare/go; 1Mbps DXL required");header();
  next_us=micros();last_sample=next_us;
}
void loop() {
  serialPoll();flushTx();
  uint32_t now=micros();if((int32_t)(now-next_us)<0)return;
  uint32_t dt=now-last_sample;last_sample=now;
  if((int32_t)(now-next_us)>=5000) {
    if(active())fault("DEADLINE");
    core.reset(sample.delta);event("ESTIMATOR_RESET");next_us=now;
  }
  next_us+=5000;
  bool valid=readSensors();
  core.measure(sample);
  if((torque && !valid) || (active() && (!valid || fabs(sample.phi)>30 || fabs(core.out.alpha)>30)))fault("SENSOR_OR_ANGLE");
  const bool steady=valid && fabs(core.out.A)<core.cfg.trigger && fabs(core.out.dphi)<2
     && fabs(core.out.dbeta)<2 && fabs(sample.local_velocity)<2 && fabs(sample.phi)<8 && fabs(core.out.alpha)<12;
  int prior=gate.state;
  if(gate.state!=v23::DISARMED && gate.state!=v23::FAULT)gate.tick(valid,steady);
  if(gate.state==v23::READY && prior!=gate.state)event("READY");
  if(prior==v23::COUNTDOWN && gate.state==v23::PREPARING)event("COUNTDOWN_CANCELLED");
  if(gate.cue)cue();
  core.advance(sample,gate.state==v23::RUNNING);
  if(core.out.event==v23::FOLD_START)event("FOLD");
  if(core.out.event==v23::ARRIVED)event("ARRIVED");
  if(core.out.event==v23::REST_END)event("REST_END");
  if(core.out.event==v23::FOLD_TIMEOUT)fault("FOLD_TIMEOUT");
  if(gate.state==v23::RUNNING && !dry) {
    if(!dxl.setGoalPosition(MOTOR_ID,home+MOTOR_SIGN*core.out.ref*TICKS))fault("WRITE_ERROR");
  }
  char b[630];const auto& o=core.out;
  snprintf(b,sizeof(b),"D,%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.8f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%lu,%lu,%u,%u,%ld,%d",
    (unsigned long)seq++,(unsigned long)millis(),sample.phi,sample.ank,sample.delta,o.alpha,o.beta,o.dphi,o.dbeta,o.A,o.hold,o.ref,o.vref,o.phase,gate.state,o.event,err,(unsigned long)dt,(unsigned long)drops,pr,ar,(long)position,dry?1:0);
  enqueue(b);
}
