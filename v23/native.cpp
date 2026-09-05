#include "firmware/v23_controller/control_core.h"
#ifdef _WIN32
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C"
#endif
API void* core_new() { return new v23::Core(); }
API void core_delete(void* p) { delete static_cast<v23::Core*>(p); }
API void core_reset(void* p, double d) { static_cast<v23::Core*>(p)->reset(d); }
API void core_config(void* p, const double* a) {
  auto& c = static_cast<v23::Core*>(p)->cfg;
  c.p2r=a[0]; c.wphi=a[1]; c.wphid=a[2]; c.wbetad=a[3]; c.offset=a[4];
  c.gain=a[5]; c.trigger=a[6]; c.relax=a[7]; c.return_speed=a[8]; c.dead=a[9];
  c.limit=a[10]; c.step_cap=a[11]; c.vmax=a[12]; c.amax=a[13]; c.tolerance=a[14]; c.rest=a[15]; c.tau=a[16];
}
API void core_tick(void* p, const double* a, int enabled, double* result) {
  v23::Sample s = {(v23::Real)a[0],(v23::Real)a[1],(v23::Real)a[2],(v23::Real)a[3],(v23::Real)a[4]};
  auto o = static_cast<v23::Core*>(p)->tick(s, enabled != 0);
  double v[] = {o.alpha,o.beta,o.dphi,o.dbeta,o.A,o.hold,o.ref,o.vref,o.elapsed,
                (double)o.phase,(double)o.event,(double)o.folds,(double)o.saturations};
  for (int i=0;i<13;++i) result[i]=v[i];
}
API int gate_check() {
  v23::StartGate g;
  if(g.request()) return 1;
  g.prepare(); for(int i=0;i<100;++i) g.tick(true,true);
  if(g.state!=v23::READY || !g.request()) return 2;
  g.tick(true,false); if(g.state!=v23::PREPARING) return 3;
  for(int i=0;i<100;++i) g.tick(true,true); g.request();
  for(int i=0;i<399;++i) g.tick(true,true);
  if(g.state==v23::RUNNING) return 4;
  g.tick(true,true); if(!g.cue || g.state!=v23::RUNNING) return 5;
  g.tick(false,true); if(g.state!=v23::FAULT) return 6;
  return 0;
}
