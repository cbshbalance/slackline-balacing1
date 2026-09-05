#pragma once
#include <math.h>
#include <stdint.h>

namespace v23 {
#ifdef V23_FLOAT
using Real = float;
#else
using Real = double;
#endif
inline Real clip(Real x, Real lo, Real hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline Real sign(Real x) { return (x > 0) - (x < 0); }
constexpr Real DT = 0.005;
constexpr Real RAD = 0.017453292519943295;
enum Phase { IDLE = 0, FOLD = 1, REST = 4 };
enum Event { NONE = 0, FOLD_START = 1, ARRIVED = 2, FOLD_TIMEOUT = 3, REST_END = 4 };

struct Config {
  Real p2r = 0.412217905378982, wphi = 0.607864411665610;
  Real wphid = 0.102707995607934, wbetad = 0.168864153239234;
  Real offset = 0, gain = 9.5, trigger = 0.6, relax = 0.3;
  Real return_speed = 3, dead = 1, limit = 55, step_cap = 0;
  Real vmax = 420, amax = 8000, tolerance = 0.5, rest = 0.060;
  Real tau = 0.030;
};

struct Sample { Real phi, ank, delta, local_delta, local_velocity; };
struct Output {
  Real alpha = 0, beta = 0, dphi = 0, dbeta = 0, A = 0;
  Real hold = 0, ref = 0, vref = 0, elapsed = 0;
  int phase = IDLE, event = NONE, folds = 0, saturations = 0;
};

// Degrees throughout. Buffer startup and update order match v21 enc_update.
class Core {
 public:
  Config cfg;
  Output out;
  Real hp[6] = {}, hb[6] = {};
  int head = 0, count = 0;
  explicit Core(Config c = Config()) : cfg(c) {}
  void reset(Real delta = 0) {
    out = Output(); out.hold = out.ref = delta;
    head = count = 0;
  }
  void measure(const Sample& s) {
    out.alpha = s.ank - s.phi;
    out.beta = out.alpha + cfg.p2r * s.delta;
    hp[head] = s.phi; hb[head] = out.beta;
    if (count < 6) ++count;
    const int oldest = count < 6 ? 0 : (head + 1) % 6;
    Real dp = 0, db = 0;
    if (count > 1) {
      dp = (s.phi - hp[oldest]) / (DT * (count - 1));
      db = (out.beta - hb[oldest]) / (DT * (count - 1));
    }
    head = (head + 1) % 6;
    const Real a = DT / (cfg.tau + DT);
    out.dphi += a * (dp - out.dphi);
    out.dbeta += a * (db - out.dbeta);
    out.A = cfg.wphi * s.phi + out.beta + cfg.wphid * out.dphi + cfg.wbetad * out.dbeta + cfg.offset;
  }
  void advance(const Sample& s, bool enabled) {
    out.event = NONE;
    if (!enabled) return;
    if (out.phase == IDLE) {
      if (fabs(out.A) > cfg.trigger) {
        Real inc = cfg.gain * out.A;
        if (cfg.step_cap > 0) inc = clip(inc, -cfg.step_cap, cfg.step_cap);
        const Real target = out.hold + inc;
        out.hold = clip(target, -cfg.limit, cfg.limit);
        if (fabs(out.hold - target) > 1e-9 / RAD) ++out.saturations;
        ++out.folds; out.phase = FOLD; out.elapsed = 0; out.event = FOLD_START;
      } else if (cfg.return_speed > 0 && fabs(out.A) < cfg.relax && fabs(out.hold) > cfg.dead) {
        out.hold -= sign(out.hold) * cfg.return_speed * DT;
      }
    } else if (out.phase == FOLD) {
      const bool arrived = fabs(out.ref - out.hold) < 1e-9 / RAD && fabs(out.vref) < 1e-9 / RAD
                           && fabs(s.local_delta - out.hold) < cfg.tolerance;
      if (arrived || out.elapsed > 0.6) {
        out.phase = REST; out.elapsed = 0;
        out.event = arrived ? ARRIVED : FOLD_TIMEOUT;
      }
    } else if (out.phase == REST && out.elapsed >= cfg.rest) {
      out.phase = IDLE; out.elapsed = 0; out.event = REST_END;
    }
    out.elapsed += DT;
    const Real err = out.hold - out.ref;
    if (fabs(err) < 1e-9 / RAD && fabs(out.vref) < cfg.amax * DT) {
      out.ref = out.hold; out.vref = 0;
    } else {
      const Real stop = out.vref * out.vref / (2 * cfg.amax);
      if (out.vref * sign(err) > 0 && fabs(err) <= stop) out.vref -= sign(out.vref) * cfg.amax * DT;
      else out.vref = clip(out.vref + sign(err) * cfg.amax * DT, -cfg.vmax, cfg.vmax);
      out.ref += out.vref * DT;
      if ((out.hold - out.ref) * err < 0) { out.ref = out.hold; out.vref = 0; }
    }
  }
  Output tick(const Sample& s, bool enabled = true) { measure(s); advance(s, enabled); return out; }
};

enum StartState { DISARMED = 0, PREPARING = 1, READY = 2, COUNTDOWN = 3, RUNNING = 4, FAULT = 5 };
class StartGate {
 public:
  int state = DISARMED, ticks = 0, quiet = 0;
  bool cue = false;
  void prepare() { state = PREPARING; ticks = quiet = 0; cue = false; }
  bool request() { if (state != READY) return false; state = COUNTDOWN; ticks = 400; return true; }
  void stop() { state = DISARMED; ticks = quiet = 0; cue = false; }
  void fault() { state = FAULT; cue = false; }
  void tick(bool valid, bool steady) {
    cue = false;
    if (!valid) { fault(); return; }
    if (state == PREPARING || state == READY) {
      quiet = steady ? quiet + 1 : 0;
      state = quiet >= 100 ? READY : PREPARING;
    } else if (state == COUNTDOWN) {
      if (!steady) { state = PREPARING; quiet = 0; return; }
      if (--ticks <= 0) { state = RUNNING; cue = true; }
    }
  }
};
}
