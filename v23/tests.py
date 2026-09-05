"""Raw replay contract and startup guards, without a USB robot."""
import asyncio
import json
import math
import unittest

from core import Core,DEFAULT
from server import Recording,Hub

HEADER='seq,t_ms,phi,ank,del_now,Ahat,hold,start'

class ReplayTests(unittest.TestCase):
    def test_same_raw_stream_replays_after_prepare(self):
        rec=Recording(); c=Core()
        rec.line('# D,'+HEADER);rec.line('# CFG '+json.dumps(DEFAULT));rec.line('E,0,PREPARE,0')
        for i in range(160):
            phi=.1*math.sin(i*.03);ank=.4+phi;delta=.1*math.sin(i*.05)
            out=c.tick(phi,ank,delta,enabled=i>=80)
            rec.line(f'D,{i},{i*5},{phi:.6f},{ank:.6f},{delta:.6f},{out["Ahat"]:.8f},{out["hold"]:.6f},{4 if i>=80 else 1}')
            # A repeated status header must not reset the estimator.
            if i==100:rec.line('# CFG '+json.dumps(DEFAULT))
        self.assertTrue(all(r['replay_valid'] for r in rec.data['rows']))
        self.assertLess(max(r['error_Ahat'] for r in rec.data['rows']),1e-4)
        self.assertLess(max(r['error_hold'] for r in rec.data['rows']),1e-3)
        c.close();rec.close()

    def test_gap_never_claims_parity(self):
        rec=Recording();rec.line('# D,'+HEADER);rec.line('# CFG '+json.dumps(DEFAULT));rec.line('E,0,PREPARE,0')
        rec.line('D,10,50,0,0,0,0,0,1');rec.line('D,12,60,0,0,0,0,0,1')
        self.assertEqual(rec.data['rows'][-1]['replay_valid'],0)
        self.assertIsNone(rec.data['rows'][-1]['error_Ahat']);rec.close()

    def test_no_config_is_not_an_exact_replay(self):
        rec=Recording();rec.line('# D,'+HEADER);rec.line('D,0,0,0,0,0,0,0,1')
        self.assertEqual(rec.data['rows'][-1]['replay_valid'],0);rec.close()

    def test_malformed_input(self):
        rec=Recording();rec.line('# D,'+HEADER)
        with self.assertRaises(ValueError):rec.line('D,0,0,nan,0,0,0,0,1')
        with self.assertRaises(ValueError):rec.line('D,0,0,0')
        self.assertEqual(rec.data['rows'],[]);rec.close()

    def test_start_gate_cpp(self):
        c=Core();self.assertEqual(c.lib.gate_check(),0);c.close()

    def test_fault_and_dropped_events_invalidate_replay(self):
        rec=Recording();rec.line('# D,'+HEADER+',tx_drop')
        rec.line('# CFG '+json.dumps(DEFAULT));rec.line('E,0,PREPARE,0,1.25')
        rec.line('D,0,0,0,0,1.25,0,1.25,1,0')
        self.assertEqual(rec.data['rows'][-1]['replay_hold'],1.25)
        rec.line('E,5,SENSOR_OR_ANGLE,1,1.25')
        rec.line('D,1,5,0,0,1.25,0,1.25,5,0')
        self.assertEqual(rec.data['rows'][-1]['replay_valid'],0)
        rec.line('E,10,PREPARE,2,1.25');rec.line('D,2,10,0,0,1.25,0,1.25,1,1')
        self.assertEqual(rec.data['rows'][-1]['replay_valid'],0);rec.close()

if __name__=='__main__':unittest.main()
