"""Run with Python 3, without a ROS master."""
from __future__ import annotations
import ast
from dataclasses import dataclass,replace
import importlib.util
import math
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parent
@dataclass
class PathPoint:
    x:float
    y:float
    z:float=0.
pkg=types.ModuleType('purepursuit_mgeo');pkg.__path__=[]
path=types.ModuleType('purepursuit_mgeo.path');path.PathPoint=PathPoint
sys.modules[pkg.__name__]=pkg;sys.modules[path.__name__]=path
spec=importlib.util.spec_from_file_location('clearance_planner',ROOT/'frenet_planner.py')
planner=importlib.util.module_from_spec(spec);sys.modules[spec.name]=planner;spec.loader.exec_module(planner)
tree=ast.parse((ROOT/'frenet_fsm_node.py').read_text())
nodes=[n for n in tree.body if isinstance(n,ast.ClassDef) or
       (isinstance(n,ast.FunctionDef) and n.name=='planned_lateral_acceleration')]
ns={'math':math,'replace':replace,'NORMAL':'NORMAL','RETURN':'RETURN','AVOIDANCE':'AVOIDANCE',
    'rospy':types.SimpleNamespace(logwarn=Mock())}
exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0)]+nodes,type_ignores=[])),'<actual FSM helpers>','exec'),ns)
Memory=ns['RecentThreatMemory'];Gate=ns['GpsFreshnessGate'];Node=ns['FrenetFsmNode']

def obstacle(track=1,x=20,y=0,length=2,width=1):
    return planner.TrackedObstacle(track,x,y,length,width,0,0,0,'STATIC',0,True,'BBOX')

def make_planner(tunnel=False):
    p=planner.FrenetPlanner(planner.ReferencePath([PathPoint(0,0),PathPoint(150,0)]),
        safety_margin=.3,maximum_lateral_acceleration=100,maximum_curvature=3,assume_static_obstacles=True)
    if tunnel:p.weight_offset=5.;p.weight_safety=40.;p.weight_clearance=100.;p.weight_completion=10.
    return p

OFFSETS=[1.,1.5,2.,2.5,3.,3.5,4.,4.5,5.,5.5,6.]

class Tests(unittest.TestCase):
    def test_wider_clearance_is_selected_in_tunnel(self):
        a=make_planner().plan(0,0,0,6,6,[obstacle()],0,OFFSETS,[2.,3.,4.])
        b=make_planner(True).plan(0,0,0,6,6,[obstacle()],0,OFFSETS,[2.,3.,4.])
        self.assertGreater(b.minimum_clearance,a.minimum_clearance)
        self.assertGreater(b.target_offset,a.target_offset)

    def test_wider_path_still_respects_walls(self):
        p=make_planner(True)
        objects=[obstacle(),obstacle(2,20,4.5,60,.2),obstacle(3,20,-4.5,60,.2)]
        c=p.plan(0,0,0,6,6,objects,0,OFFSETS,[2.,3.,4.])
        self.assertIsNotNone(c);self.assertLess(c.target_offset,4.)
        self.assertGreater(p.last_diagnostics['reasons'].get('collision',0),0)

    def test_only_threats_are_held_and_empty_frames_do_not_refresh_age(self):
        m=Memory(1.5);box=obstacle();wall=obstacle(2,20,5,20,.5)
        m.observe([box,wall],10);m.protect([1]);m.observe([],10.7)
        held,meta=m.supplement([],11.,True)
        self.assertEqual([o.track_id for o in held],[1]);self.assertEqual(meta,[(1,1.)])
        self.assertEqual(m.supplement([],11.51,True),([],[]))

    def test_redetection_updates_geometry_without_duplicate_same_id(self):
        m=Memory(1.5);m.observe([obstacle(width=.04)],10);m.protect([1])
        full=obstacle(width=1.4);m.observe([full],10.5)
        result,_=m.supplement([full],10.6,True)
        self.assertEqual(len(result),1);self.assertEqual(result[0].width,1.4)

    def test_planned_acceleration_continues_only_near_old_path(self):
        times=[i*.1 for i in range(11)]
        c=types.SimpleNamespace(points=[PathPoint(t,t*t) for t in times],times=times,
                                lateral_offsets=[t*t for t in times])
        self.assertAlmostEqual(ns['planned_lateral_acceleration'](c,(.5,.25,0,5),.946),2.)
        self.assertEqual(ns['planned_lateral_acceleration'](c,(20,0,0,5),.946),0.)

    def test_completion_cost_reduces_restarting_full_duration(self):
        p=make_planner(True)
        a=p.plan(0,0,0,6,6,[],0,[2.],[2.,3.,4.])
        b=p.plan(0,0,0,6,6,[],0,[2.],[2.,3.,4.],completion_budget=1.,continuation_direction=1)
        self.assertLess(b.planning_time,a.planning_time)

    def test_completion_preference_cannot_override_dynamics(self):
        p=make_planner(True);p.maximum_lateral_acceleration=1.
        c=p.plan(0,0,0,6,6,[],0,[2.],[2.,3.,4.],completion_budget=1.,continuation_direction=1)
        self.assertIsNotNone(c);self.assertEqual(c.planning_time,4.)

    def test_collision_is_still_a_hard_rejection(self):
        p=make_planner(True)
        self.assertIsNone(p.plan(0,0,0,6,6,[obstacle(x=1.35)],0,[-6.,6.],[2.,3.,4.]))

    def test_gps_gap_and_continuous_recovery(self):
        g=Gate(10);g.observe(10,10,0,0);self.assertFalse(g.observe(10.5,10,0,0))
        self.assertTrue(g.update(11.01));g.observe(12,12,0,0)
        self.assertTrue(g.update(12));g.observe(12.8,12.8,0,0)
        g.observe(13.1,13.1,0,0);self.assertFalse(g.update(13.1))

    def test_mode_changes_costs_without_changing_times_or_rate(self):
        n=Node.__new__(Node);n.lock=threading.RLock();n.tunnel_mode=False
        n.gps_gate=types.SimpleNamespace(update=lambda now:True);n.planner=make_planner()
        p=n.planner;n._normal_costs=(100.,4.,0.,1.,0.)
        n.tunnel_weight_offset=5.;n.tunnel_weight_safety=40.;n.tunnel_weight_clearance=100.
        n.tunnel_clearance=1.;n.tunnel_weight_completion=10.;n.threat_memory=Memory()
        n.planning_rate=2.;n.planning_times=[2.,3.,4.]
        n._update_tunnel_mode(12)
        self.assertEqual((n.planning_rate,n.planning_times),(2.,[2.,3.,4.]))
        self.assertEqual((p.weight_offset,p.weight_clearance),(5.,100.))
        n.gps_gate.update=lambda now:False;n._update_tunnel_mode(14)
        self.assertEqual((p.weight_offset,p.weight_safety,p.weight_clearance),(100.,4.,0.))
        self.assertEqual((p.weight_time,p.weight_jerk),(.05,30.))

if __name__=='__main__':unittest.main(verbosity=2)
