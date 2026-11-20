"""CrossBot real-world simulator (MuJoCo 3). The parts are rigid; the connections are what bends and fails.

Bodies
  * every part is a body. Parts on one axle (cross fits: the axle, its gears, wheels, bushes) form one rigid cluster.
  * an axle cluster hangs in the kinematic tree under one of its bearings (the motor for a motor shaft) on a hinge
    joint; its other bearings are soft point constraints. Every other cluster is a free body.
Connections
  * stud connection -> one weld per engaged stud. A stud lets go when its pull (axial force + peel moment / stud
    radius) exceeds the pull-off strength, or its shear exceeds the shear strength; released parts collide normally, so
    a structure comes apart, a motor lifts off its rails, a loose part falls. The weld softness gives the bending.
  * axle in a hole, Technic pin: two point constraints along the hole (soft = bearing play and compliance).
Gear mesh
  * a joint equality between the two axle hinges (the tooth ratio, solved implicitly), plus the tooth force's
    translational part on the two gears and its separating part (tan 20 deg); the mesh disengages when the centre
    distance opens by the tooth height or the gears slide apart along their axes, re-engages when they come back, and
    a tooth skips when the transmitted force exceeds what the remaining engagement carries.
PF motor
  * actuator on the shaft hinge: tau = tau_stall (duty - w / w0) ('brake' = shorted, duty 0; level 0 = float),
    rotor inertia as joint armature, internal Coulomb friction as joint friction loss, IR command latency.
Mechanisms (v13; each behind a Params switch, all off = v12)
  * idler gears on their own hinge; worm self-locking (friction ~ tooth torque on the worm shaft, tooth force along the
    worm axis); knob wheels at their contact radius; chain links' mass on the sprockets
  * rack slides: the moving side on a slide joint in its guides, coupled to the pinion (x = z 3.2 mm / 2 pi theta) with
    the spur skip force, travel stops, running off the pinion fails the run
  * differential (case hinge, ring mesh, tendon w_L + w_R = 2 w_case); CV joint (1 : 1 + ball); shocks (rod body on a
    sprung slide joint); treads (rollers on the bottom run at the outer radius, belt speed, sprockets 1 : 1)
  * PF servo (dead time, drive to the angle, then gearbox friction only), pullback spring motor, no program = all +7
Course and judge are the benchmark's (lego_bench.obstacles); the design format is the benchmark's.
"""
from __future__ import annotations

import base64
import json
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "src"))
os.environ.setdefault("MUJOCO_GL", "egl" if sys.platform.startswith("linux") else "glfw")  # MuJoCo allows EGL only on Linux

import mujoco  # noqa: E402

# (2026-10-29) high-quality renders for the results page / paper: CSIM_HQ=1 changes only what is drawn (lights, shadow
# map, tessellation, 4K offscreen buffer, motion blur streamed to ffmpeg, low-CRF encode); the physics is the same.
HQ = os.environ.get("CSIM_HQ") == "1"
HQ_VISUAL = ('<visual><global offwidth="3840" offheight="2160" fovy="40"/>'
             '<quality shadowsize="16384" offsamples="16" numslices="64" numstacks="48" numquads="8"/>'
             '<headlight ambient="0.22 0.22 0.23" diffuse="0.12 0.12 0.12" specular="0.02 0.02 0.02"/>'
             '<map znear="0.003" zfar="30" shadowclip="1.6" shadowscale="0.5" haze="0.15"/>'
             '<rgba haze="0.93 0.91 0.86 1"/></visual>')
HQ_LIGHTS = ('<light name="key" pos="0.6 -0.9 1.5" dir="-0.35 0.55 -1" diffuse="0.72 0.69 0.64" specular="0.35 0.35 0.35" '
             'castshadow="true"/>'
             '<light name="fill" pos="-0.9 0.8 1.0" dir="0.55 -0.5 -0.7" diffuse="0.24 0.26 0.30" specular="0.05 0.05 0.05" '
             'castshadow="false"/>'
             '<light name="rim" pos="-0.3 -1.2 0.6" dir="0.2 1 -0.4" diffuse="0.18 0.18 0.2" specular="0.25 0.25 0.25" '
             'castshadow="false"/>')

from ai_pd_tasks.lego_bench import obstacles as OB  # noqa: E402
from ai_pd_tasks.lego_bench.design import (  # noqa: E402
    SINGLE_BEVEL_FACE, Design, compute_stats, crossed_mesh_sense, gear_spins, motor_params, motor_trains, place,
    wheel_motors)
from ai_pd_tasks.lego_bench.parts import Box, Boxes, Cyl, Segs, bench_parts  # noqa: E402

GEOM_CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "geom_cache")
SURF_MU = {"abs": 0.4, "laminate": 0.35, "wood": 0.45, "rubber_mat": 1.0, "waxed": 0.12, "cord": 0.3, "steel": 0.3}


@dataclass
class Params:
    """Physical parameters (SI). Each is to be measured or fitted; defaults are first estimates."""
    dt: float = 5e-4
    m_stall_nm: float = 0.075          # PF M stall torque at full duty (AAA box + IR receiver)
    m_no_load_rad_s: float = 41.9      # PF M no-load speed (400 rpm)
    motor_friction: float = 0.15       # running (kinetic) friction of motor + gearbox, fraction of the stall torque
    motor_static: float = 0.45         # starting (static) friction, fraction of the stall torque (bench: M level 1 < it < level 2)
    l_stall_nm: float = 0.13           # PF L motor stall torque at full duty (bench: A1 climbs 32 deg at level 3, not 2)
    level_exp: float = 0.45            # PF level -> drive fraction (L/7)**this (bench: level 4 = 77% of top speed)
    rotor_j: float = 2.0e-5            # rotor inertia reflected to the output shaft [kg m^2]
    ir_latency_s: float = 0.035
    axle_fric_nm: float = 0.0005       # an axle turning in its holes (plus whatever presses on it)
    mesh_drag_nm: float = 0.0          # a meshed gear pair's no-load drag, on its slower shaft [N m]
    mesh_mixed_k: float = 1.0          # a double-bevel meshed with a spur gear: tooth-force capacity factor
    mesh_axial_k: float = 0.0         # double-bevel gears meshed as spurs: axial thrust / tooth force
    mesh_mu: float = 0.0               # tooth sliding loss: drag += mesh_mu * tooth force * radius
    pwm_beta: float = 0.0              # back-EMF scale duty**this (see set_level)
    stud_pull_n: float = 1.2           # pull-off strength per stud on an electrical housing [N] (bench, motor on 1x2: 1.05-1.26 N)
    stud_load_tau: float = 0.002       # stud loads are judged smoothed over this time [s] (impacts last ms)
    stud_slip_vmax: float = 0.01       # fastest a stud slides out of its tube [m/s] (an impact spike does not pop it)
    stud_rock_nmm: float = 22.0        # moment one stud takes rocking in its tube (bench: 1x2 on the motor, 39-51 N mm)
    stud_pull_brick_n: float = 1.32     # the same between plates and bricks (tighter tubes than the PF housings)
    stud_push_n: float = 4.0           # force that pushes a lifted stud back in [N]
    stud_height_mm: float = 1.7        # slid this far out, the stud is free
    stud_slip_k: float = 3.0e4         # slip increment = excess force / this [N/m]
    stud_slip_weaken: float = 0.0      # (fit10) a stud slid part way out of its tube holds (1 - this * slip / stud
    # height) of its strength (its grip is the interference along the engaged length); 0: full strength until it is out
    stud_shear_n: float = 60.0         # shear load that breaks a stud out of its tube [N] (it does not slide out in shear)
    stud_r_m: float = 2.4e-3           # peel lever [m]
    stud_peel_k: float = 2.4           # a stud levered open holds this many times its straight pull (bench: 40-50 N mm)
    stud_spread: float = 0.15          # stud-to-stud strength spread (s.d., fraction; bench: 30% between columns)
    seed: int = 0
    mu_rod: float = 0.0                # plastic on the printed PLA bump rods (0: mu_board)
    mu_board: float = 0.42             # hard plastic on the foam-core boards (bench: a 2x4 brick slides at 23 deg)
    axle_slide: bool = True            # axles slide along their holes between whatever stops them
    axle_slide_fric_n: float = 0.3     # friction of an axle sliding in its holes [N]
    shaft_grip_n: float = 3.0          # an axle in a cross hole (motor output) slides out above this [N]
    slide_stop_solref: tuple = (0.002, 1.0)  # an axle's end stops (a bush or gear against a brick face): hard plastic
    # contact, as stiff as the part contacts (MuJoCo's default joint limit, 0.02 s, let axles push 10 mm past their stops)
    stud_solref: tuple = (0.001, 1.0)  # stud point-constraint softness (bending)
    stud_solimp: tuple = (0.9, 0.95, 0.001, 0.5, 2.0)  # stud impedance: dmax near 1 = a stiff joint even between light bricks
    stud_torquescale: float = 0.003  # rotational rows ~ stud radius: moments go through stud pairs
    stud_k_n_m: float = 0.0            # >0: each stud point constraint gets this stiffness [N/m] whatever the masses
    # of the two parts (MuJoCo's solref is a time constant, so a fixed solref is ~100x softer on a 2 g plate than on
    # a 30 g motor); its time constant is set per constraint from its effective mass (_stiffen)
    bearing_k_n_m: float = 0.0         # the same for an axle or pin in its hole
    stiff_imp: float = 0.99            # impedance of those constraints (near 1: the stiffness comes from the reference
    # spring, so light parts reach it without a time constant below 2 dt)
    bearing_solref: tuple = (0.001, 1.0)
    bearing_solimp: tuple = (0.9, 0.95, 0.001, 0.5, 2.0)  # bearings and pins, kept as stiff as the studs they share loads with
    mesh_solref: tuple = (0.002, 1.0)
    mesh_cap_n: float = 40.0           # tooth force a fully engaged mesh carries before a tooth skips [N]
    addendum_mm: float = 0.85
    mesh_cap_z_exp: float = 1.0        # skip force scales with (smaller tooth count / 12) ** this
    chain_cap_n: float = 30.0          # chain tension at which it jumps a sprocket tooth [N]. UNCALIBRATED (2026-10-26): a default, no filmed run loads a chain; set it from the real-world round (20 of 39 L1 designs use chains)
    chain_cap_n_per_tooth: float = 0.0  # (v12) >0: the chain skips at this tension per tooth of its smaller sprocket
    # [N/tooth] instead of chain_cap_n (0.625: 16T 10 N, 24T 15 N, 14T 8.75 N, 40T 25 N). An ESTIMATE from one filmed
    # lifting test (YouTube "How Much Can LEGO Chain Links Lift?", Build it with Bricks: a slack chain skipped at
    # ~5-10 N, a tensioned 24T sprocket held ~12-13 N); otherwise uncalibrated
    chain_engage_mm: tuple = ()        # (v12) (closer, farther, axial) [mm]: the chain carries only while its sprockets
    # are at most `closer` nearer / `farther` further apart than as built (about half a link of slack) and at most
    # `axial` out of line; outside it is disengaged (it skips / falls off). Empty: the old window (25 mm closer,
    # 2.5 mm further, 50 mm axial)
    pressure_deg: float = 20.0
    engage_off: float = 0.15           # engagement below which the mesh lets go
    mesh_slip_frac: float = 1.0        # mean tooth force while the teeth ratchet, fraction of the skip force
    mesh_reengage: bool = True         # a free mesh whose teeth come back into engagement ratchets, then locks
    mesh_clutch: bool = False          # (fit10) the mesh as an implicit slip clutch: a fixed tendon qa - a1 qb with a
    # friction loss of the skip torque (cap * ra) while the teeth hold, mesh_slip_frac of it while they ratchet. The
    # solver never passes more than the skip force, also within a step (the old locked joint equality passed an
    # unbounded impulse each time it re-locked: a ratcheting mesh carried ~3x its ratchet force, and an impact spike
    # reached the teeth in full). False: the joint equality (locked) / applied ratchet torque (slipping) as before.
    mesh_lock_v: float = 0.03          # (equality mesh) a ratcheting mesh locks again below this tooth sliding speed
    # [m/s]. Each re-lock syncs the two sides in one step: an impulse ~ J dv the skip force does not bound (fit10:
    # at 0.03 a 1 N ratchet passed ~3 N on average)
    mesh_clutch_imp: float = 0.9999    # (clutch) impedance of the stick friction: a soft friction creeps under load
    # (creep ~ (1 - d) / d * A * f / b; at 0.99 a light gear train crept past mesh_slip_v under the drive torque)
    mesh_slip_v: float = 0.005         # (clutch) tooth sliding speed above which the mesh counts as ratcheting [m/s]
    mesh_mixed_gap_mm: float = 0.0     # (fit10) >0: a double-bevel gear meshed with a spur gear carries
    # exp(-gap / this) of its tooth force with its centres pulled apart by gap (the double bevel's chamfered flanks: off
    # its pitch line the spur tooth rides on the chamfer and cams over; e.g. 20t double bevel on a 24t spur 7 plates
    # apart, 0.4 mm loose, ratchets at ~1 N). 0: no gap dependence beyond the engagement
    judge_at_goal_s: float | None = 1.0  # a crossed run's end state and flip judged this long after reaching the goal
    tyre_torsion_m: float = 0.002      # tyre torsional (spin) friction length: torque = this * normal force
    tyre_fillet_mm: float = 0.0        # tyre shoulder radius (0: a sharp-edged cylinder, as before fit8)
    mu_tyre_floor: float = 0.8
    mu_tyre_ramp: float = 0.8
    mu_tyre_rod: float = 0.0           # tyre on the printed bump rods (0: mu_tyre_floor)
    mu_abs: float = 0.4
    tyre_solref: tuple = (0.012, 0.9)  # rubber tyre contact softness (a few mm of compression under impact)
    shrink_mm: float = 0.25            # collision boxes shrunk so seated parts do not touch
    settle_s: float = 0.5
    program_late_s: float = 0.0        # (v12, "late" reliability nudge) every motor command after the first (later
    # than the program's earliest command time) fires this much later; the first command(s) unchanged
    pin_slide_n: float = 8.0           # axial force that slides a friction pin in its holes [N] (ridges grip)
    pin_friction_nm: float = 0.004     # torque that turns a friction pin in its hole [N m] (bench: 3.5-4.5 N mm)
    cables: bool = True
    cable_taped: bool = True           # leads taped along the vehicle (as on the real runs): no free loops
    stiff_tc_min_dt: float = 2.0       # stiffened joints: time constant not below this many steps (stability of light parts)
    spawn_touch_m: float = 1e-4        # part pairs closer than this at spawn get no contact (flush assembled neighbours:
                                       # their face contact only fights the joints; was -2e-4 = overlapping pairs only)
    board_solref: tuple = ()           # (fit10) contact softness of the course parts (empty: MuJoCo default 0.003 s
    # via <default>); a landing car's plastic on foam core: impacts spread over a few ms
    course_dynamic: tuple = ("ramp", "seesaw")  # (fit10) benches whose dynamic course shapes on a revolute joint move:
    # the ramp board hangs on a hinge along its top edge, its foot resting on the floor (obstacles.final_ramp, user
    # 2026-10-24); (2026-10-25) the seesaw plank pivots on its axle, resting on / landing on the ledges
    cord_dynamic: bool = True          # the rope bench's cord deflects under the vehicle (a taut string, point load)
    cord_tension_n: float = 25.0       # its tension [N] (filmed flat rope: ~28 mm deep under a 0.47 kg car, 665 mm span)
    cord_tyre_solref: tuple = ()       # tyre-cord contact softness (empty: tyre_solref)
    cord_bodies: bool = False          # the cord as a dynamic chain of segments (_cord_chain) instead of mocap
    cord_damp_ns_m: float = 0.5        # (chain) node slide damping [N s/m]
    cord_tau_s: float = 0.003          # (chain) hinge damping / stiffness [s]
    cord_nodes: bool = True            # loads per node (several point loads) instead of one V at the centroid
    cord_vertical_load: bool = True    # the cord deflects under the vertical resultant of its contact forces
    cord_sag0_mm: float = 1.0          # its unloaded sag at mid-span (filmed: taut, 0-2 mm)
    cord_tau: float = 0.15             # load smoothing [s] (contact impulses average out; the cord is quasi-static)
    cable_mass_g: float = 5.0          # a PF lead
    cable_r_m: float = 1.3e-3          # flat 4 x 2 mm lead as a round cable
    cable_bend: float = 4.0e7          # bending modulus of the lead [Pa]
    cable_twist: float = 1.0e7
    cable_damping: float = 2e-5
    cable_contact: bool = True         # free leads touch the course (False: they pass through it; fit10: a free lead
    # pinched between a landing car and a step tread blew up the whole vehicle)
    cable_armature: float = 0.0        # (fit10) rotor-like inertia added to each lead joint [kg m^2]: a 0.4 g, 2.6 mm
    # segment twists at ~2900 rad/s (omega dt 1.4 at 0.5 ms; the plugin's elastic torques are explicit), and a yanked
    # free lead blew up and took the vehicle with it (sonnet ramp L4, 2026-10-24). 1e-8 brings it to ~500 rad/s
    plug_pull_n: float = 6.0           # a PF plug pops off its pad above this pull [N]
    tyre_mu_exp: float = 0.0           # (fit11) tyre friction scales with (the tyre's measured mu (parts.tyre_mu) /
    # tyre_mu_ref) ** this; 0: every tyre grips alike, as before (the fitted mu_tyre_* are the 32020c01's)
    tyre_mu_ref: float = 1.38          # 32020c01 (most of the filmed cars)
    motor_i_max: float = 0.0           # (fit11) >0: each PF output's current limit (IR receiver overcurrent protection)
    # as a fraction of the motor's full-duty stall current: the motor torque never exceeds this x its stall torque
    # (high levels saturate near stall; low levels and free running are unaffected). 0: no limit, as before
    supply_r: float = 0.0              # (fit11) battery box + receiver resistance / one motor's winding resistance:
    # motors sharing the supply lose torque together near stall (_supply_sag); 0: an ideal supply as before
    tyre_crr: float = 0.0              # (v14) rolling resistance of a tyred wheel: a rolling-friction torque up to
    # tyre_crr x wheel radius x normal load (MuJoCo condim 6), i.e. a resisting force tyre_crr x load at the tread:
    # rubber hysteresis on the foam board + the load-dependent rub of the axle in its holes. 0: condim 4, no rolling
    # resistance, as before (v13)
    axle_load_tau_s: float = 0.0       # (v14) that radial load low-passed over this time [s] (as the stud loads,
    # stud_load_tau: a stiff contact's ms impact spike does not lock a free wheel). 0: the instantaneous load
    chain_jam_slack_mm: float = 0.0    # (v14 proposal) a chain built with more slack than this [mm] (links x 6.4 mm
    # minus the loop round its sprockets, inventory_r4b.chain_geoms) jams: the loose strand rides up and wedges, the
    # chain does not move (filmed: Sol bridge L1 at +4.5 mm jammed, the same car at -1.8 mm ran; the validator now
    # takes -2.0..+3.2 mm). 0: off, engagement is relative to as built whatever the slack (v13)
    chain_jam_nm: float = 0.5          # friction torque a jammed chain puts on each of its sprocket shafts [N m]
    axle_load_fric_m: float = 0.0      # (v14) a FREE (undriven) wheel axle rubs in its holes in proportion to the
    # load on its wheels: friction torque = axle_fric_nm + this x the radial load (|vector sum of the contact forces on
    # the axle's wheels|, its axial part removed) [N m per N],
    # i.e. hard-plastic mu x the axle's bearing radius (0.4 x 2.4 mm). Driven axles keep their fitted losses (the
    # motor / gearbox / mesh terms were calibrated with them). 0: load-independent axle_fric_nm only, as before (v13)
    step_nose_mm: float = 0.0          # (fit11) steps bench: the treads' front edges rounded with this radius [mm]
    # (1/2 in foam board noses; 0: sharp box edges as before)
    # ---- (v13) the mechanisms the design format offers (lego_bench.racks / treads / inventory_r4b). Every switch off
    # (False / 0) reproduces v12 exactly; params_v13.json turns them on.
    no_program_all7: bool = False      # a design without a motor_program: every PF motor at +7 from t = 0 (the docs)
    all_skips_pullback: bool = False   # "all" never commands a pullback motor (it is not IR controlled)
    servo_pos: bool = False            # 99498c01 as the PF position servo (philohome.com/pfservo): level k -> k/7 x
    # 90 deg from centre (0 and "brake": centre). After its dead time it drives towards the target with up to
    # servo_torque_nm at up to servo_w0_rad_s; on reaching it (or blocked for servo_block_s) it stops driving and holds
    # only by its gearbox friction (no active correction: a load can push it off until the next command)
    servo_torque_nm: float = 0.10      # its drive torque (measured usable 8.5..12.8 N cm)
    servo_w0_rad_s: float = 13.09      # its speed (90 deg in ~0.12 s, ~125 rpm)
    servo_dead_s: float = 0.12         # command sent -> motion (incl. the 35 ms IR latency)
    servo_block_s: float = 0.25        # blocked this long, it gives up until the next command
    servo_tol_deg: float = 1.0         # within this of its target it stops driving
    servo_kp_s: float = 60.0           # approach: speed command = this x angle error [1/s] (capped at servo_w0)
    servo_kv: float = 4.0              # speed loop: torque fraction per fraction of servo_w0 speed error
    pullback_model: bool = False       # 12787c01 as the spring motor: held until t = 0, then tau = stall x (wound
    # fraction) x (1 - w / w0) in its drive sense, freewheeling when faster (inventory_r4b)
    pullback_stall_nm: float = 0.05    # its torque fully wound [N m] (rating)
    pullback_no_load_rad_s: float = 62.83  # 600 rpm (rating)
    pullback_turns: float = 10.0       # output turns of a full wind
    pullback_hold_nm: float = 1.0      # before the release its output is held (ratchet / hand)
    idler_hinge: bool = False          # a round-bore gear (6542b, 18946, 69761) turns on its axle on its own hinge
    rack_slide: bool = False           # rack slides: the moving side on a slide joint in its guides, pinion-rack
    # coupling x = r theta (r = z 3.2 / 2 pi mm) with the gear meshes' skip force, stops at the travel ends; running off
    # the pinion fails the run
    diff_model: bool = False           # 62821b: the case on a hinge about the output axis, ring meshes to the case,
    # w_L + w_R = 2 w_case (a tendon equality on the three hinges)
    cv_model: bool = False             # CV joint: the two shafts turn 1 : 1 (joint equality) + a ball joint
    shock_model: bool = False          # shocks: rod body on a slide joint with the spring rate, preload, stroke
    # limits and damping (inventory_r4b.SHOCKS)
    worm_lock_k: float = 0.0           # worm (4716) self-locking: friction on the worm shaft = this x the tooth torque
    # on it (>= 1: the gear cannot turn the worm; forward efficiency 1 / (1 + this) on top of mesh_mu)
    knob_r_mm: float = 0.0             # knob wheels (32072): effective contact radius [mm] (0: z / 2 = 2 mm, as v12)
    chain_link_mass_g: float = 0.0     # 3711 chain links' mass, half on each sprocket [g per link] (0: none, as v12)
    tread_model: bool = False          # treads: rollers along the bottom run at the tread's outer radius, turning
    # at the belt speed (sprocket rate x pitch radius), sprockets coupled 1 : 1, the links' mass on the rollers
    tread_mu_3873: float = 0.0         # tread link friction on the course, per link (3873, 57518, 3711); 0: as a tyre
    tread_mu_57518: float = 0.0        # of the link's measured grip (treads.TREAD_LINKS tyre_mu: 0.23 / 0.25 / 0.23)
    tread_mu_3711: float = 0.0         # mapped like every wheel's: mu_tyre_* x (tyre_mu / tyre_mu_ref) ** tyre_mu_exp
    tread_spacing_mm: float = 6.4      # roller spacing along the bottom run
    rack_lock_v: float = 0.005         # (rack) a ratcheting rack mesh locks again below this sliding speed [m/s]
    rack_pinion_armature: float = 5e-7  # (rack) rotational inertia added to the pinion's shaft [kg m^2] (~ a gear train
    # on it; a bare 1 g pinion against a 200 g slide left the coupling numerically soft, mm under a few N)

    @classmethod
    def from_dict(cls, d: dict) -> "Params":
        p = cls()
        for k, v in (d or {}).items():
            if not hasattr(p, k):
                raise KeyError(k)
            setattr(p, k, tuple(v) if isinstance(getattr(p, k), tuple) else v)
        return p


def qxyzw_to_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_wxyz(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(-1))
    return q


def f3(v):
    return " ".join(f"{float(x):.7g}" for x in v)


def _z_to(v):
    """wxyz quaternion turning +z onto the unit vector v"""
    v = np.asarray(v, float) / np.linalg.norm(v)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, v))
    if c < -0.999999:
        return np.array([0.0, 1.0, 0.0, 0.0])
    a = np.cross(z, v)
    q = np.array([1.0 + c, *a])
    return q / np.linalg.norm(q)


def _axis_quat(axis: str):
    """MuJoCo cylinders run along local z; a part's cylinder along its local x / y"""
    return {"x": "0.7071068 0 0.7071068 0", "y": "0.7071068 -0.7071068 0 0"}.get(axis, "1 0 0 0")


@dataclass
class StudEq:
    eq: int
    upper: str
    lower: str
    conn: int
    released_t: float | None = None
    last: tuple = (0.0, 0.0, 0.0)
    peak: float = 0.0
    slip: float = 0.0            # how far the stud has slid out of its tube [m]
    base: list | None = None
    eqs: list | None = None      # its point constraints
    pull: float = 0.0            # its straight pull-off strength [N]
    pt: tuple = (0.0, 0.0, 0.0)  # where it is (world, at spawn)
    rock: bool = True            # a part held only by one row of studs can hinge about the row: check rocking


@dataclass
class Mesh:
    a: str
    b: str
    ra: float
    rb: float
    wa: float
    wb: float
    za: int
    kind: str = "gear"    # gear | chain
    dbl: bool = False     # both double-bevel: the chamfered teeth push the gears apart along their axles
    same: bool = True     # same tooth form (both double-bevel or both spur)
    spur: bool = True     # involute gears meshing as spurs (not knob wheels, worms or right-angle bevels)
    c0: float = 0.0       # centre distance at spawn (the designed mesh)
    eq: int = -1
    ja: int = -1          # dof index of gear a's hinge
    jb: int = -1
    sa: float = 1.0       # gear axis vs its hinge axis
    a1: float = 0.0       # qa = c0 + a1 qb
    engaged: float = 1.0
    active: bool = True
    state: str = "locked"
    skips: int = 0
    tau_a: float = 0.0
    tau_b: float = 0.0
    f_t: float = 0.0
    peak: float = 0.0




def _faces_away(part_id, p, n, q) -> bool:
    """A single bevel (6589 / 32198) at p, axis n (its local y), whose toothed face points away from the axes'
    intersection with the gear at q: the two gears touch back to back and do not mesh."""
    face = SINGLE_BEVEL_FACE.get(part_id)
    return face is not None and float(np.dot(np.asarray(q) - np.asarray(p), n)) * face <= 0


def _fillet_cyl(r, hl, rho, n=72, k=6):
    """<mesh> vertex list of a cylinder (radius r, half length hl along z) whose two rim edges are rounded with radius
    rho: a tyre's cross-section (its shoulders are round, so a cord or a rod edge rides up them instead of catching a
    sharp corner)"""
    pts = []
    for sz in (-1.0, 1.0):
        for j in range(k + 1):
            a = 0.5 * math.pi * j / k
            rr, zz = r - rho + rho * math.cos(a), sz * (hl - rho + rho * math.sin(a))
            for i in range(n):
                t = 2 * math.pi * (i + 0.5 * (j % 2)) / n
                pts.append((rr * math.cos(t), rr * math.sin(t), zz))
    return " ".join(f"{v:.6g}" for p_ in pts for v in p_)


def _tube_obj(r_out, r_in, half, ax, cen, n=96):
    """OBJ lines of a closed tube (mm, axis `ax` through `cen`): a tyre's tread band"""
    lines, idx = [], {}
    def v(r, t, z):
        p = [0.0, 0.0, 0.0]
        o = [i for i in range(3) if i != ax]
        p[o[0]], p[o[1]], p[ax] = r * math.cos(t), r * math.sin(t), z
        lines.append("v %.3f %.3f %.3f" % tuple(np.array(p) + cen))
        return len([l for l in lines if l[0] == "v"])
    ring = {}
    for j, (r, z) in enumerate(((r_out, -half), (r_out, half), (r_in, half), (r_in, -half))):
        ring[j] = [v(r, 2 * math.pi * i / n, z) for i in range(n)]
    for j in range(4):
        A_, B_ = ring[j], ring[(j + 1) % 4]
        for i in range(n):
            a, b, c, d = A_[i], A_[(i + 1) % n], B_[(i + 1) % n], B_[i]
            lines += ["f %d %d %d" % (a, b, c), "f %d %d %d" % (a, c, d)]
    return lines

def _smooth_obj(V, F, crease_deg=35.0):
    """OBJ text with per-corner normals: corners share a normal across faces meeting at less than crease_deg (smooth
    curved surfaces, crisp edges), vertices welded by position (the source meshes repeat them per triangle)"""
    P = V[F].reshape(-1, 3).astype(np.float64)
    key = np.round(P * 100).astype(np.int64)  # weld at 0.01 mm
    _, wid = np.unique(key, axis=0, return_inverse=True)
    wid = wid.reshape(-1)
    # consistent winding: neighbours across an edge run it in opposite directions; then each connected patch faces
    # outwards (positive signed volume). The source meshes mix windings, which lights some faces from behind.
    W3 = wid.reshape(-1, 3).copy()
    nf0 = len(W3)
    edges = {}
    for f_ in range(nf0):
        for k_ in range(3):
            a_, b_ = int(W3[f_, k_]), int(W3[f_, (k_ + 1) % 3])
            edges.setdefault((min(a_, b_), max(a_, b_)), []).append(f_)
    flip = np.zeros(nf0, dtype=bool)
    seen = np.zeros(nf0, dtype=bool)
    comp = np.full(nf0, -1)
    nc = 0
    for s0 in range(nf0):
        if seen[s0]:
            continue
        stack = [s0]
        seen[s0] = True
        comp[s0] = nc
        while stack:
            f_ = stack.pop()
            tri = W3[f_][::-1] if flip[f_] else W3[f_]
            for k_ in range(3):
                a_, b_ = int(tri[k_]), int(tri[(k_ + 1) % 3])
                for g_ in edges[(min(a_, b_), max(a_, b_))]:
                    if seen[g_]:
                        continue
                    tg = W3[g_]
                    same = any(int(tg[j]) == a_ and int(tg[(j + 1) % 3]) == b_ for j in range(3))
                    flip[g_] = same  # the neighbour must run a->b the other way
                    seen[g_] = True
                    comp[g_] = nc
                    stack.append(g_)
        nc += 1
    Pt = P.reshape(-1, 3, 3)
    Pt[flip] = Pt[flip][:, ::-1]
    Wt = wid.reshape(-1, 3)
    Wt[flip] = Wt[flip][:, ::-1]
    vol = np.einsum("ij,ij->i", Pt[:, 0], np.cross(Pt[:, 1], Pt[:, 2]))
    cv = np.bincount(comp, weights=vol, minlength=nc)
    # open patches (edges used by one face, e.g. tyre tread lugs) have no inside: face them away from the part centre
    open_c = np.zeros(nc, dtype=bool)
    for (a_, b_), fs in edges.items():
        if len(fs) == 1:
            open_c[comp[fs[0]]] = True
    fcen = Pt.mean(axis=1)
    fnr = np.cross(Pt[:, 1] - Pt[:, 0], Pt[:, 2] - Pt[:, 0])
    out_ = np.einsum("ij,ij->i", fnr, fcen - P.mean(axis=0))
    co = np.bincount(comp, weights=out_, minlength=nc)
    cv = np.where(open_c, co, cv)
    bad = cv[comp] < 0
    Pt[bad] = Pt[bad][:, ::-1]
    Wt[bad] = Wt[bad][:, ::-1]
    P = Pt.reshape(-1, 3)
    wid = Wt.reshape(-1)
    a, b, c = P[0::3], P[1::3], P[2::3]
    fn = np.cross(b - a, c - a)
    ln = np.linalg.norm(fn, axis=1, keepdims=True)
    fn = fn / np.maximum(ln, 1e-12)
    nf = len(F)
    corner_face = np.repeat(np.arange(nf), 3)
    order = np.argsort(wid, kind="stable")
    ws = wid[order]
    bounds = np.flatnonzero(np.diff(ws)) + 1
    groups = np.split(order, bounds)
    cosc = np.cos(np.radians(crease_deg))
    N = np.zeros((nf * 3, 3))
    for g in groups:
        f_ = corner_face[g]
        n_ = fn[f_]
        w_ = ln[f_, 0:1]
        sim = n_ @ n_.T
        m = (sim >= cosc).astype(np.float64)
        acc = m @ (n_ * w_)
        N[g] = acc / np.maximum(np.linalg.norm(acc, axis=1, keepdims=True), 1e-12)
    vk = np.round(np.hstack([P * 100, N * 1000])).astype(np.int64)
    uk, inv = np.unique(vk, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    first = np.zeros(len(uk), dtype=np.int64)
    first[inv[::-1]] = np.arange(len(inv))[::-1]
    lines = ["v %.3f %.3f %.3f" % tuple(P[i]) for i in first] + ["vn %.4f %.4f %.4f" % tuple(N[i]) for i in first]
    I = inv.reshape(-1, 3) + 1
    lines += ["f %d//%d %d//%d %d//%d" % (x, x, y, y, z, z) for x, y, z in I]
    return lines


def _implicit_bearings(d):
    """A real axle that passes through a brick's or liftarm's hole is held there whether or not the design lists the
    connection: add those bearings (hinge connections) to the design. -> the added (axle, brick) pairs"""
    from ai_pd_tasks.lego_bench.design import DesignConnection, axle_through_holes
    P = place(d)
    linked = {frozenset((c.child, c.parent)) for c in d.connections}
    added = []
    for a, pa in P.items():
        if pa.bp.kind != "axle":
            continue
        for b, pb in P.items():
            if b == a or pb.bp.kind not in ("technic_brick", "axle_brick") or frozenset((a, b)) in linked:
                continue
            try:
                hit = axle_through_holes(pa, pb)
            except Exception:  # noqa: BLE001
                hit = []
            if hit:
                d.connections.append(DesignConnection(a, b, "hinge"))
                added.append((a, b))
    return added

class CrossSim:
    def __init__(self, design_path: str, bench: str, level: int, params: Params | dict | None = None,
                 ladder: str = "final", start_gap_mm: float | None = None, reverse: bool = False, spec=None,
                 program: list | None = None):
        """start_gap_mm: front of the vehicle to the obstacle's start line (default: the design's, else the bench's);
        reverse: the vehicle turned round on the start line and driven rear first towards the obstacle (the motors
        run the other way); spec: a custom course for this bench (obstacles.generate(..., spec=); e.g. a measured
        filmed course), None = the level's own; program: a motor program replacing the design's (a command without
        "motor" drives the design's main motor)"""
        self.p = params if isinstance(params, Params) else Params.from_dict(params or {})
        self.cat = bench_parts()
        raw = json.load(open(design_path)) if isinstance(design_path, str) else dict(design_path)
        d = Design.from_dict(raw)
        dir0 = compute_stats(d).get("motor_direction", 1) or 1
        if reverse:
            for pt in d.parts:
                pt.pos_mm = (-pt.pos_mm[0], -pt.pos_mm[1], pt.pos_mm[2])
                pt.yaw_deg = (pt.yaw_deg + 180.0) % 360.0
        if start_gap_mm is not None:
            d.start_gap_mm = float(start_gap_mm)
        if program is not None:
            d.motor_program = [dict(c, motor=c.get("motor", d.motor)) for c in program]
        self.implicit = _implicit_bearings(d)  # axles resting in holes the design did not declare (they still hold)
        spins = gear_spins(d)
        for pt in d.parts:
            if pt.name in spins:
                pt.spin_deg = spins[pt.name]
        self.design = d
        self.design_path = design_path if isinstance(design_path, str) else ""
        self.stats = compute_stats(d)
        self.ob = OB.with_start_gap(OB.generate(bench, level, ladder, spec=spec), d)
        self.pose = OB.place_vehicle(self.ob, d, self.stats, lift=0.0005)
        self.ob = OB.finalize(self.ob, d, self.stats, self.pose)
        self.parts = {pt.name: pt for pt in d.parts if pt.part in self.cat}
        self.world = {n: (np.array(pos), qxyzw_to_mat(q)) for n, (pos, q) in self.pose.parts.items() if n in self.parts}
        self.log: list[str] = []
        self.wall_limit_s = float(os.environ.get("CSIM_WALL_S", "300"))
        self._build()

    # ================================================================ model
    def kind(self, n):
        return self.cat[self.parts[n].part].kind

    def _part_geoms(self, n, bp):
        out = []
        s, sh = bp.shape, self.p.shrink_mm
        mat = f' friction="{self.p.mu_abs} 0.005 0.0001"'
        boxes = ()
        if isinstance(s, Boxes) and s.boxes:
            boxes = s.boxes
        elif isinstance(s, Segs) and s.segs:
            for k, (x0, z0, x1, z1) in enumerate(s.segs):
                out.append(f'<geom name="{n}__s{k}" type="capsule" fromto="{f3(np.array([x0, 0, z0]) * 1e-3)} '
                           f'{f3(np.array([x1, 0, z1]) * 1e-3)}" size="{(s.radius - sh) * 1e-3:.5g}"{mat} group="3"/>')
        elif isinstance(s, Box):
            boxes = ((s.lo, s.hi),)
        for k, (lo, hi) in enumerate(boxes):
            lo, hi = np.array(lo, float), np.array(hi, float)
            half = np.maximum((hi - lo) / 2 - sh, 0.2)
            out.append(f'<geom name="{n}__b{k}" type="box" pos="{f3((lo + hi) / 2 * 1e-3)}" size="{f3(half * 1e-3)}"'
                       f'{mat} group="3"/>')
        if isinstance(s, Cyl):
            r = s.radius
            if bp.kind == "gear":
                r = max(bp.teeth / 2.0 - 0.8, 2.5)  # root circle (the teeth are the mesh model)
            elif bp.kind != "wheel":
                r = max(r - sh, 0.5)
            hl = max(s.half_len - (0 if bp.kind == "wheel" else sh), 0.3)
            rho = min(self.p.tyre_fillet_mm, hl * 0.9, r * 0.5) if (bp.kind == "wheel" and bp.tyre_mu) else 0.0
            if rho > 0.05:  # a tyre: a cylinder with rounded shoulders (convex mesh), not sharp-edged
                mn = f"tyre_{bp.part_id}"
                if mn not in self._tyre_assets:
                    self._tyre_assets[mn] = _fillet_cyl(r * 1e-3, hl * 1e-3, rho * 1e-3)
                out.append(f'<geom name="{n}__c" type="mesh" mesh="{mn}" pos="{f3(np.array(s.centre) * 1e-3)}" '
                           f'quat="{_axis_quat(s.axis)}"{mat} group="3" priority="2"/>')
            else:
                out.append(f'<geom name="{n}__c" type="cylinder" pos="{f3(np.array(s.centre) * 1e-3)}" '
                           f'quat="{_axis_quat(s.axis)}" size="{r * 1e-3:.5g} {hl * 1e-3:.5g}"{mat} group="3"'
                           + (' priority="2"' if bp.kind == "wheel" else "") + '/>')
        if os.path.exists(os.path.join(GEOM_CACHE, f"{bp.part_id}.json")):
            for name, col in CrossSim._meshcols.get(bp.part_id, []):
                out.append(f'<geom type="mesh" mesh="m_{name}" contype="0" conaffinity="0" group="1" rgba="{col}" '
                           f'material="plastic" mass="0"/>')
            if False:
                out.append(f'<geom type="mesh" mesh="m_{bp.part_id}" contype="0" conaffinity="0" group="1" '
                       f'rgba="{self._rgba(bp.part_id)}" mass="0"/>')
        return out

    def _rgba(self, pid):
        c = json.load(open(os.path.join(GEOM_CACHE, f"{pid}.json")))["c"]
        return f3([int(c[k:k + 2], 16) / 255 for k in (1, 3, 5)]) + " 1"

    def _inertia(self, bp):
        m = max(bp.mass_g, 0.15) * 1e-3
        s = bp.shape
        if isinstance(s, Cyl):
            r, L = s.radius * 1e-3, 2 * s.half_len * 1e-3
            ia, it = 0.5 * m * r * r, m * (3 * r * r + L * L) / 12
            I = {"x": (ia, it, it), "y": (it, ia, it), "z": (it, it, ia)}[s.axis]
            com = np.array(s.centre) * 1e-3
        else:
            lo, hi = np.array(s.lo) * 1e-3, np.array(s.hi) * 1e-3
            e = hi - lo
            I = (m * (e[1] ** 2 + e[2] ** 2) / 12, m * (e[0] ** 2 + e[2] ** 2) / 12, m * (e[0] ** 2 + e[1] ** 2) / 12)
            com = (lo + hi) / 2
        if any(bp.com_mm):
            com = np.array(bp.com_mm) * 1e-3
        return m, com, np.maximum(np.array(I), 2e-9)

    def _build(self):
        p, cat, d, ob = self.p, self.cat, self.design, self.ob
        names = list(self.parts)
        # ---- rigid clusters (cross fits)
        par = {n: n for n in names}

        def find(x):
            while par[x] != x:
                par[x] = par[par[x]]
                x = par[x]
            return x
        for c in d.connections:
            if c.kind == "cross" and c.parent in par and c.child in par:
                a_, b_ = find(c.parent), find(c.child)
                if a_ != b_:
                    par[b_] = a_
        self._v13_plan()
        for a_, b_ in self._v13_merge:  # (v13) differential satellites ride on the case
            ra_, rb_ = find(a_), find(b_)
            if ra_ != rb_:
                par[rb_] = ra_
        # a Technic pin is held fast in one of the two parts it joins (that part turns with it, the other is its
        # bearing): pin + that part form one cluster, which hinges in the other part with the pin's friction
        for pn in names:
            if self.kind(pn) != "pin":
                continue
            partners = [c.parent if c.child == pn else c.child for c in d.connections
                        if c.kind == "pin" and pn in (c.parent, c.child)]
            partners = [q for q in partners if q in par and self.kind(q) not in ("axle", "pin")]
            if self._shocks:  # (v13) a pin in a shock's end hole turns there (the shock is two bodies): never fused
                partners = [q for q in partners if q not in self._shocks]
                partners = partners if len(partners) >= 2 else []
            if len(partners) >= 2:
                a_, b_ = find(pn), find(partners[-1])
                if a_ != b_ and not any(self.kind(q) in ("axle", "pin") for q in names if find(q) == b_):
                    par[b_] = a_
        groups: dict[str, list[str]] = {}
        for n in names:
            groups.setdefault(find(n), []).append(n)
        root_of = {}
        clusters = {}
        for members in groups.values():
            axles = [m for m in members if self.kind(m) in ("axle", "pin")]
            root = axles[0] if axles else members[0]
            clusters[root] = members
            for m in members:
                root_of[m] = root
        self.root_of = root_of
        # ---- tree joints: an axle cluster hangs under one bearing (motor first)
        bear = {}  # cluster root -> list of (connection index, connection) bearing the cluster
        for ci, c in enumerate(d.connections):
            if c.kind not in ("hinge", "motor", "pin") or c.parent not in par or c.child not in par:
                continue
            if ci in self._v13_nobear:  # (v13) an idler gear's / differential case's bearing: placed below
                continue
            ka, kb = self.kind(c.child), self.kind(c.parent)
            ax_part = c.child if ka in ("axle", "pin") else (c.parent if kb in ("axle", "pin") else c.child)
            other = c.parent if ax_part == c.child else c.child
            if root_of[ax_part] == root_of[other]:
                continue
            bear.setdefault(root_of[ax_part], []).append((ci, c, ax_part, other))
        tree_parent = {}  # cluster root -> (parent part, connection index, axle part, kind)

        def ancestors(r):
            out = set()
            while r in tree_parent:
                r = root_of[tree_parent[r][0]]
                if r in out:
                    break
                out.add(r)
            return out
        for r, lst in bear.items():
            lst = sorted(lst, key=lambda e: 0 if e[1].kind == "motor" else 1)
            for ci, c, axp, other in lst:
                if r in ancestors(root_of[other]) or root_of[other] == r:
                    continue
                tree_parent[r] = (other, ci, axp, c.kind)
                break
        self._v13_tree(tree_parent, root_of, ancestors)
        self.tree_parent = tree_parent
        children: dict[str, list[str]] = {}
        for r, (pp, *_rest) in tree_parent.items():
            children.setdefault(pp, []).append(r)
        # ---- XML
        A, W, E, X, C, ACT = [], [], [], [], [], []
        self._tyre_assets = {}
        for pid in sorted({self.parts[n].part for n in names}):
            mp = os.path.join(GEOM_CACHE, f"{pid}.json")
            if os.path.exists(mp):
                groups_ = self._export_mesh(pid, mp)
                is_wheel = self.cat[pid].kind == "wheel" and len(groups_) > 1
                dark = min((g_[1] for g_ in groups_), key=lambda c_: sum(float(v) for v in c_.split()[:3])) if is_wheel else None
                for name, _c in groups_:
                    # a wheel's rim drawn 0.6 % smaller: its faces that coincide with the tyre's no longer flicker
                    sc = 0.000994 if is_wheel and _c != dark else 0.001
                    A.append(f'<mesh name="m_{name}" file="{name}.obj" scale="{sc} {sc} {sc}"/>')
        W.append(f'<geom name="floor" type="plane" size="3 3 0.1" pos="0 0 {ob.floor_z}" material="table" contype="3" conaffinity="3" '
                 f'friction="{p.mu_board} 0.005 0.0001" priority="1"/>')
        for k_, (x0_, x1_, y0_, y1_) in enumerate(ob.params.get("plates_mm") or []):  # printed bump plates (visual)
            pc = "0.16 0.16 0.17 1" if ob.level >= 5 else "0.95 0.94 0.9 1"
            W.append(f'<geom type="box" size="{(x1_ - x0_) / 2e3:.4f} {(y1_ - y0_) / 2e3:.4f} 0.0008" '
                     f'pos="{(x0_ + x1_) / 2e3:.4f} {(y0_ + y1_) / 2e3:.4f} {ob.floor_z - 0.0006:.5f}" rgba="{pc}" '
                     f'material="matte" contype="0" conaffinity="0" group="2"/>')
        if ob.floor_z > -0.01:  # the white foam-core approach board before the obstacle (visual only)
            W.append(f'<geom type="box" size="0.25 0.152 0.0004" pos="-0.25 0 {ob.floor_z - 0.0003:.5f}" material="board" '
                     f'contype="0" conaffinity="0" group="2"/>')
        self.course = {"floor": ob.floor_surface}
        self._cord = []
        self._cord_specs = []
        self._course_excl, self._course_pairs = [], []
        for s in ob.shapes:
            q = (s.quat[3], s.quat[0], s.quat[1], s.quat[2])
            col = "0.93 0.92 0.89 1"  # the real course boards are white
            if s.name.startswith("bump"):  # printed rods: white PLA, the L5 plates black
                col = "0.16 0.16 0.17 1" if ob.level >= 5 else "0.95 0.94 0.9 1"
            mu_s = p.mu_rod if (p.mu_rod and s.name.startswith("bump")) else p.mu_board  # printed PLA rods: own friction
            fr = f'friction="{mu_s} 0.005 0.0001" priority="1" contype="3" conaffinity="3"'  # all boards: foam core
            if p.board_solref:  # (fit10) hard parts on the course: the foam-core boards / printed parts give a little
                fr += f' solref="{f3(p.board_solref)}"'
            jt = next((j for j in getattr(ob, "joints", []) if j.kind == "revolute" and j.body1 == s.name
                       and not j.body0), None) if (s.dynamic and ob.kind in p.course_dynamic) else None
            if jt is not None and s.kind == "box":
                # a swinging course part: its own body on a hinge at the joint's anchor (axis in the world frame),
                # its real mass; no contact with the other course parts except the floor (its foot rests there)
                Rq = np.zeros(9)
                mujoco.mju_quat2Mat(Rq, np.array(q, float))
                Rq = Rq.reshape(3, 3)
                axw = {"X": np.array([1.0, 0, 0]), "Y": np.array([0, 1.0, 0])}.get(jt.axis.upper(), np.array([0, 0, 1.0]))
                sz = np.array(s.size, float)
                mk = max(float(s.mass_kg), 1e-3)
                Ib = mk / 12.0 * np.array([sz[1] ** 2 + sz[2] ** 2, sz[0] ** 2 + sz[2] ** 2, sz[0] ** 2 + sz[1] ** 2])
                g = (f'<body name="crs_{s.name}" pos="{f3(s.pos)}" quat="{f3(q)}">'
                     f'<joint name="cj_{s.name}" type="hinge" pos="{f3(Rq.T @ (np.array(jt.anchor) - np.array(s.pos)))}" '
                     f'axis="{f3(Rq.T @ axw)}" damping="0.002"/>'
                     f'<inertial pos="0 0 0" mass="{mk:.4g}" diaginertia="{f3(Ib)}"/>'
                     f'<geom name="c_{s.name}" type="box" size="{f3(sz / 2)}" rgba="{col}" material="matte" {fr}/></body>')
                self._course_excl.append((f"crs_{s.name}", "world"))
                self._course_pairs.append(f'<pair geom1="c_{s.name}" geom2="floor" condim="3" '
                                          f'friction="{p.mu_board} {p.mu_board} 0.005 0.0001 0.0001"/>')
                if ob.kind == "seesaw":  # the plank rests on / lands on the LEGO ledges and platforms
                    for o_ in ob.shapes:
                        if o_.kind == "box" and (o_.name.startswith("ledge") or o_.name.startswith("platform")):
                            self._course_pairs.append(f'<pair geom1="c_{s.name}" geom2="c_{o_.name}" condim="3" '
                                                      f'friction="{p.mu_abs} {p.mu_abs} 0.005 0.0001 0.0001"/>')
                self.log.append(f"course part {s.name} swings on its hinge ({mk * 1e3:.0f} g)")
            elif (s.kind == "box" and p.step_nose_mm > 0 and ob.kind == "steps" and s.name.startswith("step")
                  and not any(s.quat[:3])):
                # (fit11) a tread's front edge rounded with radius step_nose_mm: the real treads are 1/2 in foam board
                # whose cut nose is soft foam between paper faces; a claw tip or a tyre pressed on it crushes and rounds
                # it (a mathematically sharp box edge is a perfect hook for a climbing claw)
                rho = p.step_nose_mm * 1e-3
                sz, ps = np.array(s.size, float), np.array(s.pos, float)
                x0_, x1_, z0_, z1_ = ps[0] - sz[0] / 2, ps[0] + sz[0] / 2, ps[2] - sz[2] / 2, ps[2] + sz[2] / 2
                rho = min(rho, 0.45 * sz[0], 0.45 * sz[2])
                parts_ = [(f"c_{s.name}", (x0_ + rho, x1_, z0_, z1_)), (f"c_{s.name}_nl", (x0_, x0_ + rho, z0_, z1_ - rho))]
                g = ""
                for gn_, (a0, a1, b0, b1) in parts_:
                    g += (f'<geom name="{gn_}" type="box" size="{f3([(a1 - a0) / 2, sz[1] / 2, (b1 - b0) / 2])}" '
                          f'pos="{f3([(a0 + a1) / 2, ps[1], (b0 + b1) / 2])}" rgba="{col}" material="matte" {fr}/>')
                g += (f'<geom name="c_{s.name}_nr" type="capsule" size="{rho:.5g} {sz[1] / 2:.5g}" '
                      f'pos="{f3([x0_ + rho, ps[1], z1_ - rho])}" quat="0.7071068 0.7071068 0 0" rgba="{col}" '
                      f'material="matte" {fr}/>')
                self.course[f"c_{s.name}_nl"] = s.surface
                self.course[f"c_{s.name}_nr"] = s.surface
            elif s.kind == "box":
                g = (f'<geom name="c_{s.name}" type="box" size="{f3(np.array(s.size) / 2)}" pos="{f3(s.pos)}" '
                     f'quat="{f3(q)}" rgba="{col}" material="matte" {fr}/>')
            elif p.cord_dynamic and s.kind == "capsule" and s.name.startswith("rope"):
                # a cord segment on a mocap body: it moves with the cord's deflection under the vehicle (_cord_update)
                Rq = np.zeros(9)
                mujoco.mju_quat2Mat(Rq, np.array(q, float))
                Rq = Rq.reshape(3, 3)
                ax_ = Rq @ {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0])}.get(s.axis.lower(), np.array([0, 0, 1.0]))
                c_ = np.array(s.pos, float)
                e0_, e1_ = c_ - ax_ * s.size[1] / 2, c_ + ax_ * s.size[1] / 2
                if p.cord_bodies:  # a dynamic chain instead (built after the loop, _cord_chain)
                    self._cord_specs.append((s.name, e0_, e1_, s.size[0], col, fr))
                    self.course[f"c_{s.name}"] = s.surface
                    continue
                self._cord.append([s.name, e0_, e1_])
                g = (f'<body name="cord_{s.name}" mocap="true" pos="{f3(c_)}" quat="{f3(_z_to(ax_))}">'
                     f'<geom name="c_{s.name}" type="capsule" size="{s.size[0]:.5g} {s.size[1] / 2:.5g}" rgba="{col}" '
                     f'material="matte" {fr}/></body>')
            else:
                r_, L = s.size[0], s.size[1]
                g = (f'<body name="crs_{s.name}" pos="{f3(s.pos)}" quat="{f3(q)}"><geom name="c_{s.name}" type="{s.kind}" '
                     f'size="{r_:.5g} {L / 2:.5g}" quat="{_axis_quat(s.axis.lower())}" rgba="{col}" material="matte" {fr}/></body>')
            if s.dynamic and jt is None:
                self.log.append(f"dynamic course shape {s.name} treated as static")
            W.append(g)
            self.course[f"c_{s.name}"] = s.surface
        self._cord_eqs, self._cord_excl = [], []
        if self._cord_specs:
            Wc, self._cord_eqs, self._cord_excl = self._cord_chain(self._cord_specs)
            W += Wc
        self.joint_of = {}  # cluster root -> joint name

        def part_xml(n, ref):
            """body of part n, posed relative to part `ref` (None = world); its cluster members and hanging clusters"""
            bp = cat[self.parts[n].part]
            pn, Rn = self.world[n]
            if ref is None:
                pos, R = pn, Rn
            else:
                pr, Rr = self.world[ref]
                pos, R = Rr.T @ (pn - pr), Rr.T @ Rn
            m_, com, I = self._inertia(bp)
            x = f'<body name="{n}" pos="{f3(pos)}" quat="{f3(mat_to_wxyz(R))}">'
            if n in clusters and n in tree_parent and tree_parent[n][3] == "rack":
                # (v13) the moving side of a rack slide: a slide joint along the rack in the other side's frame
                rk = self._rack_of_root[n]
                axl = Rn.T @ rk["e"]
                x += (f'<joint name="rk_{n}" type="slide" pos="0 0 0" axis="{f3(axl)}" range="{rk["lo"]:.6g} '
                      f'{rk["hi"]:.6g}" armature="0.002" damping="0.05" solreflimit="{f3(p.slide_stop_solref)}" '
                      f'solimplimit="0.95 0.99 0.0005"/>')
                self.joint_of[n] = f"rk_{n}"
            elif n in clusters:  # a cluster root: its joint
                if n in tree_parent:
                    pp, ci, axp, kind = tree_parent[n]
                    c = d.connections[ci]
                    pts = self._bearing_points(c, axp, pp)
                    cen = (pts[0] + pts[1]) / 2
                    ax = self.world[axp][1][:, 0]
                    loc = Rn.T @ (cen - pn)
                    axl = Rn.T @ ax
                    jn = f"j_{n}"
                    extra = ""
                    if kind == "motor":
                        st = self._motor_stall(pp)
                        extra = f' armature="{p.rotor_j:.6g}" frictionloss="{p.motor_friction * st:.6g}" solreffriction="0.001 1"'
                    elif kind == "pin":  # a friction pin turns in its hole against its ridges' friction
                        fr_ = p.pin_friction_nm * (0.1 if cat[self.parts[axp].part].pivot else 1.0)
                        extra = f' frictionloss="{fr_:.5g}" damping="0.0002" solreffriction="0.001 1"'
                    else:
                        extra = f' frictionloss="{p.axle_fric_nm:.5g}" damping="0.00002" solreffriction="0.002 1"'
                    if n in self._rack_pinions:  # (v13) a rack pinion's shaft
                        extra += f' armature="{p.rack_pinion_armature:.4g}"'
                    x += f'<joint name="{jn}" type="hinge" pos="{f3(loc)}" axis="{f3(axl)}"{extra}/>'
                    self.joint_of[n] = jn
                    # friction pins: held by their ridges; (v13) an idler / differential case turns on its axle
                    sl = self._slide.get(axp) if kind not in ("pin", "idler", "diffcase") else None
                    if sl is not None:  # the axle slides along its holes between whatever stops it
                        fp, fm = sl
                        grip = (p.shaft_grip_n if kind == "motor" else p.pin_slide_n if kind == "pin"
                                else p.axle_slide_fric_n)  # friction pins are held by their ridges
                        x += (f'<joint name="sl_{n}" type="slide" pos="{f3(loc)}" axis="{f3(axl)}" range="{-fm:.5g} {fp:.5g}" '
                              f'frictionloss="{grip:.4g}" armature="0.002" damping="0.05" solreflimit="{f3(p.slide_stop_solref)}" '
                              f'solimplimit="0.95 0.99 0.0005"/>')
                else:
                    if n not in getattr(self, "fixed", ()):  # (scenes: parts bolted to the world)
                        x += "<freejoint/>"
            xm = self._xmass.get(n, 0.0)  # (v13) chain links carried by this sprocket
            sk = self._shocks.get(n)
            if sk is not None:  # (v13) the rod's share of the shock's mass rides on the rod body
                m_, I = m_ * (1.0 - sk["rod_frac"]), I * (1.0 - sk["rod_frac"])
            if xm:
                I = I * (m_ + xm) / m_
                m_ += xm
            x += (f'<inertial pos="{f3(com)}" mass="{m_:.6g}" diaginertia="{f3(I)}"/>' + "".join(self._part_geoms(n, bp))
                  + "".join(self._inject.get(n, [])))
            if n in clusters:
                for mbr in clusters[n]:
                    if mbr != n:
                        x += part_xml(mbr, n) + "</body>"
            rod_kids = []
            for r in children.get(n, []):
                if sk is not None and tree_parent[r][1] in self._rod_ci:
                    rod_kids.append(r)
                    continue
                x += part_xml(r, n) + "</body>"
            if sk is not None:
                # (v13) the shock's rod: a body at the rod end on a slide joint along the shock axis (local z, the rod
                # moving +z when compressed); q = compression from the pose as drawn; spring F = preload + rate x
                # compression (springref), hard stops at full extension and full compression
                q0 = sk["q0"]
                kk = sk["rate"]
                x += (f'<body name="{n}__rod" pos="0 0 0"><joint name="shk_{n}" type="slide" axis="0 0 1" '
                      f'range="{-q0:.6g} {sk["stroke"] - q0:.6g}" stiffness="{kk:.6g}" '
                      f'springref="{-(sk["preload"] / kk) - q0:.6g}" damping="{sk["damping"]:.6g}" armature="0.002" '
                      f'solreflimit="{f3(p.slide_stop_solref)}" solimplimit="0.95 0.99 0.0005"/>'
                      f'<inertial pos="{f3(sk["rod_pos"])}" mass="{sk["rod_mass"]:.6g}" diaginertia="{f3(sk["rod_I"])}"/>'
                      + "".join(self._inject_rod.get(n, [])))
                for r in rod_kids:
                    x += part_xml(r, n) + "</body>"
                x += "</body>"
            return x
        self._slide, self._sliders = {}, {}
        if p.axle_slide and getattr(self, "design_path", None):
            try:
                import dfm  # noqa: PLC0415
                rep = dfm.inspect(self.design_path)
                for cl in rep["clusters"]:
                    self._slide[cl["shaft"]] = (cl["free_plus"] * 1e-3, cl["free_minus"] * 1e-3)
            except Exception as e:  # noqa: BLE001
                self.log.append(f"axle slide off: {e!r}")
        sl_inject = {}
        for ci, c in enumerate(d.connections):  # the other bearings of a sliding axle: a slider in that part
            if c.kind != "hinge" or c.parent not in self.world or c.child not in self.world:
                continue
            axp = c.child if self.kind(c.child) == "axle" else (c.parent if self.kind(c.parent) == "axle" else None)
            if axp is None or axp not in self._slide:
                continue
            other = c.parent if axp == c.child else c.child
            r = root_of[axp]
            if root_of[other] == r or (r in tree_parent and tree_parent[r][1] == ci) or ci in self._v13_skip_eq:
                continue
            pts = self._bearing_points(c, axp, other)
            cen = (pts[0] + pts[1]) / 2
            po, Ro = self.world[other]
            axo = Ro.T @ self.world[axp][1][:, 0]
            name = f"sld{ci}"
            (self._inject_rod if (ci in self._rod_ci and other in self._shocks) else sl_inject).setdefault(other, []).append(
                f'<body name="{name}" pos="{f3(Ro.T @ (cen - po))}"><joint type="slide" axis="{f3(axo)}" armature="0.002"/>'
                f'<inertial pos="0 0 0" mass="1e-4" diaginertia="1e-9 1e-9 1e-9"/></body>')
            self._sliders[ci] = (name, cen)
        self._ir = None
        self._inject, cable_ends = self._cables_plan()
        self._v13_inject()  # (v13) tread rollers on their carriers
        for k_, v_ in sl_inject.items():
            self._inject.setdefault(k_, []).extend(v_)
        for k_, v_ in getattr(self, "extra_inject", {}).items():  # (scenes: sites, markers on parts)
            self._inject.setdefault(k_, []).extend(v_)
        for r in clusters:
            if r not in tree_parent:
                W.append(part_xml(r, None) + "</body>")
        self.body_names = names
        for members in clusters.values():
            for i_, a_ in enumerate(members):
                X += [(a_, b_) for b_ in members[i_ + 1:]]
        # ---- equality constraints
        self.studs: list[StudEq] = []
        self._bear_eqs: list[int] = []
        eqi = 0
        for ci, c in enumerate(d.connections):
            a, b = c.parent, c.child
            if a not in self.world or b not in self.world:
                continue
            X.append((a, b))
            if c.kind == "stud":
                pts, up, lo = self._stud_points(a, b)
                pl, Rl = self.world[lo]
                pu, Ru = self.world[up]
                zl = Rl[:, 2]
                P = np.array(pts)
                # point constraints per stud: a 2-D stud pattern resists rotation by its spacing (one point per stud);
                # a single row of studs gets two points per stud across the row (the stud's grip in its tube), a single
                # stud four (both directions)
                if len(P) >= 3 and np.linalg.matrix_rank(P - P.mean(axis=0), tol=1e-3) >= 2:
                    offs = [np.zeros(3)]
                else:
                    if len(P) >= 2:
                        row = P[-1] - P[0]
                        row -= zl * float(np.dot(row, zl))
                        row /= np.linalg.norm(row)
                        perp = [np.cross(zl, row)]
                    else:
                        perp = [Rl[:, 0], Rl[:, 1]]
                    offs = [sg * p.stud_r_m * v for v in perp for sg in (-1.0, 1.0)]
                elec = any(cat[self.parts[n_].part].kind in ("motor", "battery", "receiver") for n_ in (a, b))
                for q_ in pts:
                    st = StudEq(eqi, up, lo, ci)
                    st.pull = p.stud_pull_n if elec else p.stud_pull_brick_n
                    st.pt = tuple(float(v) for v in q_)
                    st.eqs = []
                    for o_ in offs:
                        E.append(f'<connect body1="{up}" body2="{lo}" anchor="{f3(Ru.T @ (q_ + o_ - pu))}" '
                                 f'solref="{f3(p.stud_solref)}" solimp="{f3(p.stud_solimp)}"/>')
                        st.eqs.append(eqi)
                        eqi += 1
                    self.studs.append(st)
            elif c.kind in ("hinge", "motor", "pin"):
                ka = self.kind(b)
                axp = b if ka in ("axle", "pin") else a
                other = a if axp == b else b
                r = root_of[axp]
                if root_of[other] == r:
                    continue
                if r in tree_parent and tree_parent[r][1] == ci:
                    continue  # the tree hinge
                if ci in self._v13_skip_eq:  # (v13) an idler's / differential case's own hinge
                    continue
                bo_ = self._body_at(other, ci)  # (v13) a connection into a shock's rod end holds the rod body
                if c.kind == "pin" and self.kind(axp) == "pin":  # the pin's far end is held fast in its hole
                    E.append(f'<weld body1="{bo_}" body2="{axp}" solref="{f3(p.bearing_solref)}" solimp="{f3(p.bearing_solimp)}" torquescale="1"/>')
                    self._bear_eqs.append(eqi)
                    eqi += 1
                    continue
                if ci in self._sliders:  # through its slider: the axle may slide along this hole
                    name, cen = self._sliders[ci]
                    po, Ro = self.world[other]
                    for q_ in self._bearing_points(c, axp, other):
                        E.append(f'<connect body1="{name}" body2="{axp}" anchor="{f3(Ro.T @ (q_ - cen))}" '
                                 f'solref="{f3(p.bearing_solref)}" solimp="{f3(p.bearing_solimp)}"/>')
                        self._bear_eqs.append(eqi)
                        eqi += 1
                    continue
                for q_ in self._bearing_points(c, axp, other):
                    po, Ro = self.world[other]
                    E.append(f'<connect body1="{bo_}" body2="{axp}" anchor="{f3(Ro.T @ (q_ - po))}" '
                             f'solref="{f3(p.bearing_solref)}" solimp="{f3(p.bearing_solimp)}"/>')
                    self._bear_eqs.append(eqi)
                    eqi += 1
            elif c.kind != "cross":
                self.log.append(f"connection kind {c.kind} not modelled ({a}-{b})")
        # ---- gear meshes: joint equalities between the hinges
        self.meshes = []
        self._tendon_xml = []
        pairs = [(gm.gear_a, gm.gear_b, "gear") for gm in d.gear_meshes]
        pairs += [(ch.gears[0], ch.gears[1], "chain") for ch in (getattr(d, "chains", None) or [])]
        for ga, gb, kind in pairs:
            if ga not in self.parts or gb not in self.parts:
                continue
            ba, bb = cat[self.parts[ga].part], cat[self.parts[gb].part]
            g = Mesh(ga, gb, ba.teeth / 2e3, bb.teeth / 2e3, ba.width or 8.0, bb.width or 8.0, ba.teeth, kind)
            g.dbl = all("double bevel" in (x.title or "") for x in (ba, bb))
            g.same = ("double bevel" in (ba.title or "")) == ("double bevel" in (bb.title or ""))
            g.spur = not any(x.part_id in ("32072", "4716") or "meshes at right angles" in (x.title or "")
                             or "half bevel" in (x.title or "") for x in (ba, bb))
            if p.knob_r_mm and ba.part_id == "32072" and bb.part_id == "32072":  # (v13) knob wheels: their knobs
                g.ra = g.rb = p.knob_r_mm * 1e-3  # push at the contact radius (1 : 1), not at z / 2 = 2 mm
            if p.diff_model and (ga in self._diff_cases or gb in self._diff_cases):
                # (v13) a differential's 28T ring: a double bevel on the case, its pitch plane off the case origin
                g.ax_built = True
                o_ = bb if ga in self._diff_cases else ba
                g.same = "double bevel" in (o_.title or "")
                g.dbl = g.same
            if p.worm_lock_k > 0 and "4716" in (ba.part_id, bb.part_id):
                g.worm = "a" if ba.part_id == "4716" else "b"  # (v13) the worm side of a self-locking mesh
            X.append((ga, gb))
            ra_, rb_ = root_of[g.a], root_of[g.b]
            if ra_ not in self.joint_of or rb_ not in self.joint_of or not (
                    self._is_hinge(ra_) and self._is_hinge(rb_)):
                self.log.append(f"mesh {g.a}-{g.b}: a gear's axle has no hinge; not modelled")
                continue
            na, nb = self.world[g.a][1][:, 1], self.world[g.b][1][:, 1]
            jaw = self.world[tree_parent[ra_][2]][1][:, 0]
            jbw = self.world[tree_parent[rb_][2]][1][:, 0]
            # joint axes in the world at spawn (the hinge axis follows the axle's local x)
            g.sa = 1.0 if float(np.dot(na, jaw)) > 0 else -1.0
            sb = 1.0 if float(np.dot(nb, jbw)) > 0 else -1.0
            sab = 1.0 if float(np.dot(na, nb)) > 0 else -1.0
            if kind != "chain" and abs(float(np.dot(na, nb))) < 0.5:
                # crossed axes (worm, bevels, knob wheels): sign(na . nb) is rounding noise (~1e-16) there; the sense
                # is the geometry's (which side of the other's axle each gear sits on): w_b.nb = s (za / zb) w_a.na
                pa_w, pb_w = self.world[g.a][0], self.world[g.b][0]
                if _faces_away(self.parts[ga].part, pa_w, na, pb_w) or _faces_away(self.parts[gb].part, pb_w, nb, pa_w):
                    self.log.append(f"mesh {g.a}-{g.b}: a single bevel's teeth face away from the other gear; not meshed")
                    continue
                sab = -crossed_mesh_sense(pa_w, na, ba.teeth, pb_w, nb, bb.teeth)
            # gear: external mesh, the pitch points move oppositely; chain: both sprockets turn the same way
            g.a1 = (1.0 if kind == "chain" else -1.0) * (g.rb / g.ra) * sab * sb * g.sa
            E.append(f'<joint joint1="{self.joint_of[ra_]}" joint2="{self.joint_of[rb_]}" '
                     f'polycoef="0 {g.a1:.7g} 0 0 0" solref="{f3(p.mesh_solref)}"/>')
            g.eq = eqi
            eqi += 1
            if p.mesh_clutch:  # the slip clutch: tendon length qa - a1 qb, its friction loss set each step (_mesh_update)
                g.tendon = f"mesh{len(self.meshes)}"
                self._tendon_xml.append(
                    f'<fixed name="{g.tendon}" frictionloss="0" solreffriction="{f3(p.mesh_solref)}" '
                    f'solimpfriction="{p.mesh_clutch_imp:.6g} {p.mesh_clutch_imp:.6g} 0.001"><joint joint="{self.joint_of[ra_]}" coef="1"/>'
                    f'<joint joint="{self.joint_of[rb_]}" coef="{-g.a1:.7g}"/></fixed>')
            self.meshes.append(g)
        eqi = self._v13_equalities(E, X, eqi)  # (v13) treads, racks, CV joints, differentials
        # rocking only matters where a part's studs all lie in one row (it can hinge about the row)
        by_part = {}
        for st in self.studs:
            by_part.setdefault(st.upper, []).append(st.pt)
            by_part.setdefault(st.lower, []).append(st.pt)

        def _row(pts):
            P = np.array(pts)
            return len(P) < 3 or np.linalg.matrix_rank(P - P.mean(axis=0), tol=1.5e-3) < 2
        other = {}
        for c in d.connections:  # pins, axles, cross fits also hold a part against hinging about a stud row
            if c.kind != "stud":
                other[c.parent] = other.get(c.parent, 0) + 1
                other[c.child] = other.get(c.child, 0) + 1
        for st in self.studs:  # (a joint of a single stud always rocks: its tube is all that holds it square)
            st.rock = len(st.eqs) == 4 or any(_row(by_part[n_]) and not other.get(n_) for n_ in (st.upper, st.lower))
        # ---- motors: actuators on the shaft hinges
        self.motors = {}
        _, rel, _ = motor_trains(d)
        driving = wheel_motors(d)
        direction = self.stats.get("motor_direction", 1) or 1
        for r, (pp, ci, axp, kind) in tree_parent.items():
            if kind != "motor":
                continue
            st = self._motor_stall(pp)
            w0 = self._motor_w0(pp)
            if p.servo_pos and self.parts[pp].part == "99498c01":  # (v13) the servo's measured drive torque
                st = p.servo_torque_nm
            # the hinge axis is the shaft's local x; the program's sign is about the motor joint as in the benchmark
            lim = (f' forcelimited="true" forcerange="{-p.motor_i_max * st:.6g} {p.motor_i_max * st:.6g}"'
                   if p.motor_i_max > 0 else "")  # (fit11) the receiver output's current limit caps the torque
            ACT.append(f'<general name="a_{pp}" joint="{self.joint_of[r]}" gainprm="{st:.6g}" '
                       f'biastype="affine" biasprm="0 0 {-st / w0:.6g}" ctrlrange="-1 1"{lim}/>')
            self.motors[pp] = dict(shaft=r, joint=self.joint_of[r], act=len(self.motors), stall=st, w0=w0,
                                   sign=(direction if pp in driving else 1) * rel.get(pp, 1), mode="float")
            pid_ = self.parts[pp].part
            if p.servo_pos and pid_ == "99498c01":  # (v13) a position servo
                self.motors[pp]["servo"] = True
            if p.pullback_model and pid_ == "12787c01":  # (v13) a spring motor
                self.motors[pp]["pullback"] = self._pullback_spec(pp, r, axp)
        # ---- tyres
        for n in names:
            bp = cat[self.parts[n].part]
            if bp.kind == "wheel" and bp.tyre_mu:
                for gname, surf in self.course.items():
                    mu = p.mu_tyre_ramp if surf == "rubber_mat" else p.mu_tyre_floor
                    if gname.startswith("c_bump") and p.mu_tyre_rod:  # printed PLA rods: smoother than the boards
                        mu = p.mu_tyre_rod
                    if p.tyre_mu_exp:  # (fit11) this tyre's own measured grip relative to the 32020c01 balloon tyre
                        mu *= (bp.tyre_mu / p.tyre_mu_ref) ** p.tyre_mu_exp
                    sr = p.cord_tyre_solref if (surf == "cord" and p.cord_tyre_solref) else p.tyre_solref
                    if p.tyre_crr > 0 and surf != "cord":  # (v14) rolling resistance (not on the cord: a taut string)
                        rl = p.tyre_crr * bp.wheel_radius * 1e-3
                        C.append(f'<pair geom1="{n}__c" geom2="{gname}" condim="6" friction="{mu} {mu} '
                                 f'{p.tyre_torsion_m:.5g} {rl:.5g} {rl:.5g}" solref="{f3(sr)}"/>')
                    else:
                        C.append(f'<pair geom1="{n}__c" geom2="{gname}" condim="4" friction="{mu} {mu} {p.tyre_torsion_m:.5g} 0.0001 0.0001" '
                                 f'solref="{f3(sr)}"/>')
        C += self._v13_pairs()  # (v13) tread rollers on the course
        self.cables = []
        for cb in cable_ends:  # the plug end: a point constraint on its pad (it pops off above plug_pull_n)
            E.append(f'<connect body1="{cb["pfx"]}B_last" body2="{cb["tgt"]}" anchor="{cb["seg"]:.6g} 0 0"/>')
            cb["eq"] = eqi
            eqi += 1
            for k_, near in cb.get("tape", []):  # tape: the cable body held where it lies, on that part
                E.append(f'<connect body1="{cb["pfx"]}B_{k_}" body2="{near}" anchor="0 0 0" solref="0.01 1"/>')
                eqi += 1
            self.cables.append(cb)
        A += [f'<mesh name="{k_}" vertex="{v_}"/>' for k_, v_ in self._tyre_assets.items()]
        E += self._cord_eqs  # (after every indexed equality: studs, bearings, meshes, cables)
        X += self._cord_excl + self._course_excl
        C += self._course_pairs
        self._parts_xml = (A, W, E, X, C, ACT)
        self._compile(X)
        mujoco.mj_forward(self.m, self.d)
        extra = set()
        for k in range(self.d.ncon):
            con = self.d.contact[k]
            if con.dist < self.p.spawn_touch_m:  # overlapping, or (spawn_touch_m > 0) touching flush: assembled neighbours
                b1, b2 = self.m.geom_bodyid[con.geom1], self.m.geom_bodyid[con.geom2]
                n1, n2 = self.m.body(b1).name, self.m.body(b2).name
                if n1 in self.parts and n2 in self.parts:
                    extra.add((n1, n2))
        if self.p.spawn_touch_m > 0:  # flush neighbours (face to face, zero gap) make no contact at spawn: find them by distance
            m_, d_ = self.m, self.d
            veh = {self.m.body(n).id: n for n in self.parts if n in self.world}
            gs = [g for g in range(m_.ngeom) if int(m_.geom_bodyid[g]) in veh and m_.geom_contype[g] | m_.geom_conaffinity[g]]
            fr = np.zeros(6)
            for i_, g1 in enumerate(gs):
                for g2 in gs[i_ + 1:]:
                    b1, b2 = int(m_.geom_bodyid[g1]), int(m_.geom_bodyid[g2])
                    if b1 == b2 or (veh[b1], veh[b2]) in extra or (veh[b2], veh[b1]) in extra:
                        continue
                    if np.linalg.norm(d_.geom_xpos[g1] - d_.geom_xpos[g2]) > m_.geom_rbound[g1] + m_.geom_rbound[g2] + 1e-3:
                        continue
                    if mujoco.mj_geomDistance(m_, d_, g1, g2, 2e-3, fr) < self.p.spawn_touch_m:
                        extra.add((veh[b1], veh[b2]))
        if extra:
            self.log.append(f"{len(extra)} part pairs overlap or touch at spawn: no contact between them")
            self._compile(X + sorted(extra))
        mujoco.mj_forward(self.m, self.d)
        self.bid = {n: self.m.body(n).id for n in names}
        for g in self.meshes:
            g.ja = int(self.m.jnt_dofadr[self.m.joint(self.joint_of[root_of[g.a]]).id])
            g.jb = int(self.m.jnt_dofadr[self.m.joint(self.joint_of[root_of[g.b]]).id])
            if self.p.mesh_clutch:  # the clutch tendon carries the mesh; its joint equality stays off
                g.tid = int(self.m.tendon(g.tendon).id)
                self.m.eq_active0[g.eq] = 0
                self.d.eq_active[g.eq] = 0
        for mo in self.motors.values():
            mo["dof"] = int(self.m.jnt_dofadr[self.m.joint(mo["joint"]).id])
            mo["bias"] = float(self.m.actuator_biasprm[mo["act"], 2])
            self.m.actuator_biasprm[mo["act"], 2] = 0.0  # float until commanded
            if mo.get("servo") or mo.get("pullback"):  # (v13) controlled per step (_v13_step), no back-EMF term
                mo["bias"] = 0.0
        self.n_eq = eqi
        self._v13_init()
        for st in self.studs:
            st.base = [self.m.eq_data[e, 3:6].copy() for e in st.eqs]  # anchor on the lower part (its frame)
        self._stiffen()

    # ================================================================ (v13) mechanisms
    def _body_at(self, part, ci):
        """the body a connection holds: a shock's rod body for a connection into its rod end, else the part"""
        return f"{part}__rod" if (part in self._shocks and ci in self._rod_ci) else part

    def _is_hinge(self, root):
        tp = self.tree_parent.get(root) if hasattr(self, "tree_parent") else None
        return tp is not None and tp[3] != "rack"

    def _hinge_axis_w(self, root):
        """world axis of a cluster's hinge at spawn (its tree axle's local x)"""
        return self.world[self.tree_parent[root][2]][1][:, 0]

    def _v13_plan(self):
        """(v13) what the design's mechanism fields ask of the model, before the clusters and the tree are built"""
        p, d, cat = self.p, self.design, self.cat
        self._v13_merge, self._v13_nobear, self._v13_skip_eq = [], set(), set()
        self._shocks, self._rod_ci, self._inject_rod, self._xmass = {}, set(), {}, {}
        self._diff_cases, self._diffs, self._idlers, self._racks_spec = set(), [], {}, []
        self._rack_of_root, self._treads, self._cvs, self._rk = {}, [], [], []
        self._tread_eq, self._tread_geoms, self._rack_pinions = [], [], set()
        names = set(self.parts)
        Pd = None
        if p.chain_link_mass_g > 0:  # the links' mass, half on each sprocket
            for ch in getattr(d, "chains", None) or []:
                for g_ in ch.gears:
                    if g_ in names:
                        self._xmass[g_] = self._xmass.get(g_, 0.0) + 0.5 * ch.links * p.chain_link_mass_g * 1e-3
        if p.shock_model and any(pt.part in ("731c04", "2909c03") for pt in self.parts.values()):
            from ai_pd_tasks.lego_bench import inventory_r4b as R4  # noqa: PLC0415
            Pd = place(d)
            comp = R4.shock_compression(d, Pd)
            for ci, (sh, side) in R4.shock_sides(d, Pd).items():
                if side == "rod":
                    self._rod_ci.add(ci)
            for n, pt in self.parts.items():
                spec = R4.SHOCKS.get(pt.part)
                if spec is None:
                    continue
                m = max(cat[pt.part].mass_g, 0.15) * 1e-3
                frac = 0.33 if pt.part == "731c04" else 0.30  # the rod's share of the hull (inventory_r4b)
                q0 = float(comp.get(n, 0.0)) * 1e-3
                rp = np.array(spec["rod_hole"], float) * 1e-3 + np.array([0.0, 0.0, q0 + 0.01])
                rm = m * frac
                self._shocks[n] = dict(rate=spec["rate_n_per_mm"] * 1e3, preload=spec["preload_n"],
                                       stroke=spec["stroke_mm"] * 1e-3, q0=q0, damping=spec["damping_n_s_per_m"],
                                       rod_frac=frac, rod_mass=rm, rod_pos=rp,
                                       rod_I=np.array([rm * 1e-4, rm * 1e-4, rm * 2e-5]))
        if p.idler_hinge:  # a round-bore gear on an axle (hinge): its own hinge about the axle
            for ci, c in enumerate(d.connections):
                if c.kind != "hinge" or c.parent not in names or c.child not in names:
                    continue
                for g_, a_ in ((c.child, c.parent), (c.parent, c.child)):
                    bg, ba = cat[self.parts[g_].part], cat[self.parts[a_].part]
                    if bg.kind == "gear" and "round bore" in (bg.title or "") and ba.kind == "axle" and g_ not in self._idlers:
                        self._idlers[g_] = (a_, ci)
                        self._v13_nobear.add(ci)
        if p.diff_model:
            from ai_pd_tasks.lego_bench import inventory_r4b as R4  # noqa: PLC0415
            for df in getattr(d, "differentials", None) or []:
                if df.case not in names:
                    continue
                axles = R4.diff_axles(d, df)
                if any(a_ is None or a_ not in names for a_ in axles):
                    self.log.append(f"differential {df.name}: output axles not found; not modelled")
                    continue
                cis = [ci for ci, c in enumerate(d.connections) if c.kind == "hinge" and df.case in (c.parent, c.child)
                       and ({c.parent, c.child} - {df.case}) <= set(axles)]
                self._diffs.append(dict(name=df.name, case=df.case, axles=axles, case_ci=cis))
                self._diff_cases.add(df.case)
                self._v13_nobear |= set(cis)
                for s_ in df.satellites:
                    if s_ in names:
                        self._v13_merge.append((df.case, s_))
        if p.cv_model:
            for cv in getattr(d, "cv_joints", None) or []:
                if cv.ball in names and cv.cup in names:
                    self._cvs.append(cv)
        if p.rack_slide and getattr(d, "racks", None):
            from ai_pd_tasks.lego_bench import racks as RK  # noqa: PLC0415
            Pd = Pd or place(d)
            for g in RK.rack_geoms(d, Pd):
                if g.errors:
                    self.log.append(f"rack slide {g.slide.name}: {g.errors[0]}; not modelled")
                    continue
                tr, _e, _w = RK.travel(d, g, Pd)
                self._racks_spec.append(dict(g=g, tr=tr, Rd=Pd[g.slide.racks[0]].R))
        if p.tread_model and getattr(d, "treads", None):
            from ai_pd_tasks.lego_bench import treads as TR  # noqa: PLC0415
            Pd = Pd or place(d)
            for g in TR.tread_geoms(d, Pd):
                if g.errors:
                    self.log.append(f"tread {g.tread.name}: {g.errors[0]}; not modelled")
                    continue
                self._treads.append(dict(g=g, carrier=TR.carrier(g, d, Pd) or g.bearing_a.parent if g.bearing_a else ""))

    def _v13_tree(self, tree_parent, root_of, ancestors):
        """(v13) idler gears, differential cases and rack slides in the kinematic tree"""
        for g_, (a_, ci) in self._idlers.items():
            r = root_of[g_]
            if r in tree_parent or root_of[a_] == r or r in ancestors(root_of[a_]):
                self.log.append(f"idler {g_}: no own hinge (its cluster is already jointed)")
                self._v13_nobear.discard(ci)
                continue
            if any(self.kind(m) in ("axle", "pin") for m in self.parts if root_of[m] == r):
                continue
            tree_parent[r] = (a_, ci, a_, "idler")
            self._v13_skip_eq.add(ci)
        for dd in self._diffs:
            cr = root_of[dd["case"]]
            found = None
            for ax in dd["axles"]:
                tp = tree_parent.get(root_of[ax])
                if tp is not None and tp[3] in ("hinge", "motor") and root_of[tp[0]] != cr:
                    found = tp
                    break
            if found is None or cr in tree_parent or cr in ancestors(root_of[found[0]]) or root_of[found[0]] == cr:
                self.log.append(f"differential {dd['name']}: no output axle in a bearing of its own; not modelled")
                dd["ok"] = False
                continue
            tree_parent[cr] = (found[0], found[1], found[2], "diffcase")
            self._v13_skip_eq |= set(dd["case_ci"])
            dd["ok"] = True
        for k, rs in enumerate(self._racks_spec):
            g = rs["g"]
            rack_moves = self.design.chassis not in g.rack_side
            moving = set(g.rack_side) if rack_moves else ({pt.name for pt in self.design.parts} - set(g.rack_side))
            anchor = g.slide.racks[0] if rack_moves else g.slide.guides[0]
            host = g.slide.guides[0] if rack_moves else g.slide.racks[0]
            # the slide joint goes on the moving side's best-connected free body (its carrier: the tooth force enters
            # the structure there, as it does through the racks' studs on their carrier), the first rack failing that
            deg = {}
            for c in self.design.connections:
                if c.parent in moving and c.child in moving:
                    for n_ in (c.parent, c.child):
                        deg[root_of[n_]] = deg.get(root_of[n_], 0) + 1
            cands = sorted({root_of[n] for n in moving if n in root_of}, key=lambda r: (-deg.get(r, 0), r))
            cands = [r for r in cands if r != root_of[anchor]] + [root_of[anchor]]
            mr = next((r for r in cands if r not in tree_parent and root_of[host] != r
                       and r not in ancestors(root_of[host]) and r not in self._diff_cases), None)
            if mr is None:
                self.log.append(f"rack slide {g.slide.name}: no free body on its moving side; not modelled")
                continue
            Rw = self.world[g.slide.racks[0]][1]
            e_w = Rw @ (rs["Rd"].T @ np.asarray(g.e, float))
            n_w = Rw @ (rs["Rd"].T @ np.asarray(g.n, float))
            tree_parent[mr] = (host, -1, None, "rack")
            lo_, hi_ = rs["tr"].lo * 1e-3, rs["tr"].hi * 1e-3
            if hi_ - lo_ < 1e-4:
                lo_, hi_ = lo_ - 5e-5, hi_ + 5e-5
            self._rack_of_root[mr] = dict(e=e_w if rack_moves else -e_w, lo=lo_, hi=hi_)
            self._rack_pinions.add(root_of.get(g.slide.pinion))
            self._rk.append(dict(name=g.slide.name, root=mr, g=g, tr=rs["tr"], e_w=e_w, n_w=n_w, moving=moving,
                                 rack_moves=rack_moves))

    def _v13_inject(self):
        """(v13) tread rollers: bodies on hinges in the tread's carrier, along the bottom run between the sprockets"""
        from ai_pd_tasks.lego_bench import treads as TR  # noqa: PLC0415
        p = self.p
        for k, td in enumerate(self._treads):
            g, car = td["g"], td["carrier"]
            sa, sb = g.tread.sprockets
            ra, rb = self.root_of[sa], self.root_of[sb]
            master = sa if self._is_hinge(ra) else (sb if self._is_hinge(rb) else None)
            if master is None or car not in self.world:
                self.log.append(f"tread {g.tread.name}: no sprocket on a hinge (or no carrier); not modelled")
                continue
            L = TR.TREAD_LINKS[g.tread.link]
            ax = self._hinge_axis_w(self.root_of[master])
            ca, cb = self.world[sa][0], self.world[sb][0]
            dv = cb - ca
            dv = dv - ax * float(np.dot(dv, ax))
            D = float(np.linalg.norm(dv))
            nr = max(1, math.ceil(D * 1e3 / p.tread_spacing_mm - 1e-9))
            pc, Rc = self.world[car]
            r_t, r_p = g.r_tip * 1e-3, g.r_pitch * 1e-3
            mr = g.tread.links * L["mass_g"] * 1e-3 / (nr + 1)
            Ia = max(0.5 * mr * r_t * r_t, 1e-9)
            names_ = []
            for i in range(nr + 1):
                pos = ca + dv * i / nr
                bn = f"trd{k}_{i}"
                self._inject.setdefault(car, []).append(
                    f'<body name="{bn}" pos="{f3(Rc.T @ (pos - pc))}"><joint name="{bn}_j" type="hinge" '
                    f'axis="{f3(Rc.T @ ax)}" armature="1e-7" damping="1e-6"/>'
                    f'<inertial pos="0 0 0" mass="{mr:.6g}" diaginertia="{Ia:.4g} {Ia:.4g} {Ia:.4g}"/>'
                    f'<geom name="{bn}_g" type="cylinder" size="{r_t:.5g} {g.half_w * 1e-3:.5g}" '
                    f'quat="{f3(_z_to(Rc.T @ ax))}" contype="0" conaffinity="0" group="3" rgba="0.1 0.1 0.1 1"/></body>')
                names_.append(bn)
            td.update(ok=True, rollers=names_, master=master, other=(sb if master == sa else sa), ratio=r_p / r_t,
                      mu=getattr(p, f"tread_mu_{g.tread.link}"), tyre_mu=L.get("tyre_mu", 0.23), ax=ax)
            self.log.append(f"tread {g.tread.name}: {nr + 1} rollers r {g.r_tip:.2f} mm on {car}, belt at "
                            f"r {g.r_pitch:.2f} mm, links {g.tread.links * L['mass_g']:.1f} g")

    def _v13_pairs(self):
        out = []
        for td in self._treads:
            if not td.get("ok"):
                continue
            for bn in td["rollers"]:
                for gname, surf in self.course.items():
                    mu = td["mu"]
                    if not mu:  # the link's measured grip, mapped as a tyre's (_build: tyres)
                        p_ = self.p
                        mu = p_.mu_tyre_ramp if surf == "rubber_mat" else p_.mu_tyre_floor
                        if gname.startswith("c_bump") and p_.mu_tyre_rod:
                            mu = p_.mu_tyre_rod
                        if p_.tyre_mu_exp:
                            mu *= (td["tyre_mu"] / p_.tyre_mu_ref) ** p_.tyre_mu_exp
                    elif gname.startswith("c_bump") and self.p.mu_rod:  # printed PLA rods
                        mu *= self.p.mu_rod / self.p.mu_board
                    td.setdefault("mu_eff", {})[surf] = round(mu, 3)
                    out.append(f'<pair geom1="{bn}_g" geom2="{gname}" condim="3" friction="{mu:.4g} {mu:.4g} 0.005 '
                               f'0.0001 0.0001"/>')
        return out

    def _v13_equalities(self, E, X, eqi):
        """(v13) the couplings: tread rollers and sprockets, rack and pinion, CV joints, differentials"""
        p, root_of = self.p, self.root_of
        sol = f'solref="{f3(p.mesh_solref)}"'
        hard = 'solimp="0.9999 0.9999 0.001 0.5 2"'  # (light shafts: the default impedance leaves these couplings soft)
        for td in self._treads:
            if not td.get("ok"):
                continue
            jm = self.joint_of[root_of[td["master"]]]
            for bn in td["rollers"]:  # the run's surface moves at the belt speed: w_roller = w_sprocket r_pitch / r_tip
                E.append(f'<joint joint1="{bn}_j" joint2="{jm}" polycoef="0 {td["ratio"]:.7g} 0 0 0" {sol} {hard}/>')
                eqi += 1
            ro = root_of[td["other"]]
            if self._is_hinge(ro) and ro != root_of[td["master"]]:  # the belt turns both sprockets alike
                s_ = 1.0 if float(np.dot(self._hinge_axis_w(ro), td["ax"])) > 0 else -1.0
                E.append(f'<joint joint1="{self.joint_of[ro]}" joint2="{jm}" polycoef="0 {s_:g} 0 0 0" {sol} {hard}/>')
                eqi += 1
        for rk in self._rk:
            g = rk["g"]
            rp = root_of.get(g.slide.pinion)
            if rp is None or not self._is_hinge(rp):
                self.log.append(f"rack slide {rk['name']}: the pinion's axle has no hinge; the slide is free")
                continue
            a_w = self._hinge_axis_w(rp)
            r_eff = g.r_eff * 1e-3
            # the rack moves with the pinion's tooth at the contact (the pitch line, -n from the pinion centre)
            rk["c"] = -r_eff * float(np.dot(np.cross(a_w, rk["n_w"]), rk["e_w"]))
            # (a light pinion against a heavy slide: the default impedance left the coupling ~1 N/m soft; near-hard)
            E.append(f'<joint joint1="rk_{rk["root"]}" joint2="{self.joint_of[rp]}" polycoef="0 {rk["c"]:.7g} 0 0 0" '
                     f'{sol} solimp="0.9999 0.9999 0.001 0.5 2"/>')
            rk["eq"] = eqi
            eqi += 1
            bp = self.cat[self.parts[g.slide.pinion].part]
            z = int(bp.teeth)
            rk["cap"] = p.mesh_cap_n * min(1.0, (z / 12.0) ** p.mesh_cap_z_exp) * (
                p.mesh_mixed_k if "double bevel" in (bp.title or "") else 1.0)
            rk.update(state="locked", skips=0, peak=0.0, pinion_root=rp)
            stops = {k_.split(":", 1)[1] for k_ in (rk["tr"].lo_kind, rk["tr"].hi_kind) if k_.startswith("stop:")}
            frame = (set(g.guide_rigid or ()) | set(g.slide.guides)) - stops - rk["moving"]
            for m_ in rk["moving"]:  # the guide frame carries the slide (the joint); the pinion's teeth are the coupling
                for gd in frame:
                    X.append((m_, gd))
            for r_ in g.slide.racks:
                X.append((r_, g.slide.pinion))
        for cv in self._cvs:
            rb_, rc_ = root_of[cv.ball], root_of[cv.cup]
            if rb_ == rc_ or not (self._is_hinge(rb_) and self._is_hinge(rc_)):
                self.log.append(f"cv joint {cv.name}: a shaft has no hinge; not modelled")
                continue
            s_ = 1.0 if float(np.dot(self._hinge_axis_w(rb_), self._hinge_axis_w(rc_))) > 0 else -1.0
            E.append(f'<joint joint1="{self.joint_of[rc_]}" joint2="{self.joint_of[rb_]}" polycoef="0 {s_:g} 0 0 0" {sol} {hard}/>')
            eqi += 1
            E.append(f'<connect body1="{cv.ball}" body2="{cv.cup}" anchor="0 0 0" solref="{f3(p.bearing_solref)}" '
                     f'solimp="{f3(p.bearing_solimp)}"/>')  # the ball held in its cup
            eqi += 1
            X.append((cv.ball, cv.cup))
        for k, dd in enumerate(self._diffs):
            if not dd.get("ok"):
                continue
            cr = root_of[dd["case"]]
            ref = self._hinge_axis_w(cr)
            js = []
            for ax in dd["axles"]:
                r_ = root_of[ax]
                if not self._is_hinge(r_):
                    js = None
                    break
                js.append((self.joint_of[r_], 1.0 if float(np.dot(self._hinge_axis_w(r_), ref)) > 0 else -1.0))
            if not js:
                self.log.append(f"differential {dd['name']}: an output axle has no hinge; not modelled")
                continue
            tn = f"diff{k}"
            self._tendon_xml.append(f'<fixed name="{tn}">' + "".join(f'<joint joint="{j}" coef="{s:g}"/>' for j, s in js)
                                    + f'<joint joint="{self.joint_of[cr]}" coef="-2"/></fixed>')
            E.append(f'<tendon tendon1="{tn}" {sol} {hard}/>')  # w_L + w_R = 2 w_case
            eqi += 1
            grp = [dd["case"], *dd["axles"]]
            X += [(a_, b_) for i_, a_ in enumerate(grp) for b_ in grp[i_ + 1:]]
        return eqi

    def _pullback_spec(self, pp, root, axp):
        from ai_pd_tasks.lego_bench import inventory_r4b as R4  # noqa: PLC0415
        e = next((x for x in (getattr(self.design, "pullback", None) or []) if x.motor == pp), None)
        wind = float(e.wind_turns) if e is not None else self.p.pullback_turns
        if e is not None and e.sense:  # about the motor's output axis (its local y) -> about the joint axis
            sj = float(e.sense) * (1.0 if float(np.dot(self.world[pp][1][:, 1], self.world[axp][1][:, 0])) > 0 else -1.0)
        else:
            try:
                sj = float(R4.drive_sense(self.design, pp))
            except Exception:  # noqa: BLE001
                sj = 1.0
        full = self.p.pullback_turns * 2 * math.pi
        return dict(sense=sj, theta=full - min(max(wind, 0.0), self.p.pullback_turns) * 2 * math.pi, full=full,
                    released=False)

    def _v13_init(self):
        m = self.m
        for rk in self._rk:
            if "eq" not in rk:
                continue
            rk["dof"] = int(m.jnt_dofadr[m.joint(f"rk_{rk['root']}").id])
            rk["qadr"] = int(m.jnt_qposadr[m.joint(f"rk_{rk['root']}").id])
            jp = m.joint(self.joint_of[rk["pinion_root"]]).id
            rk["pdof"], rk["pq"] = int(m.jnt_dofadr[jp]), int(m.jnt_qposadr[jp])
        self.mech_fail = None
        self._shk_q = {n: int(m.jnt_qposadr[m.joint(f"shk_{n}").id]) for n in self._shocks}
        self._shk_rng = {}
        for mo in self.motors.values():
            if mo.get("pullback"):
                mo["mode"] = "pullback"

    def _v13_step(self, t, rows):
        """(v13) every step: servo position loops, pullback springs, rack-and-pinion skip / run-off"""
        m, d, p = self.m, self.d, self.p
        for n, mo in self.motors.items():
            if mo.get("servo") and "dof" in mo and not mo.get("dead"):
                q = float(d.qpos[m.jnt_qposadr[m.dof_jntid[mo["dof"]]]])
                w = float(d.qvel[mo["dof"]])
                w0 = p.servo_w0_rad_s
                err = mo.get("target", 0.0) - q
                st_ = mo.get("servo_state", "idle")
                if st_ == "wait" and d.time >= mo["go_t"]:
                    st_ = "drive"
                if st_ == "drive":
                    if abs(err) < math.radians(p.servo_tol_deg) and abs(w) < 0.5:
                        st_ = "idle"  # there: it stops driving
                    else:
                        mo["blocked_s"] = mo.get("blocked_s", 0.0) + p.dt if abs(w) < 0.5 else 0.0
                        if mo["blocked_s"] >= p.servo_block_s:
                            st_ = "idle"  # blocked: it gives up until the next command
                mo["servo_state"] = st_
                if st_ == "drive":
                    v_des = max(-w0, min(w0, p.servo_kp_s * err))
                    d.ctrl[mo["act"]] = max(-1.0, min(1.0, p.servo_kv * (v_des - w) / w0))
                else:
                    d.ctrl[mo["act"]] = 0.0  # held by its gearbox friction only
            pb = mo.get("pullback")
            if pb is not None and "dof" in mo:
                if t < 0:
                    continue
                pb["released"] = True
                w = pb["sense"] * float(d.qvel[mo["dof"]])
                pb["theta"] = min(max(pb["theta"] + w * p.dt, 0.0), pb["full"])  # turned back: it winds up again
                frac = max(0.0, 1.0 - pb["theta"] / pb["full"])
                w0 = p.pullback_no_load_rad_s
                tau = p.pullback_stall_nm * frac * max(0.0, min(1.0, 1.0 - w / w0))
                d.qfrc_applied[mo["dof"]] += pb["sense"] * tau
        for n, qa in self._shk_q.items():  # shock compression (from the pose as drawn) [m], for the result
            if t >= 0:
                q = float(d.qpos[qa]) + self._shocks[n]["q0"]
                lo, hi, _q = self._shk_rng.get(n, (q, q, q))
                self._shk_rng[n] = (min(lo, q), max(hi, q), q)
        for rk in self._rk:
            if "dof" not in rk:
                continue
            x = float(d.qpos[rk["qadr"]])
            th = float(d.qpos[rk["pq"]])
            ve = float(d.qvel[rk["dof"]]) - rk["c"] * float(d.qvel[rk["pdof"]])
            state = rk["state"]
            if state == "locked" and rk["eq"] in rows:
                lam = float(d.efc_force[rows[rk["eq"]]])
                rk["peak"] = max(rk["peak"], abs(lam))
                if abs(lam) > rk["cap"]:
                    state = "slipping"
            elif state == "slipping" and abs(ve) < p.rack_lock_v:
                m.eq_data[rk["eq"], 0] = x - rk["c"] * th  # the teeth lock again where they are
                state = "locked"
            if state == "slipping":
                F = rk["cap"] * p.mesh_slip_frac * math.tanh(ve / 0.01)
                d.qfrc_applied[rk["dof"]] -= F
                d.qfrc_applied[rk["pdof"]] += F * rk["c"]
            if state != rk["state"]:
                if state == "slipping":
                    rk["skips"] += 1
                d.eq_active[rk["eq"]] = 1 if state == "locked" else 0
                self.events.append((round(t, 3), f"rack {rk['name']}: {rk['state']} -> {state} (s = {x * 1e3:+.1f} mm)"))
                rk["state"] = state
            rk["s_min"] = min(rk.get("s_min", x), x)
            rk["s_max"] = max(rk.get("s_max", x), x)
            if t >= 0 and self.mech_fail is None:
                from ai_pd_tasks.lego_bench.racks import RAN_OUT_MM  # noqa: PLC0415
                tr = rk["tr"]
                if (tr.hi_kind == "mesh_end" and x * 1e3 >= tr.hi - RAN_OUT_MM) or (
                        tr.lo_kind == "mesh_end" and x * 1e3 <= tr.lo + RAN_OUT_MM):
                    self.mech_fail = f"rack {rk['name']} ran off its pinion at s = {x * 1e3:+.1f} mm"
                    self.events.append((round(t, 3), self.mech_fail))

    def _stiffen(self):
        """stud and bearing constraints with a physical stiffness K [N/m] (stud_k_n_m, bearing_k_n_m), independent of the
        parts' masses. A MuJoCo soft constraint with time constant tc, impedance d and effective inverse mass A deflects
        r = F (1 - d) A tc^2 under a static load F (damping ratio 1), so tc = 1 / sqrt((1 - d) A K) per constraint
        (A from the spawn pose; tc not below 2 dt, which caps the stiffness of the lightest parts)"""
        m, d, p = self.m, self.d, self.p
        tgt = {}
        if p.stud_k_n_m > 0:
            for st in self.studs:
                for e in st.eqs:
                    tgt[e] = (p.stud_k_n_m, p.stiff_imp)
        if p.bearing_k_n_m > 0:
            for e in self._bear_eqs:
                tgt[e] = (p.bearing_k_n_m, p.stiff_imp)
        if not tgt:
            return
        nef = d.nefc
        rows = {}
        for r in np.nonzero(d.efc_type[:nef] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)[0]:
            rows.setdefault(int(d.efc_id[r]), []).append(int(r))
        tcs = []
        for e, (K, dmin) in tgt.items():
            rs = rows.get(e)
            if not rs:
                continue
            A = float(np.mean(d.efc_diagA[rs[:3]]))
            tc = max(1.0 / math.sqrt(max((1.0 - dmin) * A * K, 1e-12)), p.stiff_tc_min_dt * p.dt)
            m.eq_solref[e] = (tc, 1.0)
            m.eq_solimp[e, :3] = (dmin, min(dmin + 0.005, 0.9999), 0.001)
            tcs.append(tc)
        if tcs:
            self.log.append(f"stiffen: {len(tcs)} constraints, time constants {min(tcs) * 1e3:.2f}-{max(tcs) * 1e3:.2f} ms")
        mujoco.mj_forward(m, d)

    def _cord_chain(self, specs):
        """the rope bench's cord as a taut string of rigid segments (cord_bodies): segment i hangs from node i on a
        vertical slide (node 0 is the start saddle) and pitches about a hinge there with stiffness T h (the string's
        energy 1/2 T sum h theta^2 for small slopes theta from the unloaded line), its far end is tied to the next
        segment's node through a short slider along the segment (the chain may lengthen slightly, as the elastic cord
        does), the last one to the goal saddle. Unloaded line: straight between the saddles with cord_sag0_mm.
        Contacts push the string down where they press, so a wedge between two tyres cannot be forced further in by
        a prescribed cord position (the quasi-static mocap cord did that). -> (worldbody xml, equalities, excludes)"""
        p = self.p
        P0, PN = np.array(specs[0][1], float), np.array(specs[-1][2], float)
        xs = [float(P0[0])] + [float(sp[2][0]) for sp in specs]
        nodes = []
        for x in xs:
            u = (x - P0[0]) / max(PN[0] - P0[0], 1e-6)
            nodes.append(P0 + (PN - P0) * u - np.array([0, 0, 4 * p.cord_sag0_mm * 1e-3 * u * (1 - u)]))
        Wc, Ec, Xc = [], [], []
        names = [f"cord_{sp[0]}" for sp in specs]
        for i, (name, _e0, _e1, r, col, fr) in enumerate(specs):
            a, b = nodes[i], nodes[i + 1]
            h = float(np.linalg.norm(b - a))
            qz = _z_to(b - a)
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, qz)
            R = R.reshape(3, 3)
            jz, jy = R.T @ np.array([0, 0, 1.0]), R.T @ np.array([0, 1.0, 0])
            k = p.cord_tension_n * h
            js = "" if i == 0 else (f'<joint name="cz{i}" type="slide" axis="{f3(jz)}" damping="{p.cord_damp_ns_m:.4g}" '
                                     f'armature="0.001"/>')
            js += (f'<joint name="cy{i}" type="hinge" axis="{f3(jy)}" stiffness="{k:.5g}" damping="{k * p.cord_tau_s:.4g}" '
                   f'armature="2e-6"/>')
            m_ = 1e-3
            fr_ = fr.replace('contype="3" conaffinity="3"', 'contype="1" conaffinity="1"')
            Wc.append(f'<body name="{names[i]}" pos="{f3(a)}" quat="{f3(qz)}" gravcomp="1">{js}'
                      f'<inertial pos="0 0 {h / 2:.5g}" mass="{m_}" diaginertia="{m_ * h * h / 12:.4g} {m_ * h * h / 12:.4g} 1e-9"/>'
                      f'<geom name="c_{name}" type="capsule" fromto="0 0 0 0 0 {h:.6g}" size="{r:.5g}" rgba="{col}" '
                      f'material="matte" {fr_}/>'
                      f'<body name="{names[i]}_end" pos="0 0 {h:.6g}"><joint type="slide" axis="0 0 1" damping="0.05" '
                      f'armature="1e-4"/><inertial pos="0 0 0" mass="1e-4" diaginertia="1e-10 1e-10 1e-10"/></body></body>')
            if i + 1 < len(specs):
                Ec.append(f'<connect body1="{names[i]}_end" body2="{names[i + 1]}" anchor="0 0 0" solref="0.002 1"/>')
            else:
                Ec.append(f'<connect body1="{names[i]}_end" anchor="0 0 0" solref="0.002 1"/>')
            for j in range(len(specs)):
                if j != i:
                    Xc.append((names[i], names[j]))
            Xc.append((names[i], "world"))  # the floor and the box-shaped course parts (saddles, platforms)
            for s2 in self.ob.shapes:
                if not s2.name.startswith("rope") and s2.kind != "box":
                    Xc.append((names[i], f"crs_{s2.name}"))
        self._cord_box_excl = names
        return Wc, Ec, Xc

    def _cables_plan(self):
        """PF leads: motor -> IR receiver output (or battery box), receiver -> battery box. Each lead is an elastic
        cable (MuJoCo cable plugin) of its real length, fixed where it leaves its part, its plug held on the pad by
        a point constraint; it hangs in a loop, drags on the course and pulls on both ends when taut."""
        from ai_pd_tasks.lego.catalog import MOTOR_LEAD_MM, RECEIVER_LEAD_MM  # noqa: PLC0415
        from ai_pd_tasks.lego_bench.design import ir_mapping  # noqa: PLC0415
        p, d = self.p, self.design
        inject, ends = {}, []
        if not p.cables:
            return inject, ends
        try:
            mp = ir_mapping(d)
        except Exception as e:  # noqa: BLE001
            self.log.append(f"cables: no IR mapping ({e!r})")
            return inject, ends
        plug = []
        self._ir = mp.get("outputs") or {}
        for mname, o in (mp.get("outputs") or {}).items():
            pads = self.cat[self.parts[o["receiver"]].part].pf_pads
            plug.append((mname, o["receiver"], pads[0 if o.get("output") == "A" else -1] if pads else None))
        for mname, box in (mp.get("box_direct") or {}).items():
            plug.append((mname, box, (self.cat[self.parts[box].part].pf_pads or (None,))[0]))
        for rx, info in (mp.get("receivers") or {}).items():
            if info.get("box"):
                plug.append((rx, info["box"], (self.cat[self.parts[info["box"]].part].pf_pads or (None,))[0]))
        for k, (dev, tgt, pad) in enumerate(plug):
            if dev not in self.world or tgt not in self.world:
                continue
            bd, bt = self.cat[self.parts[dev].part], self.cat[self.parts[tgt].part]
            L = (RECEIVER_LEAD_MM if bd.kind == "receiver" else MOTOR_LEAD_MM.get(bd.part_id, 200.0)) * 1e-3
            pd_, Rd = self.world[dev]
            pt_, Rt = self.world[tgt]
            a = pd_ + Rd @ (np.array(bd.pf_cable if bd.pf_cable is not None else (0, 0, 0)) * 1e-3)
            b = pt_ + Rt @ (np.array(pad if pad is not None else (0, 0, 0)) * 1e-3 + np.array([0, 0, 0.006]))
            # initial shape: a loop between the ends with the lead's length; rising (free leads) or, taped down as on
            # the real runs, lying flat over the vehicle towards its middle
            n = max(8, int(L / 0.016))
            dist = float(np.linalg.norm(b - a))
            if p.cable_taped:
                top = max(float(v[0][2]) for v in self.world.values())
                cen = np.mean([v[0] for v in self.world.values()], axis=0)
                ab = (b - a) / max(dist, 1e-6)
                side = np.cross(np.array([0.0, 0.0, 1.0]), ab)
                side = side / max(np.linalg.norm(side), 1e-6)
                if np.dot(cen - (a + b) / 2, side) < 0:
                    side = -side
                lift = max(0.0, top - max(a[2], b[2])) * 0.5 + 0.004
            def curve(H):
                ss = np.linspace(0, 1, n + 1)
                if p.cable_taped:
                    return np.array([a + (b - a) * x + side * H * math.sin(math.pi * x)
                                     + np.array([0, 0, 1.0]) * (lift * math.sin(math.pi * x) + 0.004 * x) for x in ss])
                return np.array([a + (b - a) * x + np.array([0, 0, 1.0]) * (H * math.sin(math.pi * x) + 0.01 * x)
                                 for x in ss])
            lo_, hi_ = 0.0, L
            for _ in range(40):
                H = (lo_ + hi_) / 2
                ln = float(np.linalg.norm(np.diff(curve(H), axis=0), axis=1).sum())
                lo_, hi_ = (H, hi_) if ln < L else (lo_, H)
            pts = curve((lo_ + hi_) / 2) if L > dist + 0.005 else curve(0.0)
            loc = (Rd.T @ (pts - pd_).T).T
            pfx = f"cab{k}"
            seg = float(np.linalg.norm(pts[-1] - pts[-2]))
            vert = " ".join(f3(v) for v in loc)
            dens = p.cable_mass_g * 1e-3 / (L * math.pi * p.cable_r_m ** 2)
            inject.setdefault(dev, []).append(
                f'<composite prefix="{pfx}" type="cable" vertex="{vert}" initial="none">'
                f'<plugin plugin="mujoco.elasticity.cable"><config key="twist" value="{p.cable_twist:.4g}"/>'
                f'<config key="bend" value="{p.cable_bend:.4g}"/><config key="vmax" value="0"/></plugin>'
                f'<joint kind="main" damping="{p.cable_damping:.4g}"'
                + (f' armature="{p.cable_armature:.4g}"' if p.cable_armature else '') + '/>'
                f'<geom type="capsule" size="{p.cable_r_m:.5g}" rgba="0.08 0.08 0.08 1" density="{dens:.5g}" '
                f'contype="{0 if (p.cable_taped or not p.cable_contact) else 2}" '
                f'conaffinity="{0 if (p.cable_taped or not p.cable_contact) else 2}" '
                f'friction="0.5 0.005 0.0001" group="1"/></composite>')
            tape = []
            if p.cable_taped:  # tape every third segment to the nearest part
                for k_ in range(2, n - 1, 3):
                    q_ = pts[k_]
                    ok_ = [nm for nm in self.world if self.kind(nm) in ("technic_brick", "plate", "battery", "receiver",
                                                                         "motor", "hub")]
                    near = min(ok_ or [dev], key=lambda nm: float(np.linalg.norm(self.world[nm][0] - q_)))
                    tape.append((k_, near))
            ends.append(dict(pfx=pfx, dev=dev, tgt=tgt, seg=seg, length=L, released=False, tape=tape))
        return inject, ends

    def _compile(self, X):
        A, W, E, _, C, ACT = self._parts_xml
        ex = "".join(f'<exclude body1="{a}" body2="{b}"/>' for a, b in dict.fromkeys(tuple(sorted(x)) for x in X)
                     if a != b)
        self.xml = (f'<mujoco model="crossbot"><compiler angle="radian" autolimits="true"/>'
                    f'<extension><plugin plugin="mujoco.elasticity.cable"/></extension>'
                    f'<option timestep="{self.p.dt}" integrator="implicitfast" cone="elliptic" impratio="10" '
                    f'jacobian="auto" iterations="80" ls_iterations="20"><flag multiccd="enable"/></option>'
                    + (HQ_VISUAL if HQ else
                       f'<visual><global offwidth="1920" offheight="1080"/><quality shadowsize="4096" offsamples="8"/>'
                       f'<headlight ambient="0.32 0.32 0.32" diffuse="0.25 0.25 0.25" specular="0.05 0.05 0.05"/>'
                       f'<map znear="0.005" zfar="30"/></visual>') +
                    f'<default><geom solref="0.003 1" solimp="0.95 0.99 0.0005" margin="0.0002"/></default>'
                    f'<asset><texture type="skybox" builtin="gradient" rgb1="0.93 0.91 0.86" rgb2="0.74 0.72 0.68" width="256" height="256"/>'
                    f'<texture name="wood" type="2d" builtin="gradient" rgb1="0.86 0.74 0.58" rgb2="0.80 0.67 0.50" width="512" height="512" mark="random" random="0.05" markrgb="0.74 0.6 0.44"/>'
                    f'<material name="table" texture="wood" texrepeat="6 6" specular="0.15" shininess="0.3" reflectance="0.02"/>'
                    f'<material name="board" rgba="0.96 0.96 0.94 1" specular="0.05" shininess="0.05"/>'
                    f'<material name="matte" specular="0.08" shininess="0.1"/>'
                    f'<material name="plastic" specular="0.55" shininess="0.75" reflectance="0.0"/>'
                    + "".join(A) + "</asset>"
                    + ('<worldbody>' + HQ_LIGHTS if HQ else
                       f'<worldbody><light pos="0.4 -0.8 1.6" dir="-0.25 0.5 -1" diffuse="0.62 0.6 0.56" specular="0.3 0.3 0.3" castshadow="true"/>'
                       f'<light pos="-0.8 0.9 1.2" dir="0.5 -0.6 -0.8" diffuse="0.25 0.27 0.3" specular="0.1 0.1 0.1" castshadow="false"/>')
                    + "".join(W) + "".join(getattr(self, "extra_world", [])) + "</worldbody>"
                    + f'<contact>{ex}{"".join(C)}</contact><equality>{"".join(E)}{getattr(self, "extra_eq", "")}</equality>'
                    + getattr(self, "extra_tendon", "")
                    + (f'<tendon>{"".join(self._tendon_xml)}</tendon>' if getattr(self, "_tendon_xml", None) else '') +
                    f'<actuator>{"".join(ACT)}</actuator></mujoco>')
        self.m = mujoco.MjModel.from_xml_string(self.xml, CrossSim._assets)
        self.d = mujoco.MjData(self.m)
        if self._cord:  # the cord: its segments' mocap ids, geom ids and the unloaded line between the saddles
            for i_, c_ in enumerate(self._cord):
                self._cord[i_] = c_[:3] + [int(self.m.body_mocapid[self.m.body(f"cord_{c_[0]}").id])]
            self._cord_geoms = {self.m.geom(f"c_{c_[0]}").id for c_ in self._cord}
            self._cord_x = (min(c_[1][0] for c_ in self._cord), max(c_[2][0] for c_ in self._cord))
            self._cord_load = [0.0, 0.5 * sum(self._cord_x)]
            if not getattr(self, "_cord_unsagged", False):
                # the course's cord carries the filmed LOADED sag as a fixed parabola; the dynamic cord makes that sag
                # itself under the vehicle, so its unloaded line is the taut cord: the parabola taken out (down to
                # cord_sag0_mm, the measured unloaded sag). Before, the two added up (~55 mm under the car, not 28).
                self._cord_unsagged = True
                x0_, x1_ = self._cord_x
                sag = float(self.ob.params.get("sag_mm") or 0.0) * 1e-3
                for c_ in self._cord:
                    for e_ in (c_[1], c_[2]):
                        u_ = min(max((e_[0] - x0_) / max(x1_ - x0_, 1e-6), 0.0), 1.0)
                        e_[2] += 4.0 * (sag - self.p.cord_sag0_mm * 1e-3) * u_ * (1.0 - u_)

    _assets: dict = {}

    _meshcols: dict = {}

    def _export_mesh(self, pid, mp):
        """one visual mesh per colour of the LDraw part (tyres black on grey rims, ...)"""
        if pid in CrossSim._meshcols:
            return CrossSim._meshcols[pid]
        g = json.load(open(mp))
        V = np.frombuffer(base64.b64decode(g["p"]), dtype=np.float32).reshape(-1, 3)
        F = np.frombuffer(base64.b64decode(g["i"]), dtype=np.uint32).reshape(-1, 3)
        C = np.frombuffer(base64.b64decode(g["v"]), dtype=np.uint8).reshape(-1, 3) if g.get("v") else None
        groups = {}
        if C is None:
            groups[(128, 128, 128)] = F
        else:
            fc = C[F[:, 0]]
            keys = [tuple(int(x) for x in k) for k in fc]
            for col in set(keys):
                groups[col] = F[np.array([k == col for k in keys])]
        out = []
        bp = self.cat.get(pid) if hasattr(self, "cat") else None
        tyre = bp is not None and bp.kind == "wheel" and bp.tyre_mu and isinstance(bp.shape, Cyl) and len(groups) > 1
        dark = min(groups, key=sum) if tyre else None
        for k, (col, Fg) in enumerate(sorted(groups.items(), key=lambda kv: -len(kv[1]))):
            if len(Fg) < 4:
                continue
            if tyre and col == dark:  # tyre: tread lugs dropped (they render as a dashed halo); a smooth tread band
                ax = {"x": 0, "y": 1, "z": 2}[bp.shape.axis]
                cen = np.array(bp.shape.centre)
                Q = V[Fg].mean(axis=1) - cen
                rad = np.linalg.norm(np.delete(Q, ax, axis=1), axis=1)
                Fg = Fg[rad < bp.shape.radius - 1.3]
                tube = _tube_obj(bp.shape.radius - 0.9, bp.shape.radius - 5.0, bp.shape.half_len * 0.94, ax, cen)
                tn = f"{pid}_tread"
                CrossSim._assets[f"{tn}.obj"] = ("\n".join(tube) + "\n").encode()
                out.append((tn, f3([x / 255 for x in col]) + " 1"))
            lines = _smooth_obj(V, Fg)
            name = f"{pid}_{k}"
            CrossSim._assets[f"{name}.obj"] = ("\n".join(lines) + "\n").encode()
            out.append((name, f3([x / 255 for x in col]) + " 1"))
        CrossSim._meshcols[pid] = out
        return out

    def _motor_stall(self, n):
        pid = self.parts[n].part
        if pid == "99499c01":  # PF L motor: its own bench value
            return self.p.l_stall_nm
        mp = motor_params(pid)
        return self.p.m_stall_nm * (mp["stall_torque_nm"] / 0.15)

    def _motor_w0(self, n):
        return motor_params(self.parts[n].part)["no_load_rad_s"] * (self.p.m_no_load_rad_s / 41.9)

    def _stud_points(self, a, b):
        cat = self.cat
        best = None
        for up, lo in ((b, a), (a, b)):
            bu, bl = cat[self.parts[up].part], cat[self.parts[lo].part]
            if not bu.studs_bottom or not bl.studs_top:
                continue
            pu, Ru = self.world[up]
            pl, Rl = self.world[lo]
            sa = np.array([pu + Ru @ (np.array([x, y, bu.bottom_z]) * 1e-3) for x, y in bu.studs_bottom])
            sb = np.array([pl + Rl @ (np.array([x, y, bl.top_z]) * 1e-3) for x, y in bl.studs_top])
            dd = np.linalg.norm(sa[:, None, :] - sb[None, :, :], axis=2)
            pts = sa[dd.min(axis=1) < 0.6e-3]
            if best is None or len(pts) > len(best[0]):
                best = (pts, up, lo)
        if best is None or not len(best[0]):
            pa, pb = self.world[a][0], self.world[b][0]
            self.log.append(f"stud {a}-{b}: no engaged stud found, one weld at the midpoint")
            return [(pa + pb) / 2], b, a
        return list(best[0]), best[1], best[2]

    def _bearing_points(self, c, axp, other):
        """two world points along the axle (or pin) where it sits in its hole or the motor output"""
        pax, Rax = self.world[axp]
        ax = Rax[:, 0]
        ph, Rh = self.world[other]
        bh = self.cat[self.parts[other].part]
        if c.kind == "motor":
            ax_h = Rh[:, 1]
            pts = [ph - ax_h * 1.0e-3, ph + ax_h * 5.0e-3]
            return [pax + ax * float(np.dot(q - pax, ax)) for q in pts]
        holes = [ph + Rh @ (np.array([x, 0.0, z]) * 1e-3) for x, z in bh.holes]
        holes += [ph + Rh @ (np.array(h[:3]) * 1e-3) for h in bh.pin_holes]
        holes += [ph + Rh @ (np.array(h[:3]) * 1e-3) for h in bh.axle_holes]
        cen = None
        if holes:
            dist = [np.linalg.norm((q - pax) - ax * float(np.dot(q - pax, ax))) for q in holes]
            k = int(np.argmin(dist))
            if dist[k] < 3e-3:
                cen = holes[k]
        if cen is None:
            cen = ph
        cen = pax + ax * float(np.dot(cen - pax, ax))
        return [cen - ax * 3.5e-3, cen + ax * 3.5e-3]

    # ================================================================ runtime
    def _apply_motor_program(self, t):
        for mo in self.motors.values():
            mo["w"] = float(self.d.qvel[mo["dof"]]) if "dof" in mo else 0.0
        prog = self.design.motor_program
        if not prog and self.p.no_program_all7:  # (v13) no program: every motor runs at +7 from the start (docs)
            prog = [{"t": 0.0, "motor": "all", "level": 7}]
        t_first = min((float(c.get("t", 0.0)) for c in prog), default=0.0)
        for k, cmd in enumerate(prog):
            t_cmd = float(cmd.get("t", 0.0))
            if self.p.program_late_s and t_cmd > t_first + 1e-9:  # (late nudge) every command after the first
                t_cmd += self.p.program_late_s
            if k in self._fired or t < t_cmd + self.p.ir_latency_s:
                continue
            self._fired.add(k)
            names = list(self.motors) if cmd.get("motor") == "all" else [cmd.get("motor")]
            for n in names:
                mo = self.motors.get(n)
                if mo is None or mo.get("dead"):
                    continue
                if mo.get("pullback") or (self.p.all_skips_pullback and cmd.get("motor") == "all"
                                          and self.parts[n].part == "12787c01"):
                    continue  # (v13) a pullback motor is not IR controlled
                lv = cmd.get("level")
                self.set_level(mo, lv)
                self.events.append((round(t, 3), f"{n}: {lv}"))
        if self.p.supply_r > 0:
            self._supply_sag()

    def _supply_sag(self):
        """(fit11) the motors share one battery box: its internal resistance (plus the IR receiver's output stage and
        leads), as a fraction supply_r of a motor's winding resistance, drops the supply voltage by the total current.
        Normalised: motor i draws c_i = |d_i| x - s_i b_i w_i / w0_i (its torque fraction), the supply is at x = 1 -
        supply_r * sum c_i of its open-circuit voltage -> x = (1 + r B) / (1 + r D); the back-EMF term is unchanged. On
        the flat (motors near no-load speed) x ~ 1; two motors held near stall lose a large part of their torque"""
        m, d, r = self.m, self.d, self.p.supply_r
        D = B = 0.0
        act = []
        for mo in self.motors.values():
            du = float(mo.get("duty", 0.0))
            if mo.get("mode") != "drive" or du == 0.0 or "dof" not in mo:
                continue
            k = float(m.actuator_biasprm[mo["act"], 2]) / mo["stall"]  # torque fraction per rad/s (negative)
            act.append((mo, du, k))
            D += abs(du)
            B -= math.copysign(1.0, du) * k * float(d.qvel[mo["dof"]])
        x = 1.0
        for _ in range(3):  # a motor driven backwards (generating) draws nothing: drop it and solve again
            x = min(1.0, max(0.05, (1.0 + r * B) / (1.0 + r * D)))
            neg = [(mo, du, k) for mo, du, k in act if abs(du) * x + math.copysign(1.0, du) * k * float(d.qvel[mo["dof"]]) < 0]
            if not neg:
                break
            for it in neg:
                act.remove(it)
                D -= abs(it[1])
                B += math.copysign(1.0, it[1]) * it[2] * float(d.qvel[it[0]["dof"]])
        self.supply_x = x
        for mo in self.motors.values():
            if mo.get("mode") == "drive":
                d.ctrl[mo["act"]] = float(mo.get("duty", 0.0)) * x

    def set_level(self, mo, lv):
        """a PF level on one motor. Drive: torque duty (L/7)**level_exp; the back-EMF term scales by duty**pwm_beta
        (the PF output floats between pulses: 1 = no-load speed independent of level, 0 = a plain voltage cut)"""
        if mo.get("servo"):  # (v13) a position servo: level k -> k/7 x 90 deg from centre; 0 / brake: centre
            k_ = 0.0 if lv in (0, None, "brake") else max(-7.0, min(7.0, float(lv)))
            mo["mode"], mo["target"] = "servo", mo["sign"] * k_ / 7.0 * math.pi / 2
            mo["go_t"] = self.d.time + max(0.0, self.p.servo_dead_s - self.p.ir_latency_s)  # its dead time
            mo["servo_state"], mo["blocked_s"] = "wait", 0.0
            return
        if lv == "brake":
            mo["mode"], duty, bs = "brake", 0.0, 1.0
        elif lv in (0, None):
            mo["mode"], duty, bs = "float", 0.0, 0.0
        else:
            u = (min(abs(float(lv)), 7.0) / 7.0) ** self.p.level_exp
            mo["mode"], duty, bs = "drive", mo["sign"] * math.copysign(u, float(lv)), u ** self.p.pwm_beta
        self.d.ctrl[mo["act"]] = duty
        mo["duty"] = duty
        self.m.actuator_biasprm[mo["act"], 2] = mo["bias"] * bs

    def _motor_stiction(self):
        """motor + gearbox: starting friction above running friction (a slow or stopped shaft sticks)"""
        m, d, p = self.m, self.d, self.p
        for mo in self.motors.values():
            dof = mo.get("dof")
            if dof is None:
                continue
            pb = mo.get("pullback")
            if pb is not None:  # (v13) wound and held until the release, then its own light friction
                m.dof_frictionloss[dof] = p.pullback_hold_nm if not pb["released"] else 0.05 * p.pullback_stall_nm
                continue
            w = abs(float(d.qvel[dof]))
            frac = p.motor_static if w < 0.4 else p.motor_friction
            m.dof_frictionloss[dof] = frac * mo["stall"]

    def _chain_slack(self):
        """(v14) each chain's built slack [mm] by its sprocket pair (the validator's geometry)"""
        if getattr(self, "_slack", None) is None:
            self._slack = {}
            try:
                from ai_pd_tasks.lego_bench import inventory_r4b as R4  # noqa: PLC0415
                for cg in R4.chain_geoms(self.design, place(self.design)):
                    if isinstance(cg, R4.ChainGeom):
                        self._slack[frozenset(cg.chain.gears)] = float(cg.slack_mm)
            except Exception as e:  # noqa: BLE001
                self.log.append(f"chain slack not computed: {e}")
        return self._slack

    def _free_axle_friction(self):
        """(v14) load-dependent bearing friction on the free wheel axles (Params.axle_load_fric_m), every 10 steps"""
        m, d, p = self.m, self.d, self.p
        if getattr(self, "_fa", None) is None:
            driven = {mo.get("dof") for mo in self.motors.values()}
            for g in self.meshes:  # gear pairs and chains
                driven |= {g.ja, g.jb}
            for e in range(m.neq):  # joint couplings (tread rollers, CV, differential ...)
                if m.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT:
                    for j in (m.eq_obj1id[e], m.eq_obj2id[e]):
                        if j >= 0:
                            driven.add(int(m.jnt_dofadr[j]))
            for w in range(m.nwrap):
                if m.wrap_type[w] == mujoco.mjtWrap.mjWRAP_JOINT:
                    driven.add(int(m.jnt_dofadr[m.wrap_objid[w]]))
            g2d = -np.ones(m.ngeom, dtype=int)
            base, axj = {}, {}
            for n, pt in self.parts.items():
                if self.cat[pt.part].kind != "wheel":
                    continue
                root = self.root_of.get(n)
                jn = self.joint_of.get(root)
                if not jn or not str(jn).startswith("j_"):
                    continue
                j = m.joint(jn).id
                if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                    continue
                dof = int(m.jnt_dofadr[j])
                if dof in driven:
                    continue
                base[dof] = float(m.dof_frictionloss[dof])
                axj[dof] = j
                bodies = {self.bid[q] for q in self.parts if self.root_of.get(q) == root and q in self.bid}
                for gi in np.nonzero(np.isin(m.geom_bodyid, list(bodies)))[0]:
                    g2d[gi] = dof
            self._fa, self._fa_g2d, self._fa_n, self._fa_ax, self._fa_lf = base, g2d, 0, axj, {}
        self._fa_n += 1
        if not self._fa or (self._fa_n - 1) % 10:
            return
        # the bearing load: the radial part of the vector sum of the contact forces on the axle's cluster (forces that
        # cancel, e.g. flanges pinching a rail or a pulley groove gripping the cord, load the holes only by their sum)
        F = {k: np.zeros(3) for k in self._fa}
        f6 = np.zeros(6)
        g2d = self._fa_g2d
        for i in range(d.ncon):
            c = d.contact[i]
            d1, d2 = g2d[c.geom1], g2d[c.geom2]
            if d1 == d2:  # neither on a free axle, or both on the same one (internal)
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            fw = c.frame.reshape(3, 3).T @ f6[:3]  # world frame: the force geom1 exerts on geom2
            if d2 >= 0:
                F[d2] += fw
            if d1 >= 0:
                F[d1] -= fw
        lf = self._fa_lf
        k = min(1.0, 10 * p.dt / p.axle_load_tau_s) if p.axle_load_tau_s > 0 else 1.0
        for dof, b0 in self._fa.items():
            ax = self._fa_ax[dof]
            ax = d.xaxis[ax]
            fr = F[dof] - float(np.dot(F[dof], ax)) * ax
            lf[dof] = lf.get(dof, 0.0) + (float(np.linalg.norm(fr)) - lf.get(dof, 0.0)) * k
            m.dof_frictionloss[dof] = b0 + p.axle_load_fric_m * lf[dof]

    def _mesh_update(self, t):
        self._motor_stiction()
        if self.p.axle_load_fric_m > 0:
            self._free_axle_friction()
        """per gear pair: engagement from the geometry, then one of three states
        locked   - the tooth ratio is a joint equality (implicit); its force is the tooth force
        slipping - the teeth ride over each other: a bounded tooth force opposes the sliding (clicking), until the two
                   sides turn together again
        free     - no contact (centre distance opened by the tooth height, or the gears slid apart)
        The translational part of the tooth force and its separating part (tan 20 deg) act on the two gears; the
        hinge-equality's spurious reaction on each hinge's parent is removed."""
        d, m, p = self.d, self.m, self.p
        tan_a = math.tan(math.radians(p.pressure_deg))
        h = p.addendum_mm * 1e-3
        nef = d.nefc
        rows = {}
        if nef:
            typ, ids = d.efc_type[:nef], d.efc_id[:nef]
            for r in np.nonzero(typ == mujoco.mjtConstraint.mjCNSTR_EQUALITY)[0]:
                rows.setdefault(int(ids[r]), int(r))
        rows_f = {}
        if nef and p.mesh_clutch:
            for r in np.nonzero(d.efc_type[:nef] == mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON)[0]:
                rows_f[int(d.efc_id[r])] = int(r)
        mdofs = {mo.get("dof") for mo in self.motors.values()}
        for dof, v in getattr(self, "_mx", {}).items():  # last step's mesh losses off (motor dofs are reset above)
            if dof not in mdofs:
                m.dof_frictionloss[dof] -= v
        self._mx = {}
        self._v13_step(t, rows)
        for g in self.meshes:
            ia, ib = self.bid[g.a], self.bid[g.b]
            na = np.asarray(self._joint_axis(g.ja), dtype=float)  # gear a's turning axis (its hinge), whatever its part frame
            na = na / max(float(np.linalg.norm(na)), 1e-12)
            ca, cb = d.xpos[ia], d.xpos[ib]
            dv = cb - ca
            ax_off = abs(float(np.dot(dv, na)))
            dperp = dv - na * float(np.dot(dv, na))
            c = float(np.linalg.norm(dperp))
            u = dperp / max(c, 1e-9)
            if not g.c0:
                # parallel spur / double-bevel gears: the ideal pitch distance (a mesh built off it sits loose or tight
                # from the start). Chains, right-angle bevels, worms and knob wheels: as built (their own geometry)
                nb_ = d.xmat[ib].reshape(3, 3)[:, 1]
                par = abs(float(np.dot(na, nb_))) > 0.9 and min(g.za, int(round(g.rb * 2e3))) >= 6 and g.spur
                g.c0 = g.ra + g.rb if (g.kind != "chain" and par) else c
                g.ax0 = 0.0 if par else float(np.dot(dv, na))  # crossed axes: the as-built offset is the mesh
                if getattr(g, "ax_built", False):  # (v13) a differential ring: its as-built plane is the mesh
                    g.ax0 = float(np.dot(dv, na))
                if g.kind == "chain" and p.chain_engage_mm:  # (v12) a chain's line: as built
                    g.ax0 = float(np.dot(dv, na))
            ax_off = abs(float(np.dot(dv, na)) - getattr(g, "ax0", 0.0))
            gap = c - g.c0
            if g.kind == "chain" and p.chain_engage_mm:  # (v12) only within about half a link of slack, in line
                cl_, fa_, axm_ = (float(x) * 1e-3 for x in p.chain_engage_mm)
                eng = 1.0 if (-cl_ <= gap <= fa_ and ax_off <= axm_) else 0.0
            elif g.kind == "chain":  # the chain carries while slack (sprockets up to 12 mm closer), skips if stretched
                eng = 1.0 if -25e-3 < gap < 2.5e-3 else 0.0
            else:
                eng = 1.0 if gap <= 0.1e-3 else max(0.0, 1.0 - (gap - 0.1e-3) / (2 * h - 0.1e-3))
            wmax = (g.wa + g.wb) / 2e3 if g.kind != "chain" else 0.05  # the chain's own links keep it on its sprockets
            if g.kind == "chain" and p.chain_engage_mm:
                pass
            elif ax_off > wmax:
                eng = 0.0
            elif ax_off > wmax - 2e-3:
                eng *= (wmax - ax_off) / 2e-3
            g.engaged = eng
            if g.kind == "chain" and t >= 0:  # (v12) logged: the sprocket gap's range while driving
                g.gap_min = min(getattr(g, "gap_min", gap), gap)
                g.gap_max = max(getattr(g, "gap_max", gap), gap)
                g.ax_max = max(getattr(g, "ax_max", 0.0), ax_off)
            ja_w, jb_w = self._joint_axis(g.ja), self._joint_axis(g.jb)
            qda, qdb = float(d.qvel[g.ja]), float(d.qvel[g.jb])
            ve = g.ra * g.sa * (qda - g.a1 * qdb)  # sliding speed of the teeth [m/s]
            if g.kind == "chain":  # (v12) chain_cap_n_per_tooth: the skip tension grows with the smaller sprocket
                zc = min(g.za, int(round(g.rb * 2e3)))
                cap = (p.chain_cap_n_per_tooth * zc if p.chain_cap_n_per_tooth > 0 else p.chain_cap_n) * eng
            else:  # small pinions (8 teeth) have short, undercut teeth and a low contact ratio: they skip sooner
                zmin = min(g.za, int(round(g.rb * 2e3)))
                cap = p.mesh_cap_n * eng * eng * (min(1.0, (zmin / 12.0) ** p.mesh_cap_z_exp) if g.spur else 1.0) * (1.0 if g.same else p.mesh_mixed_k)
                if not g.same and p.mesh_mixed_gap_mm > 0 and gap > 0:  # double bevel on a spur, pulled apart
                    cap *= math.exp(-gap * 1e3 / p.mesh_mixed_gap_mm)
            if p.mesh_clutch:
                self._mesh_clutch(g, t, eng, cap, ve, gap, ax_off, rows_f)
            else:
                self._mesh_eq(g, t, eng, cap, ve, gap, ax_off, rows)
            tau_a, tau_b = g.tau_a, g.tau_b
            g.f_t = abs(tau_a) / g.ra
            g.peak = max(g.peak, g.f_t)
            if t >= 0:
                g.peak_run = max(getattr(g, "peak_run", 0.0), g.f_t)
            if g.active and (p.mesh_drag_nm or p.mesh_mu):
                jl, rl = (g.jb, g.rb) if g.jb not in mdofs else (g.ja, g.ra)
                v_ = p.mesh_drag_nm + p.mesh_mu * g.f_t * rl
                m.dof_frictionloss[jl] += v_
                self._mx[jl] = self._mx.get(jl, 0.0) + v_
            if g.kind == "chain" and p.chain_jam_slack_mm > 0 and self._chain_slack().get(frozenset((g.a, g.b)), 0.0) \
                    > p.chain_jam_slack_mm:  # (v14 proposal) a loose chain jams: its sprockets held
                for jl in (g.ja, g.jb):
                    m.dof_frictionloss[jl] += p.chain_jam_nm
                    if jl not in mdofs:
                        self._mx[jl] = self._mx.get(jl, 0.0) + p.chain_jam_nm
                if not getattr(g, "jam_logged", False):
                    g.jam_logged = True
                    self.events.append((t, f"chain {g.a}-{g.b} jammed (built slack "
                                           f"{self._chain_slack()[frozenset((g.a, g.b))]:.1f} mm > {p.chain_jam_slack_mm} mm)"))
            if getattr(g, "worm", None):  # (v13) self-locking worm: friction on the worm shaft ~ its tooth torque
                jw, tw = (g.ja, g.tau_a) if g.worm == "a" else (g.jb, g.tau_b)
                g.worm_f = getattr(g, "worm_f", 0.0)
                g.worm_f += (p.worm_lock_k * abs(tw) * (1.0 if g.active else 0.0) - g.worm_f) * min(1.0, p.dt / 0.005)
                m.dof_frictionloss[jw] += g.worm_f
                self._mx[jw] = self._mx.get(jw, 0.0) + g.worm_f
            if g.f_t == 0.0 or os.environ.get("CSIM_NO_FT"):
                continue
            if g.kind == "chain":  # the tight strand pulls the sprockets together
                f_a = g.f_t * u
            elif getattr(g, "worm", None) == "a":  # (v13) a worm: its tooth force is tangential to the GEAR (along
                # the worm's axis: the worm takes it as thrust), not tangential to the worm
                nb_ = np.asarray(jb_w, float) * (1.0 if float(np.dot(jb_w, d.xmat[ib].reshape(3, 3)[:, 1])) > 0 else -1.0)
                ub_ = -(dv - nb_ * float(np.dot(dv, nb_)))
                ub_ = ub_ / max(float(np.linalg.norm(ub_)), 1e-9)
                # and its separating part pushes the gear radially (in its own plane) away from the worm
                f_a = -(tau_b * float(np.dot(jb_w, nb_)) / g.rb) * np.cross(nb_, ub_) + g.f_t * tan_a * ub_
            else:
                f_a = (tau_a * float(np.dot(ja_w, na)) / g.ra) * np.cross(na, u) - g.f_t * tan_a * u
            if g.dbl and p.mesh_axial_k:  # double bevels: axial thrust away from the centred mesh (unstable)
                s_ax = float(np.dot(dv, na))
                f_a = f_a - na * (p.mesh_axial_k * g.f_t * math.tanh(s_ax / 2e-4))
            mujoco.mj_applyFT(m, d, f_a, np.zeros(3), ca, ia, d.qfrc_applied)
            mujoco.mj_applyFT(m, d, -f_a, np.zeros(3), cb, ib, d.qfrc_applied)
            pa_, pb_ = m.body_parentid[m.dof_bodyid[g.ja]], m.body_parentid[m.dof_bodyid[g.jb]]
            mujoco.mj_applyFT(m, d, np.zeros(3), tau_a * ja_w, d.xpos[pa_], pa_, d.qfrc_applied)
            mujoco.mj_applyFT(m, d, np.zeros(3), tau_b * jb_w, d.xpos[pb_], pb_, d.qfrc_applied)

    def _mesh_clutch(self, g, t, eng, cap, ve, gap, ax_off, rows_f):
        """(mesh_clutch) the mesh as a slip clutch on the tendon qa - a1 qb: friction loss cap * ra while the teeth hold
        (they stick: the tendon does not move), mesh_slip_frac of it while they ratchet, 0 when out of mesh. The tooth
        torque is the friction force the solver applied in the last step."""
        m, d, p = self.m, self.d, self.p
        state = g.state
        if eng < p.engage_off:
            state = "free"
        elif state != "free" or eng >= 2 * p.engage_off:
            state = "slipping" if abs(ve) > p.mesh_slip_v else "locked"
        if state == "slipping" and g.state == "locked":
            g.f_skip = abs(float(g.tau_a)) / g.ra if g.ra else 0.0
            g.cap_skip = cap
        m.tendon_frictionloss[g.tid] = 0.0 if state == "free" else cap * g.ra * (
            p.mesh_slip_frac if state == "slipping" else 1.0)
        r = rows_f.get(g.tid)
        lam = float(d.efc_force[r]) if (r is not None and g.state != "free") else 0.0
        if state != g.state:
            self._mesh_event(g, t, state, f"mesh {g.a}-{g.b} {g.state} -> {state} (centre distance "
                             f"{gap * 1e3:+.2f} mm, axial offset {ax_off * 1e3:.1f} mm, sliding {ve:.2f} m/s)")
            g.state = state
        self._mesh_event_tick(g, t)
        g.active = state == "locked"
        g.tau_a, g.tau_b = lam, -lam * g.a1

    def _mesh_eq(self, g, t, eng, cap, ve, gap, ax_off, rows):
        """the mesh as before fit10: a joint equality while locked, an applied ratchet torque while slipping"""
        m, d, p = self.m, self.d, self.p
        if True:
            state = g.state
            if state == "locked" and eng < p.engage_off:
                state = "free"
            if state != "locked" and eng >= 2 * p.engage_off and abs(ve) < p.mesh_lock_v:
                m.eq_data[g.eq, 0] = float(d.qpos[m.jnt_qposadr[m.dof_jntid[g.ja]]]) - g.a1 * float(
                    d.qpos[m.jnt_qposadr[m.dof_jntid[g.jb]]])
                state = "locked"
            elif state == "free" and eng >= 2 * p.engage_off and p.mesh_reengage:
                # the teeth are back in each other's gaps while the two sides turn at different speeds: they ratchet
                # (clicking, a bounded tooth force) until the speeds match and the mesh locks again. Before, a mesh
                # that had sprung apart for a moment stayed free while its motor spun up: it could never re-engage.
                state = "slipping"
            tau_a = tau_b = 0.0
            if state == "locked" and g.eq in rows:
                r = rows[g.eq]
                nv = m.nv
                lam = float(d.efc_force[r])  # residual qa - c0 - a1 qb: J = (1, -a1)
                tau_a = lam
                tau_b = -lam * g.a1
                if abs(tau_a) / g.ra > cap:  # more than the engagement carries: the teeth start to slip
                    state = "slipping"
                    g.f_skip = abs(tau_a) / g.ra
                    g.cap_skip = cap
            if state == "slipping":
                if eng < p.engage_off:
                    state = "free"
                else:
                    F = cap * p.mesh_slip_frac * math.tanh(ve / 0.01)  # clicking: the mean of a sawtooth tooth force
                    tau_a = -F * g.ra * g.sa
                    tau_b = -F * g.ra * g.sa * (-g.a1)
                    d.qfrc_applied[g.ja] += tau_a
                    d.qfrc_applied[g.jb] += tau_b
            if state != g.state:
                self._mesh_event(g, t, state, f"mesh {g.a}-{g.b} {g.state} -> {state} (centre distance "
                                 f"{gap * 1e3:+.2f} mm, axial offset {ax_off * 1e3:.1f} mm, sliding {ve:.2f} m/s)")
                d.eq_active[g.eq] = 1 if state == "locked" else 0
                g.state = state
            self._mesh_event_tick(g, t)
            g.active = state == "locked"
            g.tau_a, g.tau_b = tau_a, tau_b

    MESH_EV_MIN_S = 0.02    # (v12, log only) a locked -> slipping skip is logged / counted once it lasted this long
    MESH_EV_MIN_N = 2.0     # ... and only if the tooth force at the skip was at least this [N] (or, 2026-10-28, 80 %
    #   of the mesh's own limit when that is lower: a weak mixed mesh at its limit counts its skips)
    MESH_EV_MAX = 10        # mesh events kept in the event log (the first ones); the rest are counted only

    def _mesh_log(self, t, text):
        self._mesh_ev_n = getattr(self, "_mesh_ev_n", 0) + 1
        if self._mesh_ev_n <= self.MESH_EV_MAX:
            self.events.append((round(t, 3), text))

    def _mesh_event(self, g, t, new, text):
        """(v12) a mesh state change, for the log and the skip count only (no physics): a skip from locked is pending
        until it lasted MESH_EV_MIN_S at a tooth force >= MESH_EV_MIN_N (start-up chatter at tiny forces is dropped);
        the re-lock after a dropped skip is dropped too; changes to / from free are always logged."""
        old = g.state
        pend = getattr(g, "pend", None)
        if old == "locked" and new == "slipping":
            g.pend = (t, getattr(g, "f_skip", 0.0), text)
            return
        if pend is not None:  # the skip ended before it was confirmed
            g.pend = None
            if new == "free" and pend[1] >= min(self.MESH_EV_MIN_N, 0.8 * getattr(g, "cap_skip", 1e9)):
                g.skips += 1
                self._mesh_log(pend[0], pend[2])
                g.logged = True
            elif new == "locked":
                return
        if new == "free" or old == "free" or getattr(g, "logged", False):
            self._mesh_log(t, text)
            g.logged = new != "locked"
        if old == "free" and new == "slipping":
            g.logged = True

    def _mesh_event_tick(self, g, t):
        pend = getattr(g, "pend", None)
        if pend is not None and g.state == "slipping" and t - pend[0] >= self.MESH_EV_MIN_S - 1e-9:
            g.pend = None
            if pend[1] >= min(self.MESH_EV_MIN_N, 0.8 * getattr(g, "cap_skip", 1e9)):
                g.skips += 1
                self._mesh_log(pend[0], pend[2])
                g.logged = True

    def _joint_axis(self, dof):
        j = self.m.dof_jntid[dof]
        return self.d.xaxis[j]

    def _cord_update(self):
        """the cord as a taut string (tension T) under the vehicle's contact loads. cord_nodes: each contact's vertical
        load is shared between the two nearest segment ends (nodes), smoothed per node over cord_tau, and the nodes
        deflect by superposed point-load V's, w(x) = sum_j F_j G(x, a_j), G = x (L - a) / (T L) before a_j and
        a (L - x) / (T L) after it: the cord is straight between the pulleys and kinks under each one, as a real
        string does. (Before fit8: one V at the load centroid, which sat between a vehicle's two pulleys and trapped
        them on its two flanks.) Each segment is moved onto its two nodes."""
        m, d, p = self.m, self.d, self.p
        f6 = np.zeros(6)
        x0, x1 = self._cord_x
        L = max(x1 - x0, 1e-3)
        if not hasattr(self, "_cord_nx"):
            self._cord_nx = np.array(sorted({round(float(e[0]), 6) for c_ in self._cord for e in (c_[1], c_[2])}))
            self._cord_nf = np.zeros(len(self._cord_nx))
            u_ = self._cord_nx - x0
            self._cord_G = np.minimum.outer(u_, u_) * (L - np.maximum.outer(u_, u_)) / L  # G[i, j] (x_i, a_j) * T
        nx = self._cord_nx
        Fn = np.zeros(len(nx))
        F, Fx = 0.0, 0.0
        for i in range(d.ncon):
            con = d.contact[i]
            if con.geom1 in self._cord_geoms or con.geom2 in self._cord_geoms:
                mujoco.mj_contactForce(m, d, i, f6)
                # the force's vertical component (contact frame -> world; the force on geom2 points along the normal
                # from geom1 to geom2): two tyre shoulders pinching the cord press on it with far more than the load
                # in normal force, but only the vertical resultant deflects a taut cord. Before: |normal| summed.
                fw = con.frame.reshape(3, 3).T @ f6[:3]
                fz = -float(fw[2]) if con.geom1 in self._cord_geoms else float(fw[2])  # z of the force ON the cord
                fn = -fz if p.cord_vertical_load else abs(float(f6[0]))  # signed: summed, then clipped
                F += fn
                Fx += fn * float(con.pos[0])
                xc = float(con.pos[0])
                k = int(np.clip(np.searchsorted(nx, xc) - 1, 0, len(nx) - 2))
                t_ = min(max((xc - nx[k]) / max(nx[k + 1] - nx[k], 1e-9), 0.0), 1.0)
                Fn[k] += fn * (1.0 - t_)
                Fn[k + 1] += fn * t_
        Fn = np.maximum(Fn, 0.0)
        F = max(F, 0.0)
        if not hasattr(self, "_veh_w"):
            self._veh_w = 9.81 * float(sum(m.body_mass[b] for b in self.bid.values()))
        cap = 1.5 * self._veh_w  # a contact impulse is not a static load on the cord
        if F > cap:
            Fn *= cap / F
            F = cap
        a_ = min(1.0, p.dt / p.cord_tau)
        ld = self._cord_load
        ld[0] += (F - ld[0]) * a_
        if F > 1e-3:
            ld[1] += (Fx / F - ld[1]) * a_
        if p.cord_nodes:
            self._cord_nf += (Fn - self._cord_nf) * a_
            wn = self._cord_G @ self._cord_nf / p.cord_tension_n

            def w(x):
                return float(np.interp(x, nx, wn))
        else:
            a = min(max(ld[1] - x0, 0.0), L)

            def w(x):
                u = min(max(x - x0, 0.0), L)
                return ld[0] * (u * (L - a) if u <= a else a * (L - u)) / (p.cord_tension_n * L)
        for name, e0, e1, mid in self._cord:
            p0 = e0 - np.array([0.0, 0.0, w(e0[0])])
            p1 = e1 - np.array([0.0, 0.0, w(e1[0])])
            d.mocap_pos[mid] = (p0 + p1) / 2
            d.mocap_quat[mid] = _z_to(p1 - p0)

    def _stud_check(self, t):
        """each stud: its net pull against the friction that holds it in its tube, and the moment rocking it in its
        tube (a row of studs held by point pairs) against stud_rock_nmm. A stud pulled straight out lets go at its
        strength; a stud levered open (the other studs of its joint in compression) holds stud_peel_k times more, as on
        the bench. Past either the stud slides out (its anchors move) and once out by the stud height it lets go.
        Strengths spread per stud."""
        d, m, p = self.d, self.m, self.p
        if t < 0.05 or not d.nefc:
            return
        if not hasattr(self, "_stud_mult"):
            rng = np.random.default_rng(p.seed)
            self._stud_mult = {id(s_): float(np.clip(rng.normal(1.0, p.stud_spread), 0.5, 1.6)) for s_ in self.studs}
        nef = d.nefc
        typ, ids, frc = d.efc_type[:nef], d.efc_id[:nef], d.efc_force[:nef]
        rows = {}
        for r in np.nonzero(typ == mujoco.mjtConstraint.mjCNSTR_EQUALITY)[0]:
            rows.setdefault(int(ids[r]), int(r))
        loads, groups = {}, {}
        for s in self.studs:
            if s.released_t is not None:
                continue
            z = d.xmat[self.bid[s.lower]].reshape(3, 3)[:, 2]
            pulls, shear = [], np.zeros(3)
            for e in s.eqs:
                if e not in rows:
                    continue
                F = frc[rows[e]:rows[e] + 3]
                fa = float(np.dot(F, z))
                pulls.append(-fa)
                shear += F - z * fa
            if not pulls:
                continue
            pull = float(sum(pulls))
            # a stud held by a point pair across its row: the pair's difference is a moment rocking the stud in its
            # tube (resisted by the tube up to stud_rock_nmm), not a pull
            rock = 0.0
            for i_ in range(0, len(pulls) - 1, 2) if s.rock else ():
                rock = max(rock, abs(pulls[i_] - pulls[i_ + 1]) / 2.0 * 2.0 * p.stud_r_m)
            loads[id(s)] = (s, pull, pull, float(np.linalg.norm(shear)), 0.0, rock)
            groups.setdefault(s.conn, []).append(pull)
        for key, (s, pull, eff, sh, phi_in, rock) in loads.items():
            g = groups.get(s.conn, [pull])
            # a joint levered open (its other studs in compression) holds stud_peel_k more per stud than a straight pull
            pos = [x for x in g if x > 0]
            phi = (1.0 - (sum(g) / len(g)) / max(max(pos), 1e-9)) if pos and len(g) > 1 else 0.0
            phi = min(1.0, max(0.0, phi))
            mult = self._stud_mult[key]
            engaged = max(0.05, 1.0 - p.stud_slip_weaken * s.slip / (p.stud_height_mm * 1e-3)) if p.stud_slip_weaken else 1.0
            strength = s.pull * mult * (1.0 + (p.stud_peel_k - 1.0) * phi) * engaged
            rcap = p.stud_rock_nmm * 1e-3 * mult * (s.pull / p.stud_pull_n) * engaged
            ratio_now = max(eff / max(strength, 1e-9), rock / max(rcap, 1e-12))
            # a load held for a couple of milliseconds pops a stud; a one-step solver spike does not
            a_ = min(1.0, p.dt / p.stud_load_tau)
            s.lp = getattr(s, "lp", 0.0) + (ratio_now - getattr(s, "lp", 0.0)) * a_
            s.shp = getattr(s, "shp", 0.0) + (sh - getattr(s, "shp", 0.0)) * a_  # shear judged smoothed too (a solver spike is not a load)
            ratio = s.lp
            s.last = (pull, eff, sh)
            s.peak = max(s.peak, ratio)
            ds = 0.0
            if ratio > 1.0:
                ds = min((ratio - 1.0) * strength / p.stud_slip_k, p.stud_slip_vmax * p.dt)
            elif eff < -p.stud_push_n and s.slip > 0:
                ds = max((eff + p.stud_push_n) / p.stud_slip_k, -5e-5, -s.slip)
            if ds:
                s.slip += ds
                for e, b0 in zip(s.eqs, s.base):
                    m.eq_data[e, 3:6] = b0 + s.slip * np.array([0.0, 0.0, 1.0])
            if s.slip > p.stud_height_mm * 1e-3 or s.shp > p.stud_shear_n * mult:
                for e in s.eqs:
                    d.eq_active[e] = 0
                s.released_t = t
                self._released_new = True
                self.events.append((round(t, 3), f"stud released: {s.upper} on {s.lower} (pull {pull:.2f} N against "
                                    f"{strength:.2f} N, rocking {rock * 1e3:.1f} N mm against {rcap * 1e3:.1f}, "
                                    f"shear {sh:.2f} N)"))

    def _plug_check(self, t):
        """a lead pulled taut can pop its plug off the pad: the motor (or every motor on that receiver) loses power"""
        d, m, p = self.d, self.m, self.p
        if not self.cables or not d.nefc:
            return
        nef = d.nefc
        rows = {}
        for r in np.nonzero(d.efc_type[:nef] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)[0]:
            rows.setdefault(int(d.efc_id[r]), int(r))
        for cb in self.cables:
            if cb["released"] or cb["eq"] not in rows:
                continue
            r = rows[cb["eq"]]
            F = float(np.linalg.norm(d.efc_force[r:r + 3]))
            cb["peak"] = max(cb.get("peak", 0.0), F)
            if F > p.plug_pull_n and t > 0:
                d.eq_active[cb["eq"]] = 0
                cb["released"] = True
                dead = [cb["dev"]] if cb["dev"] in self.motors else [
                    n for n, o in (self._ir or {}).items() if o.get("receiver") == cb["dev"]]
                for n in dead:
                    mo = self.motors.get(n)
                    if mo:
                        mo["mode"], mo["dead"] = "float", True
                        d.ctrl[mo["act"]] = 0.0
                        m.actuator_biasprm[mo["act"], 2] = 0.0
                self.events.append((round(t, 3), f"plug pulled off: {cb['dev']}'s lead from {cb['tgt']} ({F:.1f} N); "
                                    f"no power to {', '.join(dead) or 'nothing'}"))

    def parts_now(self):
        d = self.d
        out = {}
        for n in self.body_names:
            b = self.bid[n]
            q = d.xquat[b]
            out[n] = (tuple(float(v) for v in d.xpos[b]), (float(q[1]), float(q[2]), float(q[3]), float(q[0])))
        return out

    def run(self, t_max: float | None = None, video: str | None = None, fps: int = 30, size=(360, 640),
            cams=("follow", "side"), stop_on_result=True):
        m, d, p = self.m, self.d, self.p
        self._fired = set()
        self.events = []
        self._mesh_ev_n = 0
        self._released_new = False
        self.stud_break = None  # (v12) the first stud joint released while it counts: (upper, lower, t) -> judge
        judge = None
        t_end = (t_max if t_max is not None else self.ob.time_limit_s) + p.settle_s
        if video and HQ:
            size = tuple(int(v) for v in os.environ.get("CSIM_HQ_SIZE", "1080x1920").split("x"))
            fps = int(os.environ.get("CSIM_HQ_FPS", "30"))
            self.blur = int(os.environ.get("CSIM_HQ_BLUR", "4"))
        frames = {c: [] for c in cams} if video else None
        rend = mujoco.Renderer(m, size[0], size[1]) if video else None
        hq = _HQWriter(video, cams, fps, int(os.environ.get("CSIM_HQ_CRF", "16"))) if video and HQ else None
        next_frame = 0.0
        traj = []
        self._upz = []
        result = None
        nstep = 0
        k10 = max(1, int(round(0.01 / p.dt)))
        import time as _time
        wall0 = _time.time()
        self.diverged = False
        # the final ladder's raised benches (obstacles.FINAL_CHASM_KINDS): touching the chasm floor fails the run
        chasm_hit = None
        floor_gid = None
        if self.ob.params.get("chasm_floor_z") is not None:
            try:
                floor_gid = int(m.geom("floor").id)
            except KeyError:
                floor_gid = None
        veh_bodies = set(self.bid.values())
        while d.time < t_end:
            t = d.time - p.settle_s
            d.qfrc_applied[:] = 0
            if t >= 0:
                self._apply_motor_program(t)
            self._mesh_update(t)
            if self._cord:
                self._cord_update()
            mujoco.mj_step(m, d)
            nstep += 1
            self._stud_check(t)
            if self._released_new:  # (v12) a released stud joint breaks the vehicle: the judge fails the run "broke";
                # only from the motor program's start, and not after the crossing is judged (judge_at_goal_s after
                # reaching the goal: driving off the sim's finite goal deck is not a break of the run), as studs_released
                self._released_new = False
                if self.stud_break is None and t >= 0:
                    s_ = min((x for x in self.studs if x.released_t is not None), key=lambda x: x.released_t)
                    ct_ = judge.cross_t if judge is not None else None
                    if s_.released_t >= 0 and (ct_ is None or p.judge_at_goal_s is None
                                               or s_.released_t <= ct_ + p.judge_at_goal_s + 1e-9):
                        self.stud_break = (s_.upper, s_.lower, s_.released_t)
            if t >= 0 and chasm_hit is None and floor_gid is not None:  # raised benches: a part on the chasm floor fails
                for i_ in range(d.ncon):
                    c_ = d.contact[i_]
                    if c_.geom1 == floor_gid or c_.geom2 == floor_gid:
                        b_ = int(m.geom_bodyid[c_.geom2 if c_.geom1 == floor_gid else c_.geom1])
                        if b_ in veh_bodies:
                            chasm_hit = m.body(b_).name
                            break
            if nstep % k10 == 0 and t >= 0:
                pn = self.parts_now()
                if judge is None:
                    judge = OB.Judge(self.ob, self.design, OB.VehiclePose(self.pose.origin, self.pose.quat, pn,
                                                                           self.pose.rear_x, self.pose.front_x))
                judge.chasm_touch = chasm_hit
                judge.joint_released = None if self.stud_break is None else f"{self.stud_break[0]}/{self.stud_break[1]}"
                judge.mech_broke = getattr(self, "mech_fail", None)  # (v13) a rack ran off its pinion
                if result is None:
                    result = judge.update(t, pn, None)
                ch = pn[self.design.chassis][0]
                fx = max(v[0][0] for v in pn.values())
                self._upz.append(float(d.xmat[self.bid[self.design.chassis]][8]))  # chassis up-axis z (flip)
                traj.append([round(t, 3), *[round(v, 4) for v in ch], round(fx, 4),
                             *[round(float(g.engaged), 2) for g in self.meshes],
                             *[round(float(mo.get("w", 0)), 2) for mo in self.motors.values()]])
                if result is not None and stop_on_result:
                    break
            rec = getattr(self, "record", None)
            blur = max(1, int(getattr(self, "blur", 1))) if video else 1  # sub-frames averaged per frame (motion blur)
            if (video or rec is not None) and d.time >= next_frame:
                next_frame += 1.0 / (fps * blur)
                if rec is not None:  # poses for an external renderer: every part, and each lead as a polyline
                    rec.append({"t": round(d.time - p.settle_s, 4), "parts": self.parts_now(),
                                "cables": [[[round(float(v), 5) for v in d.xpos[b]] for b in cb.setdefault(
                                    "bodies", [b_ for b_ in range(m.nbody) if (mujoco.mj_id2name(
                                        m, mujoco.mjtObj.mjOBJ_BODY, b_) or "").startswith(cb["pfx"] + "B")])]
                                           for cb in self.cables]})
                if video:
                    for cname in cams:
                        rend.update_scene(d, camera=self._camera(cname))
                        self._draw_chains(rend.scene)
                        if hq is not None:
                            hq.add(cname, rend.render(), blur)
                        else:
                            frames[cname].append(rend.render().copy())
            if not np.isfinite(d.qpos).all() or d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > 0:
                result = {"passed": False, "failure": "simulation diverged"}
                self.diverged = True
                break
            if nstep % 2000 == 0 and _time.time() - wall0 > self.wall_limit_s:
                result = {"passed": False, "failure": "wall-clock limit"}
                self.diverged = True
                break
        if hq is not None:
            hq.close()
        elif video:
            if blur > 1:  # average each run of sub-frames: spinning wheels and fast parts blur instead of strobing
                frames = {c: [np.mean(np.stack(f[i:i + blur]).astype(np.float32), axis=0).astype(np.uint8)
                              for i in range(0, len(f) - blur + 1, blur)] for c, f in frames.items()}
            self._write_video(video, frames, fps)
        res = dict(result or {"passed": False, "failure": "time limit"})
        res["outcome"] = self.classify(traj)
        res["outcome"]["flipped"] = bool(d.xmat[self.bid[self.design.chassis]][8] < 0.0)
        k_ = res["outcome"].pop("_k_end", None)
        t_judge = None
        if k_ is not None and k_ < len(self._upz):  # crossed: upright or not when judged (see classify)
            res["outcome"]["flipped"] = bool(self._upz[k_] < 0.0)
            t_judge = traj[k_][0]
        res["diverged"] = self.diverged
        res.update({
            "events": self.events, "trajectory": traj,
            "mesh": {f"{g.a}-{g.b}": dict(slips=g.skips, peak_n=round(g.peak, 2), engaged=round(g.engaged, 2),
                                          state=g.state) for g in self.meshes},
            # (fit10) a crossed run counts the studs released up to when it is judged (judge_at_goal_s after the goal):
            # driving off the far end of the sim's finite goal deck is not a break of the filmed run
            "studs_released": sum(s.released_t is not None and (t_judge is None or s.released_t <= t_judge)
                                  for s in self.studs), "n_studs": len(self.studs),
            "studs_released_total": sum(s.released_t is not None for s in self.studs),
            "cables": [dict(dev=c["dev"], to=c["tgt"], peak_n=round(c.get("peak", 0.0), 2), unplugged=c["released"])
                       for c in self.cables],
            "stud_peak_n": round(max((s.peak for s in self.studs), default=0.0), 2),
            "stud_max_slip_mm": round(max((s.slip for s in self.studs), default=0.0) * 1e3, 2),
            "max_progress": round(float(judge.best - judge.start_ref) / judge.goal_dist, 3) if judge else 0.0,
            "log": self.log, "t_end": round(d.time - p.settle_s, 3)})
        res["mesh_events_dropped"] = max(0, self._mesh_ev_n - self.MESH_EV_MAX)
        mech = {}  # (v13) the mechanisms' state over the run
        for rk in self._rk:
            mech[f"rack {rk['name']}"] = dict(s_mm=[round(rk.get("s_min", 0.0) * 1e3, 1), round(rk.get("s_max", 0.0) * 1e3, 1)],
                                             travel_mm=[rk["tr"].lo, rk["tr"].hi], peak_n=round(rk.get("peak", 0.0), 2),
                                             skips=rk.get("skips", 0), state=rk.get("state"))
        for n, v in self._shk_rng.items():
            mech[f"shock {n}"] = dict(compression_mm=[round(v[0] * 1e3, 2), round(v[1] * 1e3, 2)], end_mm=round(v[2] * 1e3, 2))
        for td in self._treads:
            if td.get("ok"):
                mech[f"tread {td['g'].tread.name}"] = dict(rollers=len(td["rollers"]), r_tip_mm=round(td["g"].r_tip, 2),
                                                          r_pitch_mm=round(td["g"].r_pitch, 2), mu=td.get("mu_eff"))
        for dd in self._diffs:
            mech[f"differential {dd['name']}"] = "modelled" if dd.get("ok") else "not modelled"
        res["mechanisms"] = mech
        # (v12) per chain: the sprocket gap's range (centre distance - as built) and peak tension while driving
        res["chains"] = [dict(name=next((c.name for c in (getattr(self.design, "chains", None) or [])
                                         if tuple(c.gears) == (g.a, g.b)), f"{g.a}-{g.b}"),
                              sprockets=[g.a, g.b], peak_n=round(getattr(g, "peak_run", 0.0), 2),
                              gap_min_mm=round(getattr(g, "gap_min", 0.0) * 1e3, 2),
                              gap_max_mm=round(getattr(g, "gap_max", 0.0) * 1e3, 2),
                              axial_max_mm=round(getattr(g, "ax_max", 0.0) * 1e3, 2), skips=g.skips, state=g.state)
                         for g in self.meshes if g.kind == "chain"]
        return res

    def _belt_phase(self, key, body, ax, r):
        """(video only) how far a chain / tread loop has travelled [m]: sprocket `body`'s spin about `ax` relative to
        the vehicle's base body, times the pitch radius `r`, integrated over sim time (called per camera: once per time step)"""
        d, m = self.d, self.m
        st = self.__dict__.setdefault("_bphase", {})
        s0, t0 = st.get(key, (0.0, None))
        b = self.bid[body]
        w = d.cvel[b][:3] - d.cvel[int(m.body_rootid[b])][:3]  # relative to the vehicle's base body
        if t0 is not None and d.time > t0:
            s0 += float(np.dot(w, ax)) * r * (d.time - t0)
        if t0 is None or d.time != t0:
            st[key] = (s0, d.time)
        return s0

    @staticmethod
    def _draw_links(scn, loop, s0, pitch, length, rad, rgba, half_w=None):
        """capsules (or, with half_w, cross-bars of that half-width vector) every `pitch` along the closed polyline
        `loop`, shifted by s0 along it; False when the scene is full"""
        seg = [float(np.linalg.norm(b - a)) for a, b in zip(loop[:-1], loop[1:])]
        L = sum(seg)
        if L < 1e-6:
            return True
        n = max(1, int(round(L / pitch)))
        step = L / n
        cum = np.concatenate([[0.0], np.cumsum(seg)])

        def at(s):
            s %= L
            i = min(int(np.searchsorted(cum, s, side="right")) - 1, len(seg) - 1)
            f = (s - cum[i]) / max(seg[i], 1e-12)
            return loop[i] + f * (loop[i + 1] - loop[i])
        for k in range(n):
            if scn.ngeom >= scn.maxgeom:
                return False
            s = s0 % step + k * step
            if half_w is None:
                p0, p1 = at(s - length / 2), at(s + length / 2)
            else:
                c = at(s)
                p0, p1 = c - half_w, c + half_w
            gm = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(gm, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), rgba)
            mujoco.mjv_connector(gm, mujoco.mjtGeom.mjGEOM_CAPSULE, rad, p0, p1)
            scn.ngeom += 1
        return True

    def _draw_chains(self, scn):
        """(v12, video only) each chain drawn as dark capsules along its loop (both straight runs between the pitch
        circles' tangent points and both wraps), placed from the sprockets' current poses. Scene geoms only: nothing
        is added to the model, so the physics is untouched."""
        from ai_pd_tasks.lego_bench.inventory_r4b import chain_radius
        d = self.d
        rgba = np.array([0.13, 0.13, 0.14, 1.0], dtype=np.float32)
        for g in self.meshes:
            if g.kind != "chain":
                continue
            ca, cb = d.xpos[self.bid[g.a]].copy(), d.xpos[self.bid[g.b]].copy()
            ax = d.xmat[self.bid[g.a]].reshape(3, 3)[:, 1].copy()
            ax /= max(float(np.linalg.norm(ax)), 1e-12)
            dv = cb - ca
            cb = cb - ax * float(np.dot(dv, ax)) * 0.5  # both loops drawn in the mid plane between the sprockets
            ca = ca + ax * float(np.dot(dv, ax)) * 0.5
            dp = cb - ca
            D = float(np.linalg.norm(dp))
            if D < 1e-6:
                continue
            e = dp / D
            n = np.cross(ax, e)
            ra = chain_radius(g.za) * 1e-3
            rb = chain_radius(int(round(g.rb * 2e3))) * 1e-3
            beta = math.asin(max(-1.0, min(1.0, (ra - rb) / D)))
            ph = math.pi / 2 - beta
            pts_a = [ca + ra * (math.cos(a_) * e + math.sin(a_) * n) for a_ in np.linspace(ph, 2 * math.pi - ph, 24)]
            pts_b = [cb + rb * (math.cos(a_) * e + math.sin(a_) * n) for a_ in np.linspace(-ph, ph, 24)]
            loop = pts_a + pts_b + [pts_a[0]]  # a's wrap, the lower run to b, b's wrap, the upper run back
            # (2026-10-28) separate links that travel round the loop with sprocket a's turning (a solid loop looked stuck)
            s_ = self._belt_phase(("chain", g.a, g.b), g.a, ax, ra)
            if not self._draw_links(scn, loop, s_, 6.4e-3, 4.2e-3, 1.6e-3, rgba):
                return
        for td in getattr(self, "_treads", []):  # (v13) each tread loop: dark capsules round both sprockets
            if not td.get("ok"):
                continue
            g_ = td["g"]
            sa_, sb_ = g_.tread.sprockets
            ca, cb = d.xpos[self.bid[sa_]].copy(), d.xpos[self.bid[sb_]].copy()
            ax = d.xmat[self.bid[sa_]].reshape(3, 3)[:, 1].copy()
            ax /= max(float(np.linalg.norm(ax)), 1e-12)
            dp = cb - ca
            dp -= ax * float(np.dot(dp, ax))
            D = float(np.linalg.norm(dp))
            if D < 1e-6:
                continue
            e = dp / D
            n = np.cross(ax, e)
            r_ = 0.5 * (g_.r_tip + g_.r_in) * 1e-3
            th = 0.5 * (g_.r_tip - g_.r_in) * 1e-3
            loop = [ca + r_ * (math.cos(a_) * e + math.sin(a_) * n) for a_ in np.linspace(0.5 * math.pi, 1.5 * math.pi, 16)]
            loop += [ca + dp + r_ * (math.cos(a_) * e + math.sin(a_) * n) for a_ in np.linspace(-0.5 * math.pi, 0.5 * math.pi, 16)]
            loop.append(loop[0])
            s_ = self._belt_phase(("tread", sa_, sb_), sa_, ax, r_)  # (2026-10-28) cross-bars that travel with the tread
            if not self._draw_links(scn, loop, s_, 6.4e-3, 1.2e-3, th * 1.05, rgba, ax * (g_.half_w * 1e-3 - th)):
                return
            for off in np.linspace(-1.0, 1.0, max(2, int(g_.half_w / 4.0))):
                for p0, p1 in zip(loop[:-1], loop[1:]):
                    if scn.ngeom >= scn.maxgeom:
                        return
                    gm = scn.geoms[scn.ngeom]
                    mujoco.mjv_initGeom(gm, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), rgba)
                    w_ = ax * off * (g_.half_w * 1e-3 - th)
                    mujoco.mjv_connector(gm, mujoco.mjtGeom.mjGEOM_CAPSULE, th, p0 + w_, p1 + w_)
                    scn.ngeom += 1

    def course_span(self):
        """where the obstacle starts (its foot / the first rod / the plank; never the approach platform or a tilted
        panel's centre-minus-half-length, which lies before the foot) and its length to the goal"""
        xs = [s.pos[0] - s.size[0] / 2 for s in self.ob.shapes if s.kind == "box" and "stop" not in s.name
              and not s.name.startswith(("platform", "device"))]
        xs += [s.pos[0] - s.size[0] for s in self.ob.shapes if s.kind != "box" and not s.name.startswith(("platform", "device"))]
        x0 = max(0.0, min(xs)) if xs else 0.0
        return x0, max(self.ob.goal.x_min - x0, 0.05)

    def classify(self, traj):
        """stage (none / foot / climb / crossed), climb fraction, end state (stay / rollback / held / fell)"""
        if not traj:
            return {"stage": "none", "f": 0.0, "end": "stay"}
        x0, L = self.course_span()
        cx = np.array([r[1] for r in traj])
        fx = np.array([r[4] for r in traj])
        disp = float(cx.max() - cx[0])
        f = max(0.0, float(cx.max() - x0) / L)
        cz = np.array([r[3] for r in traj])
        zg = self.ob.goal.z_min if self.ob.goal.z_min is not None else -1e9
        # crossed: the chassis reached the goal side AT the goal's height (not after falling to the chasm floor and
        # driving on under the goal deck: rope flat, 2026-10-23)
        if self.ob.goal.x_min and np.any((cx >= self.ob.goal.x_min) & (cz >= zg - 0.02)):
            stage = "crossed"
        elif disp < 0.02:
            stage = "none"
        elif fx.max() - x0 < 0.02 or f < 0.05:
            stage = "foot"
        else:
            stage = "climb"
        k_end = None
        if stage == "crossed" and self.p.judge_at_goal_s is not None:
            # a crossed vehicle is judged judge_at_goal_s after it reached the goal: the filmed cars drove on out of the
            # frame (or were caught); the sim's goal deck is finite and a car that drives off its far end is not a
            # rollback or a flip of the crossing (cal2 opus bridge, cal1 A1)
            k0 = int(np.argmax((cx >= self.ob.goal.x_min) & (cz >= zg - 0.02)))
            k_end = min(len(cx) - 1, k0 + int(round(self.p.judge_at_goal_s / 0.01)))
            cx, cz = cx[:k_end + 1], cz[:k_end + 1]
        back = float(cx.max() - cx[-1])
        end = "rollback" if back > 0.05 else ("held" if stage == "climb" else "stay")
        drop = 0.12 if self.ob.kind == "rope" else 0.05  # (a car hanging from the cord sits lower than the start deck)
        if stage != "crossed" and cz[0] - cz[-1] > drop:  # ended well below where it started: fell off the course
            end = "fell"
        out = {"stage": stage, "f": round(min(f, 1.0), 3), "end": end, "disp_m": round(disp, 3),
               "front_gain_m": round(float(fx.max() - fx[0]), 3), "back_m": round(back, 3)}
        if k_end is not None:
            out["_k_end"] = k_end
        return out

    def _camera(self, name):
        cam = mujoco.MjvCamera()
        ch = self.d.xpos[self.bid[self.design.chassis]]
        if name == "follow":
            cam.lookat[:] = ch
            cam.distance, cam.azimuth, cam.elevation = 0.5, -60, -18
        else:  # roughly the real camera: in front of the course, a little left of the start, looking across it
            gx = self.ob.goal.x_min
            if self.ob.kind in ("bumps", "steps", "wall"):  # filmed square-on from the side of the table
                cam.lookat[:] = (gx * 0.3, 0.0, 0.05)
                cam.distance, cam.azimuth, cam.elevation = 0.8, 90, -12
            else:
                cam.lookat[:] = (gx * 0.45, 0.0, 0.08)
                cam.distance, cam.azimuth, cam.elevation = 0.95, 62, -20
        return cam

    def _write_video(self, path, frames, fps):
        import subprocess
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        for cname, fr in frames.items():
            if not fr:
                continue
            out = path.replace(".mp4", f"_{cname}.mp4")
            h, w = fr[0].shape[:2]
            pr = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s",
                                   f"{w}x{h}", "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p", "-c:v", "libx264", out],
                                  stdin=subprocess.PIPE)
            for f in fr:
                pr.stdin.write(f.tobytes())
            pr.stdin.close()
            pr.wait()


class _HQWriter:
    """CSIM_HQ: average each run of `blur` sub-frames (motion blur) and stream it to ffmpeg (x264, slow preset)"""

    def __init__(self, path, cams, fps, crf):
        import subprocess
        self.sp, self.path, self.fps, self.crf = subprocess, path, fps, crf
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.pr, self.acc, self.n = {}, {}, {}

    def add(self, cam, img, blur):
        if cam not in self.pr:
            h, w = img.shape[:2]
            self.pr[cam] = self.sp.Popen(
                ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r",
                 str(self.fps), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", str(self.crf), "-pix_fmt",
                 "yuv420p", "-movflags", "+faststart", self.path.replace(".mp4", f"_{cam}.mp4")], stdin=self.sp.PIPE)
            self.acc[cam], self.n[cam] = np.zeros(img.shape, np.float32), 0
        self.acc[cam] += img
        self.n[cam] += 1
        if self.n[cam] >= blur:
            self.pr[cam].stdin.write((self.acc[cam] / self.n[cam] + 0.5).astype(np.uint8).tobytes())
            self.acc[cam][:] = 0
            self.n[cam] = 0

    def close(self):
        for pr in self.pr.values():
            pr.stdin.close()
            pr.wait()


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("design")
    ap.add_argument("bench")
    ap.add_argument("level", type=int)
    ap.add_argument("--params", default="{}")
    ap.add_argument("--video", default=None)
    ap.add_argument("--t", type=float, default=None)
    a = ap.parse_args()
    sim = CrossSim(a.design, a.bench, a.level, json.loads(a.params))
    r = sim.run(a.t, a.video)
    tr = r.pop("trajectory")
    r.pop("params", None)
    print(json.dumps({k: v for k, v in r.items() if k not in ("params",)}, indent=1, default=str)[:4000])
    print("traj tail", tr[-3:])
