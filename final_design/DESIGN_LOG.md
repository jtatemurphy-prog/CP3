# Design log (agent)

Generators live in `agent_gen/` (run from that folder: `python gC.py out.json '{"chassis":"ubL"}'`).

| id | generator | what | valid | steps | rope | ramp | bridge | notes |
|---|---|---|---|---|---|---|---|---|
| A1_invalid | gA.py | first straddle proto, 4 direct-drive motors | no (needs 2 boxes, L/M speed mismatch) | | | | | dead end: driven wheels must turn at exactly equal speed |
| B1_invalid | gB.py | gear-train straddle with studded columns | no (stud pull-off criticals) | | | | | review rejects wheel assemblies hanging from studs |
| C1 | gC.py | pinned liftarm straddle, wb 128, 4x 32020c01, 2 L motors on idler A, 40T trains, 2 driven rope wheels (55982c01) at z 80, battery on top | yes | stuck at 1st riser | stuck (rolled 90 deg on cord) | PASS 24.4 s (slow) | PASS 1.47 s | CoG 74 mm high |
| C1g150 / C1g250 | gC.py + start_gap | C1 with longer run-up for steps | yes | stuck | | | | momentum does not help |
| D1 | gD.py | low battery, motors one side | yes | stuck | stuck (rolled) | wall-clock limit | FAIL drift -> chasm | one-sided drive yaws |
| F1 | gF.py | 40T flanges beside rope wheel, narrow deck | yes | - | stuck 0.18 m (upright; front rope wheel behind front tyre) | wall-clock limit | PASS 1.50 s | flanges keep the car upright on the cord |
| G1 | gG.py | whegs (liftarm spokes, tip r 35.6), M motors | yes | stuck 0.14 (wheels STALL against riser: torque-limited) | stuck 0.29 | stuck 0.32 | PASS 1.97 s | whegs need more torque; ABS spokes poor on ramp |
| H1 | gH.py | rope wheels 24 mm AHEAD of each ground axle, front/rear independent L drives, wb 104 | yes | stuck at riser 1 | 0.80 progress: crosses cord, stuck at goal deck face | stuck at ramp foot (cause TBD, H1_ramp_rec queued) | FAIL drift off plank (motr) | rope fixed except exit; rope-wheel bottom 80.1 mm = zero margin over the 80 mm cord top |
| K1 | gK.py | H1 + driven helper wheels 56145c01 at (xF+64, R+8), 17.7 mm above ground, geared 1.67x via 40T-40T-8T-24T-24T | yes | stuck 0.23 (helper gets the nose onto step 1, best so far; then stuck) | running at stop | running at stop | FAIL drift off plank (motr) at 0.71 | helper should wedge the nose up each riser; also should lift the front onto the rope goal deck |

## Key learnings (2026-11-18)
- Steps, 2-axle car: the climbing axle needs a push of ~1.0-1.4x its load from the other axle at mu 0.82 -> front climb needs rear-heavy, rear climb needs front-heavy: impossible statically. Momentum (1.5 m/s) does not help.
- Fix being tested (K1): a driven helper wheel ahead of the front axle with its centre ABOVE the riser corner (contact angle ~29 deg). Needs CoG at <~0.5 of the helper-to-rear distance and >~55% on the front axle (K1: CoG x 24.7, 70% front).
- Gear chains on the 8 mm grid: usable diagonals are (8,16) 17.9 mm (12T+24T, 16T+20T, knob wheels), (24,32) 40 mm (40T+40T), (16,32) 35.8 (36T+36T); spur tolerance -0.15..+0.6 mm. Count meshes: an odd number reverses the wheel.
- Rope: rope-wheel bottom must sit several mm BELOW 80 mm above the ground-wheel bottoms, otherwise the ground wheels meet the goal deck face (H1). Plan: ZX = R+56 via an 8T rope gear on idler A.
- Bridge: wheels at y +-14.2 only overlap the 40 mm plank by 15.8 mm; lateral drift tolerance is small.
- agent_runner.py now accepts a per-job "wall" (s) to raise CSIM_WALL_S for exploration (needs a runner restart).
- H1 ramp (record): front wheels slip on the 30 deg panel as load shifts rearward, rear wheel stalls against the panel's 6.9 mm foot lip. Direct-drive L motors lack torque/traction there; C1 (geared down) passed. Next: gear the ground drive down and move CoG forward.
- Bridge failures (H1, K1) both "motr touched chasm floor" around 0.7 m: the car drifts sideways off the plank. Next: check yaw in a record run; consider wheels further inboard and a symmetric motor layout.

## Stopped 2026-11-18 ~10:05 at the user's request after K1. Suggested next steps
1. K1 steps: record run to see where it stalls after the helper climbs step 1 (likely the front wheels against riser 1 or the helper against riser 2).
2. Lower the rope wheels (8T rope gear, ZX = R+56) to fix the rope exit.
3. Gear the drive down (ramp) and fix the bridge drift.

## Round 2+ (2026-11-18 afternoon): population runs
Generators: `gM.py` (H-frame, options), `gN.py` (gear/frame planes shifted 8 mm out so a 3711 chain fits at y 36;
options: drive ff/fr, chain, helper, rb_drive, rope_mode classic/pair, rope_wheel, motor L/XL), `gK.add_helper`.
`popdb.py <agent_results>` prints the population table.

| id | idea | steps | rope | ramp | bridge |
|---|---|---|---|---|---|
| K2 | K1 + rope lowered (8T rope gear, ZX R+56) | stuck 0.23 | broke (front axle out of motor) 0.75 at goal deck | PASS 20.8 s | drift off plank 0.60 |
| K3 | K2 with XL motors | stuck 0.49 (front up 2 steps, rear can't follow) | stuck 0.75 at goal deck | PASS 7.7 s | drift 0.56 |
| N0b | both L motors on front axles + chains to rear, no helper | stuck 0.15 | stuck 0.21 (chain disengaged) | stuck 0.14 | |
| N1 / N1b / N1X | N0 + helper | (N1/N1b cancelled: gear loop) | | | |
| N2 / N2X | N1 without the rear rope drive (breaks the closed gear loop) | queued | | | |
| P2 / P2X / P1X | rope A moved to the helper column (xF+64, driven off the helper chain gear), rope B free at x 56, lateral CoG centred; P1X uses 56145 tyre pairs as rope wheels (lower, 14 mm exit margin) | queued | | | |

Learnings:
- Every gear mesh costs ~39% of the torque in this simulator (mesh_mu 0.387 x tooth force x radius); motors
  should sit directly on the wheels that need torque. XL motors (0.4 N m) fixed the ramp (7.7 s) and got further up the steps.
- Closed gear loops (front L-R through one rope axle, rear L-R through another, front-rear through chains) bind and
  stall the motors (N0b). Keep the drive a tree.
- K1 steps record: the helper lifts the nose onto step 1, then both motors STALL with the front tyre against riser 1:
  torque-limited, not traction-limited (L motor through 4 meshes).
- Bridge drift is a slow yaw (~2 deg over 0.4 m) toward -y with the diagonal motor layout.
- Rope exit: the goal-deck face meets the lowest forward parts (helper, XL motor) while the car still hangs low on
  the sagging cord: the front rope wheel must be the front-most part and sit as low on the car as roll stability allows.

## 2026-11-18 evening / 11-19 morning: population + targeted series
Tools: `sample.py` (random valid designs over gN's settings, ~93% valid after constraint rules), `campaign.py`
(diverse picks + job submission), `surrogate.py` (random-forest progress model per course, ranks untested samples),
`POPULATION.md` (table of every plain run).

Results:
- Random batch S3 (16 designs, steps + bridge): bridge PASS for S3_004 (2.21 s), S3_007 (2.75 s), S3_012 (2.21 s),
  S3_022 (1.33 s), S3_028 (2.65 s), S3_030 (2.13 s). Steps: every design stops at ~0.23 (helper up step 1, then stall).
- N2X/P2X/P1X (XL, front motors + chains): ramp PASS 6.2-6.7 s; rope: front axle pulls out of the motor or the chain
  slips off (+1 mm frame stretch); bridge: tip off at ~0.6.
- "fr" designs without the rear left-right coupling (gN default) yaw on the start deck (rear-left wheel free) ->
  early bridge failures (S3_002/003/008/013/019, T1-T5). Fixed: sampler forces rb_drive for fr; gN adds a
  coupling axle above the cord in pair mode.
- U1-U3 (fr XL + coupling + pair rope wheels, wb 104): steps 0.49 (same as K3), ramp stuck 0.5 (fr gives only two
  well-driven wheels; the coupled wheels get ~14% torque), rope: front axle pulled out of the motor at 0.73.
- K3 steps record: rear wheels stall at riser 1 while the front wheel spins on the riser-2 nose. Cause: with wb 104
  the car is pitched 32 deg and the front sits exactly on a step nose when the rear has to climb. Pitch geometry
  says wb ~120-128 puts the front mid-tread then (horizontal wb = sqrt(wb^2 - 57.6^2)).
Queued: V1/V2 (ff XL + chains + one rigid 1x15 bottom beam so the chain can't stretch; V2 adds the axle stop),
W1/W2 (fr XL, wb 128 via an upper-beam extension, pair / flanged rope wheels).

## W5 / W6 / W7 (rb_high: free/driven rear rope wheels raised to R+72 on 1x7 posts at xR+48)
- W5 (fr XL, pair 56145, xR -72, rear_ext, rb_drive, helper, rb_high, free high rope B): rope 0.627 stuck; motor mesh gfL-igaL disengaged (slip, 6 N).
- W6 (= W5 + rb_chain: 24T chains cax -> high rope B axle, 1 link loop each side): bridge PASS 3.31 s, ramp PASS 8.89 s, steps 0.576 stuck, rope 0.686 stuck; gfL-igaL disengaged at 13 N peak (front-left motor gear walked off idler A). Chains locked, no skips.
- W7 (= W6 + mg 8 motor stop bush): queued rope + steps to test whether the motor-mesh separation is what stalls the rope exit. W6 rope record queued.

## W7-W14 (2026-11-19 evening)
- W7 (W6 + mg 8): rope 0.732 (best of W series), steps 0.575. No mesh slip. Record: front motor stalls on the uphill end of the sagging cord (rope A ~0-15 rpm) while rear rope pair B (R+72) spins at free speed (~188 rpm) -> B carries ~no load/traction.
- W8 (rbz 64, 16T 94925 sprockets): rope 0.439, steps 0.578, ramp PASS 8.87, bridge PASS 3.31.
- W11 (rbz 56 = same height as A, driven): rope 0.229 - stuck on the start saddle post (confirms B must be high to pass the post).
- W8 steps record: front wheels jammed against riser 3 (car pitched 14 deg, rear on step 1); helper wheel hovers ~5 mm above tread 3 -> never pulls. Both motors near stall.
- W12 (hz 0: helper lowered 8 mm to front-axle height, driven by 24T -> 40T; helper rim 0.69x ground): queued all 4 + steps record.
- W13 / W14 (rope B = classic 55982 single wheel riding ON TOP of the cord, at R+64 / R+72): weight-bearing, clears the post (bottom 80.1 / 88.1). Queued rope (+ W13 record).
- Runner crash (17:45, 18:37): process died with no traceback right after starting a batch -> agent_runner.py now validates in a child process (validate_sub). Restarted 19:02 by the user.
- W12 (hz 0): bridge PASS 3.17, ramp PASS 9.35, steps 0.582 (record: front wheel sits on the riser-3 nose, car pitched ~15 deg, helper still ~2 mm above tread 3; rear motor stalled). W11: ramp PASS 8.86, steps 0.576. W13 rope 0.734.
- W15 (hz -8: helper axle 8 mm below the front axle, driven 40T->40T on an inner plane y32 from a 3706 carrier axle; helper bottom 1.7 mm above flat ground): queued all 4 + steps record.
- W13 record: rear classic rope wheel now grips (rpm matches rope A), but BOTH motors stall at deck x~455 (same spot as W7). Diagnosis: torque-limited - both rope drives are geared UP (rope A 1.6x motor through 4 meshes at 39% loss each; rope B ~5x via cax 8T) -> ~1.6 N + ~1.1 N at the rims vs ~2 N of grade near the goal end.
- W16/W17/W18: ra_low (rope A geared down 5:1: 8T on the helper carrier axle (3706, y32) -> 40T on the rope axle; rope A rim ~0.23x ground speed, ~5x torque) ; W16/W18 also rope B chain 16T->24T (rb_sp 24). W16 queued on all 4 + rope record.
- W16/W17 (rope A geared down 5:1): rope still 0.73, stall at the SAME place (deck x ~456). W16 bridge PASS 3.09, ramp PASS 9.44, steps 0.582. W15 (hz -8): bridge PASS 4.81, ramp FAIL 0.663 (low helper), rope 0.447.
- KEY FINDING (W16 record): the stall is geometric, not torque. The front rope PAIR (56145 x2 straddling the cord) and the helper wheels reach x ~580 and hit the goal saddle post (x 582.5..602.5, |y|<5, 78.5 mm tall). A pair that straddles the cord can never pass a saddle post; only a wheel riding ON TOP of the cord (classic 55982 + 40T flanges at |y| 7.4..15.4) can. Every part below ~80 mm must stay outside |y| < ~6-7 mm (ground wheels' inner faces are at |y| 8; helper inner faces at 6.2 -> moved out to 8.2 with hy 19).
- W19 (classic rope A + classic B R+64, ra_low, rb 16->24) / W20 (classic A, no reductions) / W21 (= W19 + helper at y 19, 8T/40T rope reduction in plane y34): queued rope (+W21 all 4 + rope record).
- W19/W20/W21 (classic rope A): rope 0.731 / 0.725 / 0.73. W21: bridge PASS 2.98, ramp PASS 9.07, steps 0.529.
- W22 (classic A, helper at y 22, rb 16->24): bridge PASS 3.17, ramp PASS 8.31, steps 0.576, rope 0.735.
- W22 record: the car ROLLS progressively while crossing (1 deg at the start edge -> 25 deg at x 384 -> 50 deg at x 423) and drifts -14 mm sideways; the cord leaves the rope wheel tyre (rides the 40T flange), and the low-side helper wheel swings into the centre line and stops against the goal saddle post at deck x ~454 (helper front ~582 = post face). The deck-x-454 stall in W7..W22 is the helper (or the pair) meeting the goal post.
- Root cause: hang depth only ~11 mm (CoG 61 mm vs rope-wheel bottom 72 mm) -> almost no roll stiffness. Need a much lower CoG relative to the rope-wheel contact (or a higher rope-wheel contact), and nothing below the cord level within |y| < ~d*sin(roll) of the post.
- W24/W25: rope B classic at R+72 (bottom 88) to raise the support line behind the CoG (estimated ~23 mm depth at the CoG). Queued rope (+W24 record).
- W24/W25 (rope B R+72): rope 0.727; record shows roll growing to ~70 deg, rear rope wheel falls off the cord (worse).
- W26 (rope A raised to R+64 via 24T->40T, B R+72): rope 0.275 - hung up at the start edge (rear ground wheels on the 8 mm clamp arm, rope B slid 17 mm sideways off the cord at the start post).
- W27 (A R+64, B R+64): rope 0.733 (same goal-end stall).
- Status 22:40: bridge + ramp solved (many designs, ~3.0 s + ~8.3-9.4 s). Steps stuck at ~0.58 (front wheels on the riser-3 nose, helper cannot reach tread 3 without hurting the ramp). Rope stuck at ~0.73 because the hanging car has almost no roll stiffness (CoG only 11-19 mm below the rope-wheel contact) and rolls 25-70 deg, so low parts (helper wheels) swing into the goal saddle post.
- Next ideas: (1) lower the CoG a lot (battery/receiver/motors low beside the wheels, outside the |y|<6 tunnel) or raise both rope contacts above the deck with a slotted deck; (2) roll guides: two small free wheels/rails at |y| ~ 8-10 just below the cord level that bear on the cord sideways; (3) a centred, symmetric drivetrain (no diagonal 'fr' motors) so drive reactions do not roll the car; (4) steps: a third driven axle or a longer, lower helper arm that only drops below the front-wheel line when pitched.

## 2026-11-20 morning: specialists first (user: get steps-only and rope-only passes, then work back)
- New gN options: bat_low (both AAA boxes low and outboard on 1x6 Technic bricks pinned to the wheel segments, 4x8 plate, box on top; left box behind, right box in front -> laterally balanced; wheel axle lengthened through the brick with its bush outside), bat_ly, fl_y / fl_part (rope flange position/part).
- R6 (classic rope wheels at R+64 (bottom 80), fr XL, xR -56, NO helpers, bat_low 0): rope 0.936 (best ever). Record: crosses the whole cord, front wheels reach the goal deck, then stalls with the rear rope wheel's 40T flange hooked on the goal saddle post (car yawed ~11 deg / 6 mm sideways). Bridge FAIL (motr touched chasm floor), steps 0.14 (no helper), ramp 0.27.
- S1/S2 (W12 + low batteries): steps 0.518 / 0.082 - rear can't climb riser 1 while the front is two steps up (fr drive: one driven wheel per axle, front pull too weak).
- R7/R8/R9: R6 with flanges moved out (fl_y 15.4 / 13.4) or 24T flanges: rope queued. RS1/RS3: R6 platform + helper (hy 22 / 17): steps queued (+RS1 rope, steps record).
- R7/R8 (flanges out / 24T flanges): rope 0.235 / 0.222; R9 (fl_y 13.4): 0.799. R11/R12 (rope wheels R+72 on 1x11 posts, 40T rope gears): 0.239 - record: car yaws/shifts ~10 mm sideways as the front leaves the start deck (fr diagonal drive: only the rear-right wheel pushes) and the rear rope flange lands on the START saddle post. Same flange-on-post mechanism as R6's goal-end stall.
- Fix direction: symmetric drive (ff + chain, both motors at the front, both battery boxes low at the rear: bat_pos 'rear') so the car tracks straight. R13 (zx 64), R14 (zx 72), R15 (zx 64, fl_y 13.4) queued (R13 on all four + rope record).
- Steps: RS1/RS3 (R6 + geared helper) steps 0.227 (front wedged at riser 1, rear motor stalled, helper hovering). SC2 (chain-driven helper 1:1 straight from the front axle, hz 8, R+72 rope) queued on all four + steps record.
- R13/R14 (ff+chain, both boxes low at the rear): rope 0.218 / 0.079; R13 steps 0.151. Record: at the start edge the rear-left wheel's inner 4 mm rides up the 8 mm clamp arm (|y|<12), the car yaws, the rear rope flange lands on the start post.
- SC2 (chain helper): steps 0.219, bridge FAIL, ramp 0.319, rope 0.747. Steps record (and RS1): front wheel at riser 1, helper sitting on tread 1 but slipping, REAR MOTOR STALLED at 0 rpm in every bat_low design.
- Suspected bug in my bat_low mount: the wheel axle passes through the battery brick's hole without a declared hinge -> in the sim it may collide/jam (the rear-left (L box) and front-right (R box) wheels are the ones whose axles pass through the bricks). Fixed: axle declared as a hinge in the brick. R6b / RS1b / SC2b re-queued.
- R16/R17/R18: wheels moved out (wheel_y 18.2: inner face 12 clears the clamp arm) + flanges out (fl_y 14.4): rope + bridge queued.
- Hinge fix confirmed: designs built before it (R13/R16/R18) crossed the bridge in ~17 s (axles dragging in the brick holes); RS1b (same as RS1 + fix) steps 0.227 -> 0.412.
- RS1b steps record: front climbs riser 1 (after 2 s), then front wheels sit against riser 2 with the rear still on the floor (pitch ~16 deg); helper (hz 8) hovers 3.8 mm above tread 2. Geometry: with the rear one level lower, helper bottom = tread + 26.2 + 0.97*hz (wb 112) -> needs hz <= ~2.7; but at riser 1 (car level) the helper needs hz >= ~5 to meet the corner below its centre. One rigid helper height cannot serve both cases with wb 112/128.
- ST1 (hz 0, wb 112), S1b (W12 + fixed low boxes, wb 128, hz 0), S1c (same, hz -8) queued.
- ST1 (hz 0, wb 112): steps 0.087 (can't even climb riser 1); S1c (hz -8): 0.081; S1b (W12 + fixed low boxes, hz 0, wb 128): 0.516.
- bat_mount 2 (both boxes at the rear on 1x4 bricks clear of the wheel axles; ff+chain): R19 bridge PASS 9.4 s (slow), ramp 0.28, rope 0.223 (record: the rear-left wheel's inner face (y 8) rides up the 8 mm start clamp arm (|y|<12), the car jumps 14 mm sideways and stalls). R20 (= R19 + wheel_y 18.2, fl_y 14.4): passes the start, rope 0.579, bridge PASS 9.3, ramp 0.283, steps 0.151.
- All low-battery builds fail the ramp at ~0.28 and are slow on the bridge -> something drags; R20 ramp + rope records queued. R6d/e/f (R6 + wider wheels / outboard flanges) rope queued.
- **R6f PASSES THE ROPE (7.78 s)** - first rope pass. R6f = fr XL drive, classic rope wheels at R+64 (bottom 80 mm), 40T flanges at y +-11.4, NO helper, both AAA boxes low/outboard (diag: L box on the rear segment, R box on the front segment; wheel axles declared as hinges in the bricks), wheel_y 18.2 (wheel inner faces at 12 mm: clear of the 24 mm-wide clamp arms). R6d (+ flanges out to 14.4): 0.57; R6e (flanges 13.4, wheel_y 14.2): 0.798.
- R20 ramp record: all four wheels at 0-4 rpm from the foot of the ramp: the ff+chain+rb_drive drivetrain is a closed loop (FL-chain-RL-rope axle B-RR-chain-FR-rope axle A-FL) and binds under load -> explains the slow bridge (9-17 s) and ramp failures of R13/R16/R19/R20. Don't combine ff+chain with driven rope axles.
- R6f: rope PASS 7.78 s also with commands_50ms_late (offset_1mm pending); bridge FAIL (motr touched chasm floor at 0.39), ramp 0.282, steps 0.144 (no helper). Ramp/bridge records queued.
- T1/T2/T3: ff+chain with rope A driven and rope B FREE (no drivetrain loop), both boxes at the rear (mount 2), wheel_y 18.2, helper hz 8 / hz 0 / none: all four courses queued (+T1 steps record).
- R6f rope also PASSES offset_1mm (7.41 s): rope specialist done (plain / offset_1mm / 50ms-late all pass).
- R6f ramp record: front wheels spin (70 rpm) on the slope while the rear (fr: only the rear-right motor) stalls at the foot - fr gives effectively one driven wheel per axle, not enough on 30 deg with the extra 109 g box.
- T1 (ff+chain, rope B free): bridge PASS 2.36 s; ramp 0.34, steps 0.227, rope flipped 0.75. T1 steps record: front wheels spin while the rear wheels stand still - result shows both chains 'free' (thrown: gap -3.4 mm, sprocket axial 5 mm) -> separate front/rear segments let the wheelbase stretch under load.
- T4/T5/T6: T1 + longseg (one 1x15 per side carrying both axles: rigid wheelbase), rear boxes on two 1x4 bricks (3L pins through segment + both bricks, plate on two stud rows: no critical review issues). Helper hz 8 / hz 0 / none. All four courses + T4 steps/ramp records queued.
- **T3 (ff+chain, rope A driven / rope B free, no helper, separate segments, boxes rear on one 1x4 brick): ROPE PASS 3.42 s.** (bridge fell, ramp broke the rear axle cluster, steps 0.42)
- **T4 (longseg + helper hz 8 + two-brick box mount): bridge PASS 2.28, ramp PASS 7.33, rope 0.81, steps 0.491.**
- **T6 (= T4 without helper): bridge PASS 1.56 s, ramp PASS 3.92 s, rope 0.803, steps 0.161.**
- T5 (hz 0): steps broke immediately (stud joint released bplL/bbrL) - box joint still weak under impacts.
- T6 rope record + T7 (fl_y 13.4) / T8 (no longseg) / T9 (rope R+72) rope queued to find what separates T3 (pass) from T6 (0.80).
- T6 rope record: crosses fast (375 mm/s) then at the goal end the front rope wheel slips (107 rpm, car stopped) on the steep last part of the sagging cord just short of the saddle; rope B was free -> not enough rope traction.
- **T10 = T6 + rope B driven from the LEFT idler only (rb_side 'L': no left-right loop): ROPE PASS 3.79 s, RAMP PASS 5.27 s, BRIDGE PASS 1.70 s -> 3 of 4 courses, total 10.76 s.** T11 (rope wheels R+72): bridge 1.65, ramp 5.36, rope crossed but then left the lane.
- Queued: T10 nudges (offset_1mm, commands_50ms_late) on rope/ramp/bridge, T10 steps, T14 (T10 + helper hz 0 hy 22) all four. T13 (helper hz 8) is over the 24T-gear budget (11 > 10).
- T10 nudges: rope 50ms-late PASS 3.79; rope offset_1mm FAIL (left_lane after crossing - drives off the side of the goal deck); ramp both nudges PASS; bridge both nudges PASS. T10 steps 0.156.
- bat_mount 3 (needs longseg): two 1x6 bricks side by side outside the long segment at xR+40, two 3L pins each through segment + both bricks, 4x8 plate on 12 studs, box on 24 studs (clean review).
- **T15 (= T10 with mount 3): rope PASS 3.31, ramp PASS 5.32, bridge PASS 1.69 (3/4, 10.32 s total, clean validation).** steps 0.154. T15 nudges queued.
- T14 (T10 + helper hz 0): bridge 2.38, ramp 12.0 (slow), rope 0.77, steps broke at t=0.2 (old mount's stud joint). T16 (T15 + helper hz 0) and T17 (T15 + helper hz 8 hy 22, rope A driven from the left only to fit the 24T budget) on all four + steps records.
- Steps analysis (T4 record): with ff+chain both wheels of a side are locked together; when the rear wheel meets riser 1 (front already on tread 2, pitch 31 deg) the rear wheel's contact is nearly horizontal (4.6 deg) and the side jams: the front can only pull ~0.82*W_f, the rear needs ~1.03*W_r of pull -> needs >= ~56% of the weight on the front wheels at that pitch (static CoG ~67% front).
- **T15 nudges: rope offset_1mm PASS 4.31, rope 50ms-late PASS 3.31, ramp both PASS (5.31/5.32), bridge both PASS (1.68/1.69). T15 is the robust 3/4 design (only steps missing).**
- T16 (T15 + helper hz 0): steps 0.091, rope 0.747 (helpers reach the goal post), ramp 20.7 s (drag). Helpers hurt rope and ramp.
- T18 (T15 + helper hz 8 + boxes moved forward to xR+56, CoG x 23.9 ~71% front), T20 (T15 + boxes forward) queued.
- User (10:57): once a design passes all four, explore per-course speed specialists and port their ideas to the 4/4 design.
- T17 (T15 + helper hz 8, ra_side L) steps 0.23 / rope 0.77 / ramp 10.0 / bridge 2.44; T18 (+ boxes forward) steps 0.23, ramp 9.35, bridge 2.45; T20 (T15 + boxes forward) bridge 1.69.
- T17 steps record: at riser 1 EVERYTHING stalls at 0 rpm (front wheel lifted 9 mm on the corner, helper on tread 1, rear on the floor): with ff+chain the front and rear wheels of a side are locked 1:1 and the helper is geared to 1.15x rim speed, so the three contacts demand different speeds and the drivetrain jams (front wheel climbing a corner cannot translate at the speed the rear wheels impose).
- Steps sweep around T4 (best ff steps 0.491): T21 (mount 3), T22 (mount 3 + boxes forward, CoG 25.6 ~73% front), T23 (T22 + helper y 17) queued (+T22 record).
- **T20 (T15 with the boxes moved forward to xR+56): rope 3.38, ramp 4.92, bridge 1.69 = 3/4, 9.99 s total (new best total).**
- T21/T22/T23 (T4-like helper + mount 3, boxes rear/forward): steps all 0.23 - total drivetrain lock at riser 1 (T22 record: every wheel and helper at 0 rpm from t=1 s).
- Steps physics (written down): with rigid front/rear coupling (ff+chain) a front wheel pressed on a riser corner can only move forward at ~0.42x its rim speed while the rear wheels impose 1.0x -> static lock. Independent axles (fr) avoid the lock but then each axle has effectively one driven wheel (L-R coupling through the rope axle is ~14% efficient). Chain link budget (100) rules out chain L-R coupling.
- Idea: the sim's chain skips at 0.625 N per tooth of the smaller sprocket (16T: 10 N tension = 0.084 N m at the wheel) and keeps ratcheting at that force -> a 16T chain is a built-in SLIP CLUTCH between front and rear: the rear keeps pushing (~2.7 N/side) while the front is held, no lock. T26 (T4-like helper + 16T chains), T27 (+ boxes forward), T28 (T15 + 16T chains, no helper) on all four courses.
- 16T chains (slip clutch): T26 steps 0.495 (vs 0.23 with 24T), ramp 7.12, bridge 2.29, rope 0.77; T27 0.423; T30 (helper y17) 0.494; T31 (hz 0) 0.496.
- T28 (T15 + 16T chains, no helper): bridge 1.69, RAMP 2.94 s, rope crossed but left the lane on the goal deck (2.86), steps 0.449.
- **T32 = T26 with the battery back on the TOP deck (single box, receiver beside it): STEPS PASS 2.86 s - first steps pass ever.** Rope/ramp/bridge + steps nudges queued.
- **T32: steps PASS 2.86 (also offset_1mm and 50ms-late), bridge PASS 2.24, ramp PASS 4.38, rope 0.811 -> 3/4 including the steps.**
- **T35 (T32 without helpers + rope B driven left): bridge 1.63, ramp 2.79, rope 3.05 -> 3/4 at 7.47 s total (fastest); steps 0.153.**
- T34 (T32 + rope B driven): bridge 2.37, ramp 5.37, rope broke (deck stud joint dbRb/tbR) at 0.81.
- So: helpers are needed for the steps and are what stops the rope at ~0.81. T32 rope record queued.
- T34 steps PASS 3.06; T38 (T34-like + fl_y 13.4) steps PASS 3.14, rope 0.746.
- T32 rope record: the car drifts sideways progressively (front rope wheel y -0.6 -> -9.5 mm, roll 5-13 deg; hang depth only ~18 mm with the top box) and the front-left 40T flange (y +2) hooks the goal saddle post; front rope wheel then slips.
- U6 (T32 + rope wheels R+72: hang depth 23), U8 (helpers at y 17), U10 (T34 + R+72) queued on rope + steps.
- U6/U8/U10 (T32 + R+72 / helpers y17 / T34 + R+72): steps all PASS (3.07/2.81/3.19), rope 0.81/0.81/0.77.
- T35 vs T32 rope records: both drift ~-8..-13 mm sideways on the cord; T35 (no helpers) is fine, but in T32 the right helper (inner face 11 mm - 8 mm drift = 3 mm) runs into the goal saddle post (|y|<5) at rwf x~535, stalls, then the front flange hooks the post.
- U14/U15: helpers moved out to y 25 (inner face 14.2 mm), U15 also rope B driven left: rope + steps queued. U11-U13 (top box moved back) pending.
- **U12 (T32 + bat_x 16 + rb_drive L): steps PASS 3.06, rope PASS 3.62** (first steps+rope). U13 (U12 + hy 17): steps 2.99, rope 4.94. U11 (bat_x 16 only, rope B undriven): rope 0.77. Driving rope B is what gets it past the goal post. Ramp/bridge/nudges queued.
- **U12 = FIRST 4/4 (plain)**: steps 3.06, rope 3.62, bridge 2.38, ramp 5.35 → total 14.41 s. Bridge/ramp also pass 50ms-late. Recipe: gN.build(motor="58121c01", drive="ff", chain=True, spz=16, helper=True, rope_mode="classic", rope_wheel="55982c01", xR=-56, bat_x=16, bat_y=8, rx_y=-40, rb_drive=True, rb_side="L", mg=8, zx_off=64, wheel_y=18.2, hz=8, hy=22, longseg=True).
- U14 (hy 25): steps 2.85, rope 0.81. U15 (hy 25 + rb L): steps 3.07, rope 0.747 → moving helpers outboard does NOT help; bat_x 16 + rb drive is the key.
- Speed phase: V1/V2 (hz 12/16) invalid (helper gear train geometry); V3 (bat_x 8) overlaps deck; V4 (bat_x 24 + rb L = T34-like) queued; U12 ramp recorded for ramp-slowness analysis.
- U12 nudges: all PASS (rope offset 6.13, steps offset 3.05, ramp offset 5.36, bridge offset 2.38; 50ms-late identical to plain). brick check x4 queued.
- U12 ramp record: on 30° slope wheels run only 28 rpm (flat 110) -> motor near stall; helper train carries ~1.9 N even with helper wheels airborne (parasitic drag).
- Z3 (U12, helper=False): ramp 2.81. **Z1 (U12 with 8T helper gears removed -> helper wheels free idlers): steps PASS 2.42 (!), ramp 3.46.** Driven helpers are unnecessary.
- Z4 = Z1 minus all helper gears/idler axles, helper axle play closed with bushes (hfill*). Queued 4 courses. V4 (bat_x 24 + rb L) rope broke at 0.81 (stud joint) -> bat_x 16 matters.
- **U12 brick check: ACCEPTED on all 4 (steps, rope, ramp, bridge).** Official baseline 14.41 s.
- **Z1 4/4 (plain): steps 2.42, rope 3.12, bridge 1.70, ramp 3.46 = 10.70 s** (U12 14.41). brick check queued.
- Z4 (no helper gears at all): steps 2.28, ramp 3.37, bridge 1.70, rope reached goal 2.56 s but "left_lane after crossing" (fail). Record queued.
- Z4 rope failure cause: judge tracks the design's "chassis" part for left_lane (rope lane |y|<=100 mm, settle 5 s after goal). Our chassis was ubL at y=+60 → only 40 mm margin; Z4 yawed 3° after the goal and drifted out. FIX: chassis="deck" (centred plate). Z4c/Z1c = Z4/Z1 with chassis deck; Z4c queued on all 4.
- Z1 brick check: steps/ramp/bridge ACCEPTED; rope REJECTED (offset_1mm nudge stuck at 0.745). U12 remains the official accepted 4/4 (14.41 s).
- **Z4c (Z4 + chassis="deck") 4/4 plain: steps 2.28, rope 2.56, bridge 1.70, ramp 3.37 = 9.91 s.** Not yet nudge-checked / brick-checked.
- STOPPED here at user request (15:48). Next: brick check Z4c (watch the rope offset_1mm nudge, which Z1 failed); if it fails, rope-robustness tweaks on Z4c (wheel_y, hy, bat_x).
- 19:21 Z4c brick check: steps ACCEPTED (2.28; offset 2.29, late 2.28), ramp ACCEPTED (3.37), bridge ACCEPTED (1.70); rope REJECTED: plain 2.56 and 50ms-late 2.56 pass, offset_1mm stuck at 0.745 (same failure as Z1). U12 remains the only fully accepted 4/4 (14.41 s).
- Z4c rope offset_1mm diagnosis (records): failure is ROLL, not pitch. Plain: roll within ±7°. Offset: roll -36° at t≈1.3 s leaving the start platform, never recovers; stuck at x≈456 with -23° roll, rope wheels 12 mm off the cord (cord on the flange), both rope wheels spinning/slipping. Battery on the top deck → CG high → weak pendulum restoring.
- Tighter flanges (fl_y 8.6–10.2) invalid (collide with rope wheel). Centred battery (bat_y 0) invalid (receiver overlap / lead 2 mm short).
- S4 = Z4c + bat_low=0, bat_mount=3 (two boxes low, +128 g): rope plain PASS 3.39; offset_1mm roll peak only -18° and car got past the old stuck point to x≈597, then "broke (stud joint released: bplL/bbqL)" at t 3.34, prog 0.933. Low CG fixes the roll; the mount 3 plate-to-brick joint is too weak. Steps not yet tested (earlier low-battery designs failed steps).
- 11/21: U12-based variants queued: Q1 (U12+deck chassis), Q2 (U12 hz=0: 24T->40T helper, 1:1), Q5 (Z4c soft start L5 until 1.4 s). hchain impossible (150 links > 100). hz=-8 invalid (overlap).
- Q1 (U12+deck): steps 3.06, ramp 5.35 (= U12). Q2 (U12 hz 0, helpers 1:1): steps 3.32, ramp 5.23, rope offset FLIPPED 0.751 → worse.
- **Q5 = Z4c + soft start (level 5 for 0-1.4 s, then 7): rope plain 3.23, rope offset_1mm PASS 3.82.** Launching gently onto the cord avoids the roll. brick check x4 queued; Q6 (L6→1.4 s), Q7 (L5→1.0 s), Q8 (L4→1.0 s) rope plain+offset queued to tune the soft start.
- Q5 brick check: rope ACCEPTED (3.23 / off 3.82 / late 3.29), ramp ACCEPTED 3.94, bridge ACCEPTED 2.04, STEPS REJECTED (stuck in all 3: soft start leaves too little speed for free helpers at the first riser).
- Q6 (L6→1.4): rope 2.98 / off 3.49 ✓. Q7 (L5→1.0): rope 3.12 / off 4.32 ✓. Q8 (L4→1.0): rope 3.78. Steps plain queued for Q6-Q10; Q9 (L6→1.0), Q10 (L6→0.7) rope queued.
- Steps with soft start: L4/L5 starts stuck at first riser (Q7, Q8); L6 starts pass: Q6 2.56, Q9 (L6→1.0) 2.45, Q10 (L6→0.7) 2.45.
- Rope: Q9 2.94 / off 3.34; **Q10 2.77 / off 3.29**. Q10 brick check x4 queued.
- **Q10 brick check: ACCEPTED on all 4.** steps 2.45 (off 2.43, late 2.45), rope 2.77 (off 3.29, late 2.91), ramp 3.46 (off 3.43, late 3.37), bridge 1.779 (all). **Total 10.46 s** — new best accepted (U12 14.41).
  Q10 = Z4c hardware + motor_program [{t:0, all, level 6}, {t:0.7, all, level 7}].

## 2026-11-22: Ev1 = Q10 + start_gap_mm 0 (final_design/Ev1.json)
- Q10 had no `start_gap_mm`, so each course used its default gap (steps 30, rope 60, ramp 38, bridge 30 mm). The goal is
  judged on the vehicle's REAR, so all of that is extra distance. Setting `start_gap_mm: 0` (allowed: 0 <= gap) cuts 158 mm.
- Same machine (Linux VM), plain runs: Q10 1.779 + 2.44 + 3.48 + 2.92 = 10.62 s -> **Ev1 (gap 0): bridge 1.729, steps 2.36,
  ramp 3.37, rope 2.67 = 10.13 s**. All 12 acceptance runs pass: steps off 2.42 / late 2.36, rope off 2.55 / late 2.74,
  ramp off 3.33 / late 3.38, bridge off/late 1.729. Validate + review clean on all 4.
- Rope robustness (the knife-edge course): gap 0 passes start offsets -0.5..+1.5 mm (incl. the official +1 mm), 0.2/0.5 mm
  micro-offsets and 10/20/30 ms delays (6/6 micro-perturbations); fails at -0.8, -1 and +2 mm. Old Q10 (gap 60) passes
  those, but nearly stalls (4.49 s at -0.5 mm, 5.19 s at -1 mm). Gaps 10 and 30 fail -1 mm / the official +1 mm run.
- Failure mechanism (records): not roll but sideways creep along the cord while climbing the far side (lateral offset
  grows to 10-18 mm, the cord climbs a 40T flange, stall at deck x ~455). Present in every run (plain drifts to -11 mm and
  survives); root cause is the short hang depth (~14 mm). A real fix needs a lower CoG relative to the rope-wheel contact.
- Tried and rejected: L motors (bridge 1.35 s but steps/ramp stuck), full power from t=0 (rope plain stuck), soft start
  to 0.5 / 0.9 s (no rope robustness gain), removing the dead rear-right idler igbR/grR (-8 g, ramp -0.11 s, but the rope
  official offset run fails: lateral balance), smaller 55982c01 helper wheels (-14 g, steps stuck).
- Ramp analysis: on the 30 deg board the XL motors run at ~46 rpm (flat 170): shared-supply sag (supply_r 0.39) + 14%
  motor friction leave them near stall. Mass is the main lever there (simulator models one global supply, so a second
  battery box does not help).

## 2026-11-22 evening: Ev2 = Ev1 + two pullback motors (final_design/Ev2.json), 7.11 s
- Why: the ramp recording showed the XL motors near stall (46 rpm on the slope). The simulator's battery is one shared
  supply (supply_r 0.39), so extra PF motors make the sag worse (2 XL + 2 L estimated to give LESS ramp torque) and
  4 PF motors also need a 2nd battery box (max 3 per box). The 12787c01 pullback (spring) motors bypass the supply:
  0.05 N m x (1 - turns used / 10) x (1 - w / 600 rpm), released at t = 0, ignore the program ("all" skips them).
- Mount: one per rear wheel axle (axrL/axrR, the 8L axle 3707), body pointing forward (yaw 180) at y +-72..96. The axle
  is the pullback's motor shaft, so its hinge declarations were removed (a motor shaft may not also be declared turning
  in a hole; same as the front XL axles); bush 3713 at y +-68 between the 1x15 side beam and the pullback stops it
  sliding; the rear stop bush aburL/R was removed. The pullback's bottom pin-hole row is 8 mm below the side beam's
  holes, so two vertical 1x3 liftarms (32523, x -32 and -16, plane y +-68) bridge them with 2L pins. +66 g (684 g).
  Generator: tools/pb.py (post-processes the Ev1 json).
- PB1 (wind 10, Q10 program): plain steps 1.539 / rope 2.23 / ramp 1.749 / bridge 1.409 = 6.93 s, but rope 1 mm offset
  stuck at 0.727 (the usual sideways creep).
- PBw6 (wind 6) and PBl5 (XL level 5 for 0.7 s, then 7) pass the rope offset run; PBng (default gaps) fails it.
- **Ev2 = PBl5: all 12 acceptance runs pass.** Plain steps 1.589, rope 2.20, ramp 1.849, bridge 1.469 = **7.11 s**;
  offset 1.599 / 2.70 / 1.839 / 1.469; 50 ms late 1.599 / 2.21 / 1.859 / 1.479. Validate + review clean (no critical).
- Rope robustness: passes offsets -2, -1, -0.5 (slow 4.66 s), -0.2, +0.2, +0.5, +1, +2 mm and 20 / 50 ms delays (11/11).
  Ev1 failed -1, -0.8 and +2 mm.
- Open: review warns that each pullback hangs on a single-pin linkage (the two 1x3s can rack); add a brace (e.g. a 1x5
  at x -48 into the pullback's upper pin hole) if a real build is planned.

## 2026-11-22 late: Ev3 (final_design/Ev3.json), 7.00 s, rope made robust
- Ev2 scored 8.7 s on Evan's Mac: steps/ramp/bridge as here, but rope 3.8 s (2.20 s here) - the slow "stall then
  recover" rope mode (seen here once, 4.66 s at -0.5 mm).
- Slow-run record: car drifts ~12 mm sideways while climbing the far side of the cord and sits 2.5 s at deck x ~471.
  The free helper wheels' inner faces were at |y| 11.2 and their axles reached |y| 7, vs a ~+-4 mm goal saddle post.
  H26 (tools/hout.py): helper wheels to |y| 26, axles shifted 8 mm out (inner ends |y| 15) -> the -0.5 mm case 2.75 s.
- Remaining failures (cord riding up a rope-wheel flange after a 10-12 mm drift with ~25 deg roll) came from the rear
  rope wheel being driven from the LEFT only (mesh forces 44 mm off centre roll the car; larger with the pullbacks).
  RB2 (tools/rb.py both): add 24T rgrR on the rear rope axle meshing the formerly idle igbR -> rope driven from both
  sides. The 16T slip chains keep the new loop from binding.
- Program: full power (L7) and L6->0.7 fail the rope offset / +0.8 mm runs; L5 start is needed. L5 for 0.5 s (RB2a)
  is faster than 0.7 s and passes everything.
- **Ev3 = RB2a**: plain steps 1.689 / rope 2.15 / ramp 1.729 / bridge 1.429 = **7.00 s**; offset 1.679 / 2.33 / 1.739
  / 1.429; late 1.679 / 2.26 / 1.749 / 1.439. Rope passed 14/14 perturbations on two machines (offsets -2..+2 mm,
  incl. 0.2/0.5/0.8 mm, 50 ms late), 2.14-2.68 s. Validate + review clean, 685 g.
