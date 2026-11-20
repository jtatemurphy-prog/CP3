# Parts you may use (the owner's inventory)

Only these parts, at most the owned count of each. Sizes are the bounding box in the part's own frame (mm; x, y, z or diameter x width). Tyre friction is each tyre's measured value; motor ratings are nominal. Full geometry (studs, anti-studs, holes, pin holes, axle lengths, teeth, tyre friction, motor line):

```python
from bricks.parts import bench_parts
bench_parts()['3895']
```

## Technic bricks, liftarms and frames

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `6541` | Technic brick 1x1 with hole | 30 | 0.38 | 8.0 x 8.0 x 9.6 | 1 holes; 1 studs |
| `2730` | Technic brick 1x10 with holes | 15 | 3.67 | 80.0 x 8.0 x 9.6 | 9 holes; 10 studs |
| `3895` | Technic brick 1x12 with holes | 15 | 4.2 | 96.0 x 8.0 x 9.6 | 11 holes; 12 studs |
| `32018` | Technic brick 1x14 with holes | 15 | 4.92 | 112.0 x 8.0 x 9.6 | 13 holes; 14 studs |
| `3703` | Technic brick 1x16 with holes | 15 | 5.87 | 128.0 x 8.0 x 9.6 | 15 holes; 16 studs |
| `32000` | Technic brick 1x2 with 2 holes (under the studs) | 20 | 0.73 | 16.0 x 8.0 x 9.6 | 2 holes; 2 studs |
| `3700` | Technic brick 1x2 with hole | 30 | 0.82 | 16.0 x 8.0 x 9.6 | 1 holes; 2 studs |
| `3701` | Technic brick 1x4 with holes | 31 | 1.46 | 32.0 x 8.0 x 9.6 | 3 holes; 4 studs |
| `3894` | Technic brick 1x6 with holes | 20 | 2.2 | 48.0 x 8.0 x 9.6 | 5 holes; 6 studs |
| `3702` | Technic brick 1x8 with holes | 20 | 2.85 | 64.0 x 8.0 x 9.6 | 7 holes; 8 studs |
| `6588` | Technic gearbox 2x4x3 1/3 (worm + 24T housing) | 1 | 4.5 | 32.0 x 16.0 x 32.0 | 10 pin holes |
| `32140` | Technic liftarm L 2x4 thick (90 deg) | 10 | 1.43 | 31.2 x 8.0 x 15.2 | 4 holes |
| `32056` | Technic liftarm L 3x3 thin (4 mm) | 11 | 0.73 | 23.2 x 4.0 x 23.2 | 2 holes |
| `32526` | Technic liftarm L 3x5 thick (90 deg) | 10 | 1.77 | 39.2 x 8.0 x 23.2 | 7 holes |
| `60484` | Technic liftarm T 3x3 thick | 10 | 1.19 | 23.2 x 8.0 x 23.2 | 5 holes |
| `32348` | Technic liftarm bent 1x7 (4-4, 53.13 deg) | 10 | 2 | 45.6 x 8.0 x 26.4 | 5 holes |
| `6629` | Technic liftarm bent 1x9 (6-4, 53.13 deg) | 10 | 2.6 | 61.6 x 8.0 x 26.4 | 7 holes |
| `32271` | Technic liftarm bent 1x9 (7-3, 53.13 deg) | 10 | 2.63 | 64.8 x 8.0 x 20.0 | 7 holes |
| `32009` | Technic liftarm double bent 1x11.5 thick (45 deg) | 8 | 3.46 | 71.2 x 8.0 x 39.2 | 8 holes |
| `64178` | Technic liftarm frame thick 5x11 open centre (cross bars 2 holes in from the ends) | 2 | 7 | 88.0 x 8.0 x 40.0 | 8 holes; 20 pin holes |
| `64179` | Technic liftarm frame thick 5x7 open centre | 12 | 5.3 | 56.0 x 8.0 x 40.0 | 8 holes; 12 pin holes |
| `32525` | Technic liftarm thick 1x11 | 10 | 2.8 | 87.2 x 8.0 x 7.2 | 11 holes |
| `41239` | Technic liftarm thick 1x13 | 6 | 3.3 | 103.2 x 8.0 x 7.2 | 13 holes |
| `32278` | Technic liftarm thick 1x15 | 6 | 4 | 119.2 x 8.0 x 7.2 | 15 holes |
| `32523` | Technic liftarm thick 1x3 | 11 | 0.7 | 23.2 x 8.0 x 7.2 | 3 holes |
| `32316` | Technic liftarm thick 1x5 | 10 | 1.25 | 39.2 x 8.0 x 7.2 | 5 holes |
| `32524` | Technic liftarm thick 1x7 | 10 | 1.74 | 55.2 x 8.0 x 7.2 | 7 holes |
| `40490` | Technic liftarm thick 1x9 | 10 | 2.59 | 71.2 x 8.0 x 7.2 | 9 holes |
| `57515` | Technic steering arm 6x2 with tow ball socket (fork + socket) | 4 | 1.52 | 47.6 x 16.0 x 7.2 | 6 pin holes |

## Bricks and plates

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `3001` | Brick 2x4 | 500 | 2.32 | 32.0 x 16.0 x 9.6 | 8 studs |
| `4477` | Plate 1x10 | 15 | 1.61 | 80.0 x 8.0 x 3.2 | 10 studs |
| `3023` | Plate 1x2 | 44 | 0.36 | 16.0 x 8.0 x 3.2 | 2 studs |
| `3623` | Plate 1x3 | 40 | 0.53 | 24.0 x 8.0 x 3.2 | 3 studs |
| `3710` | Plate 1x4 | 36 | 0.71 | 32.0 x 8.0 x 3.2 | 4 studs |
| `3022` | Plate 2x2 | 45 | 0.64 | 16.0 x 16.0 x 3.2 | 4 studs |
| `3021` | Plate 2x3 | 40 | 0.93 | 24.0 x 16.0 x 3.2 | 6 studs |
| `3020` | Plate 2x4 | 55 | 1.2 | 32.0 x 16.0 x 3.2 | 8 studs |
| `3795` | Plate 2x6 | 35 | 1.74 | 48.0 x 16.0 x 3.2 | 12 studs |
| `3034` | Plate 2x8 | 51 | 2.27 | 64.0 x 16.0 x 3.2 | 16 studs |
| `3030` | Plate 4x10 | 10 | 5.55 | 80.0 x 32.0 x 3.2 | 40 studs |
| `3031` | Plate 4x4 | 10 | 2.22 | 32.0 x 32.0 x 3.2 | 16 studs |
| `3032` | Plate 4x6 | 10 | 3.3 | 48.0 x 32.0 x 3.2 | 24 studs |
| `3035` | Plate 4x8 | 10 | 4.7 | 64.0 x 32.0 x 3.2 | 32 studs |
| `3033` | Plate 6x10 | 10 | 7.95 | 80.0 x 48.0 x 3.2 | 60 studs |
| `3028` | Plate 6x12 | 10 | 9.47 | 96.0 x 48.0 x 3.2 | 72 studs |
| `3958` | Plate 6x6 | 10 | 4.71 | 48.0 x 48.0 x 3.2 | 36 studs |
| `3036` | Plate 6x8 | 10 | 6.4 | 64.0 x 48.0 x 3.2 | 48 studs |
| `92438` | Plate 8x16 | 1 | 17 | 128.0 x 64.0 x 3.2 | 128 studs |
| `41539` | Plate 8x8 | 10 | 9.4 | 64.0 x 64.0 x 3.2 | 64 studs |

## Connectors

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `32013` | Technic angle connector #1 | 11 | 0.46 | 8.0 x 15.6 x 7.2 | 1 pin holes |
| `32034` | Technic angle connector #2 (180 deg) | 11 | 0.68 | 8.0 x 24.0 x 7.2 | 1 pin holes |
| `32015` | Technic angle connector #5 (112.5 deg) | 11 | 0.69 | 8.0 x 19.8 x 16.0 | 1 pin holes |
| `6536` | Technic axle and pin connector perpendicular | 21 | 0.39 | 8.0 x 8.0 x 15.2 | 1 pin holes |
| `32184` | Technic axle and pin connector perpendicular 3L | 21 | 0.64 | 8.0 x 8.0 x 23.2 | 1 pin holes |
| `42003` | Technic axle and pin connector perpendicular 3L with 2 pin holes | 21 | 0.77 | 8.0 x 8.0 x 23.2 | 2 pin holes |
| `57585` | Technic axle connector hub with 3 axles | 4 | 0.65 | 21.6 x 8.0 x 19.5 |  |
| `6575` | Technic cam (4 mm thick, 4 cross holes in a line) | 2 | 0.7 | 15.2 x 4.0 x 23.2 |  |
| `6641` | Technic changeover catch (lever with a cross hole and an axle socket) | 4 | 0.5 | 20.8 x 8.0 x 7.2 |  |
| `35188` | Technic changeover rotary catch (round, cross hole) | 2 | 1.08 | ø 22.1 x 11.8 |  |
| `65414` | Technic differential housing, new style (no ring gear: a plain part) | 3 | 2.31 | ø 21.0 x 22.8 | 1 pin holes |
| `48989` | Technic pin connector perpendicular 3L with 4 pins | 5 | 1.22 | 8.0 x 24.0 x 23.2 | 3 pin holes |
| `55615` | Technic pin connector perpendicular 3x3 bent with 4 pins | 5 | 1.9 | 31.6 x 8.0 x 31.6 | 5 pin holes |

## Axles

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `92906` | Technic CV joint cup with integral axle (turns in a hole, takes a wheel / gear) | 4 | 0.45 | 22.4 x 8.0 x 8.0 |  |
| `23948` | Technic axle 11 | 8 | 1.65 | ø 4.4 x 87.6 |  |
| `50451` | Technic axle 16 | 8 | 2.37 | ø 4.4 x 127.6 |  |
| `32062` | Technic axle 2 notched | 50 | 0.26 | ø 4.4 x 15.6 |  |
| `4519` | Technic axle 3 | 33 | 0.43 | ø 4.4 x 23.6 |  |
| `24316` | Technic axle 3 with stop | 8 | 0.45 | ø 4.4 x 23.8 |  |
| `6587` | Technic axle 3 with stud | 8 | 0.48 | ø 4.4 x 23.6 |  |
| `3705` | Technic axle 4 | 20 | 0.59 | ø 4.4 x 31.6 |  |
| `87083` | Technic axle 4 with stop | 8 | 0.59 | ø 4.4 x 31.8 |  |
| `32073` | Technic axle 5 | 20 | 0.66 | ø 4.4 x 39.6 |  |
| `15462` | Technic axle 5 with stop | 8 | 0.73 | ø 4.4 x 39.8 |  |
| `3706` | Technic axle 6 | 44 | 0.86 | ø 4.4 x 47.6 |  |
| `44294` | Technic axle 7 | 20 | 1.05 | ø 4.4 x 55.6 |  |
| `3707` | Technic axle 8 | 16 | 1.18 | ø 4.4 x 63.6 |  |
| `60485` | Technic axle 9 | 8 | 1.3 | ø 4.4 x 71.6 |  |

## Pins

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `65249` | Technic axle-pin 1L pin (no friction) + 2L axle | 8 | 0.3 | ø 4.4 x 24.0 |  |
| `43093` | Technic axle-pin 1L/1L with friction | 153 | 0.26 | ø 4.4 x 16.0 |  |
| `15100` | Technic pin 1L with friction and perpendicular pin hole | 7 | 0.4 | ø 4.4 x 15.6 | 1 pin holes |
| `87082` | Technic pin 3L with centre pin hole | 5 | 0.5 | ø 4.4 x 24.0 | 1 pin holes |
| `6558` | Technic pin 3L with friction ridges | 101 | 0.37 | ø 4.4 x 24.0 |  |
| `32054` | Technic pin 3L with friction ridges and stop bush | 102 | 0.33 | ø 4.4 x 24.0 |  |
| `2780` | Technic pin with friction ridges | 121 | 0.2 | ø 4.4 x 16.0 |  |
| `3673` | Technic pin without friction ridges (pivot) | 143 | 0.16 | ø 4.4 x 16.0 |  |

## Bushes

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `32494` | Technic CV joint ball (axle driver; cross-fit on an axle) | 4 | 0.55 | ø 7.2 x 24.0 |  |
| `3713` | Technic bush | 10 | 0.14 | ø 7.2 x 8.0 |  |
| `4265c` | Technic bush 1/2 smooth | 66 | 0.1 | ø 7.2 x 4.0 |  |

## Gears

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `69778` | Gear 12 tooth (spur, 4 mm wide teeth) | 8 | 0.5 | ø 14.2 x 8.0 | 12 teeth |
| `6589` | Gear 12 tooth bevel (meshes at right angles only) | 35 | 0.29 | ø 12.8 x 4.0 | 12 teeth |
| `32270` | Gear 12 tooth double bevel (meshes as a spur gear) | 45 | 0.58 | ø 13.6 x 8.0 | 12 teeth |
| `69762` | Gear 14 tooth bevel thick (half bevel, teeth on the -y face) | 6 | 1.2 | ø 21.6 x 8.0 | 14 teeth |
| `4019` | Gear 16 tooth (axle hole, open sides) | 1 | 0.7 | ø 17.3 x 8.0 | 16 teeth |
| `94925` | Gear 16 tooth (reinforced) | 20 | 0.7 | ø 17.3 x 8.0 | 16 teeth |
| `18946` | Gear 16 tooth with clutch on both sides (round bore: turns freely) | 10 | 0.84 | ø 17.4 x 8.0 | 16 teeth |
| `6542b` | Gear 16 tooth with clutch, smooth centre (round bore: turns freely) | 8 | 0.7 | ø 17.5 x 8.0 | 16 teeth |
| `32198` | Gear 20 tooth bevel (meshes at right angles only) | 33 | 0.75 | ø 20.8 x 4.0 | 20 teeth |
| `32269` | Gear 20 tooth double bevel (meshes as a spur gear) | 44 | 1.4 | ø 21.6 x 8.0 | 20 teeth |
| `69761` | Gear 22 tooth bevel half with pin hole (round bore: turns freely) | 3 | 2.2 | ø 32.9 x 8.0 | 22 teeth |
| `3648` | Gear 24 tooth | 10 | 1.17 | ø 25.9 x 7.7 | 24 teeth |
| `32498` | Gear 36 tooth double bevel (meshes as a spur gear) | 6 | 3.5 | ø 37.6 x 8.0 | 36 teeth |
| `3649` | Gear 40 tooth | 18 | 3.76 | ø 41.9 x 8.0 | 40 teeth |
| `10928` | Gear 8 tooth | 39 | 0.22 | ø 9.9 x 8.0 | 8 teeth |
| `32072` | Technic knob wheel (4 knobs; meshes with another knob wheel, 1 : 1) | 26 | 0.87 | ø 24.0 x 8.0 | 4 teeth |
| `4716` | Worm gear (single start, meshes at right angles with a spur gear: 1 : z) | 11 | 0.6 | ø 10.2 x 16.0 | worm (1 : z) |

## Gear racks

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `3743` | Technic gear rack 1x4 (10 teeth, studs underneath) | 22 | 0.79 | 32.0 x 8.0 x 5.6 |  |
| `87761` | Technic gear rack 1x7 with axle and pin holes (8 teeth, end stops) | 2 | 1.8 | 56.0 x 8.0 x 8.0 | 2 pin holes |
| `6630` | Technic gear rack 1x8 with holes (14 teeth) | 4 | 2 | 64.0 x 8.0 x 5.6 | 2 pin holes |

## Differentials

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `62821b` | Technic differential (one-piece, 28T bevel ring) | 2 | 2.8 | ø 28.8 x 24.0 | 2 pin holes |

## Shock absorbers

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `731c04` | Technic shock absorber 6.5L, hard spring (a sprung slide between its end holes) | 6 | 2.1 | 8.0 x 8.0 x 52.0 | 2 pin holes |
| `2909c03` | Technic shock absorber 9.5L, soft spring (a sprung slide between its end holes) | 2 | 4 | 17.2 x 10.4 x 72.0 | 1 pin holes |

## Wheels and sprockets

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `55423c01` | RC train wheel with rubber friction band (55423) | 8 | 1.1 | ø 24.0 x 5.0 | radius 12 mm; modelled as hard ABS (no tyre friction) |
| `57519` | Technic tread sprocket wheel large (40.4 mm lobes, cross hole, 6 pin holes) | 4 | 5.1 | ø 40.8 x 14.6 | radius 20.4 mm; hard ABS; 6 pin holes |
| `57520` | Technic tread sprocket wheel small (25.4 mm, 6 lobes, 12 tread links per turn) | 10 | 1.63 | ø 25.6 x 8.0 | radius 12.8 mm; hard ABS |
| `4185` | Technic wedge-belt wheel (24 mm) | 4 | 0.97 | ø 24.0 x 4.0 | radius 12 mm; hard ABS |
| `4185t` | Wedge-belt wheel 4185 with rubber tyre 2815 | 4 | 1.72 | ø 30.4 x 4.0 | radius 15.2 mm; rubber tyre, friction 0.37 |
| `55982c01` | Wheel 18 x 14 rim with tyre 30.4 x 14 offset tread (55982 / 30391) | 4 | 8.37 | ø 30.4 x 14.8 | radius 15.2 mm; rubber tyre, friction 1 |
| `56145c01` | Wheel 30.4 x 20 rim with tyre 43.2 x 22 ZR (56145 / 44309) | 8 | 15.1 | ø 43.2 x 21.6 | radius 21.6 mm; rubber tyre, friction 1.19 |
| `32020c01` | Wheel 62.4 x 20 mm with rubber tyre (32020 / 32019) | 4 | 33 | ø 62.6 x 20.0 | radius 31.32 mm; rubber tyre, friction 1.38 |

## Motors

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `99499c01` | Power Functions L-motor | 2 | 45 | 24.0 x 56.0 x 31.6 | 390 rpm, 0.18 N m stall; 14 pin holes |
| `58120c01` | Power Functions M-motor | 2 | 33.5 | 23.2 x 48.0 x 23.6 | 400 rpm, 0.15 N m stall |
| `58121c01` | Power Functions XL-motor | 2 | 72 | 40.0 x 48.0 x 39.5 | 220 rpm, 0.4 N m stall; 10 pin holes |
| `99498c01` | Power Functions servo motor (88004) | 2 | 44 | 24.0 x 55.6 x 39.6 | position motor ±90°, 0.10 N m drive torque, 90° in 0.12 s; 12 pin holes |
| `12787c01` | Pullback motor 6 x 5 x 3 (spring motor, not IR controlled) | 2 | 31 | 50.0 x 24.0 x 39.2 | 600 rpm, 0.05 N m stall; 14 pin holes |

## Battery boxes

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `59510c01` | Power Functions AA battery box (8881) | 2 | 191.5 | 88.0 x 32.0 x 56.0 | 12 pin holes |
| `87513c01` | Power Functions AAA battery box | 2 | 109 | 64.0 x 32.0 x 38.4 | 18 studs |

## IR receivers

| id | name | owned | mass g | size mm | notes |
|---|---|---|---|---|---|
| `58123c01` | Power Functions IR receiver (8884) | 2 | 20.6 | 32.0 x 32.0 x 32.0 |  |

## Chain, tread links and tyres

| id | name | owned | used through |
|---|---|---|---|
| `3711` | Technic link chain | 100 | a design's `chains` list; each counts as a piece |
| `3873` | Technic tread link | 50 | a design's `treads` list on 57520 sprockets; each counts as a piece; hard ABS (simulated μ 0.42 on the boards) |
| `57518` | Technic tread link wide with 2 pin holes | 52 | a design's `treads` list with `"link": "57518"` on 57519 sprockets; each counts as a piece; hard ABS (simulated μ 0.42 on the boards) |
| `2815` | Tyre for the 4185 wedge-belt wheel | 4 | the composite wheel `4185t` (one 4185 + one 2815); each counts as a piece |
