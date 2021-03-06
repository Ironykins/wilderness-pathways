patches-own [
  difficulty
  diff_mult
  integrity
  initial_integrity
  biome
  basecolor
  f g h parent-patch ; Used for A* pathfinding speedups. Rewritten each time an agent tries to find a path.
  msmoothnoise
  mbasicnoise
  last_trampled
]

turtles-own [destpatch current-path last_pathupdate traveled]
globals [biome-list max-integrity seed mem_smoothnoise lasttrip update_trip_plot numtrips totaldist]
breed [points point]

to setup
  clear-all
  reset-ticks
  ifelse map_seed != -1
  [ set seed map_seed]
  [ set seed new-seed ]
  set update_trip_plot false

  random-seed seed

  ; list <deterioration> <diff_mult> <base colour>
  ; When walked on, the integrity deteriorates to (deterioration * integrity)
  ; The difficulty is diff_mult * integrity
  set biome-list (list
    (list 0.85 40 50) ; Forest. Green.
    (list 0.95 16 30) ; Rocky. Brown.
    (list 0.40 120 0) ; Snow. Grayscale.
    (list 0.60 32 90) ; Fern. Sky Blue.
    (list 0.85 80 110) ; Jungle. Violet.
    (list 0.70 96 120) ; Swamp. Magenta
  )
  makemap ; Make Terrain
  clear-turtles ; Remove the turtles used for biome generation.
end

to go
  set update_trip_plot false
  ;then we spawn turtles
  spawn_phase
  ;then turtles make their move
  move_phase
  ;then we deteriorate the new positions
  deterioration_phase
  ;Finally, we update the map
  updatemap
  display
  tick
end

;start the deterioration of the map
to deterioration_phase
  ask turtles [
    ;; Reduce the integrity of patches being stepped on
    ask patch-here [
      let multRate item 0 (item biome (biome-list))
      set integrity multRate * integrity
      set last_trampled ticks
    ]

    ;; Kill turtles who have gotten to their destination
    if patch-here = destpatch [
      set lasttrip traveled
      set update_trip_plot true
      set totaldist totaldist + lasttrip
      set numtrips numtrips + 1
      die
    ]
  ]
end

;start the phase of spawning turtles
to spawn_phase
 if (count turtles) < max_turtles [if (ticks mod spawn_frequency) = 0 [spawn_turt]]
end

;spawn a turtle at random border position, set destination opposite position
to spawn_turt
  crt 1 [
     let turt_edge random 4
     let turt_coords (point-on-edge turt_edge)
     set xcor first turt_coords
     set ycor last turt_coords

      set color red
      set size 6

      let dest_edge random 3
      if dest_edge >= turt_edge [set dest_edge dest_edge + 1]
      let dest_coords (point-on-edge dest_edge)
      set destpatch patch (first dest_coords) (last dest_coords)
      set current-path astar patch-here destpatch
  ]
end

;; Gets a random point on a specific edge of the map. 1 = north, 2 = south, 3 = west, 4 = east
to-report point-on-edge [edge]
  ; Initially, position is random.
  let x_cor (random (2 * max-pxcor)) - max-pxcor
  let y_cor (random (2 * max-pycor)) - max-pycor

  ifelse edge = 0 [ set y_cor max-pycor ]
  [ ifelse edge = 1 [ set y_cor min-pycor]
    [ ifelse edge = 2 [ set x_cor min-pxcor ]
      [ ifelse edge = 3 [ set x_cor max-pxcor ]
      []]]]

  report list x_cor y_cor
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Movement and Pathfinding

;start the move phase
to move_phase
  ask turtles [
      ; Make the turtle move along its path
      if length current-path != 0 [
          let next first current-path
          set current-path remove-item 0 current-path
          face next
          move-to next
          set traveled traveled + [difficulty] of next
      ]

      if last_pathupdate > path_recalc_interval [
        set current-path astar patch-here destpatch
        set last_pathupdate 0
      ]


      set last_pathupdate last_pathupdate + 1
    ]
end

; the actual implementation of the A* path finding algorithm
; it takes the source and destination patches as inputs
; and reports the optimal path if one exists between them as output
to-report astar [ source-patch destination-patch]
  ; initialize all variables to default values
  let search-done? false
  let search-path []
  let current-patch 0
  let open []
  let closed []
  let curDist distance destination-patch

  ; add source patch in the open list
  set open lput source-patch open

  if debug [ ask destination-patch [set pcolor green]]

  ; loop until we reach the destination or the open list becomes empty
  while [ search-done? != true]
  [
    ifelse length open != 0
    [
      ; sort the patches in open list in increasing order of their f() values
      set open sort-by [[f] of ?1 < [f] of ?2] open

      ; take the first patch in the open list
      ; as the current patch (which is currently being explored (n))
      ; and remove it from the open list
      set current-patch item 0 open
      set open remove-item 0 open

      ; add the current patch to the closed list
      set closed lput current-patch closed

      ; explore the Von Neumann (left, right, top and bottom) neighbors of the current patch
      ask current-patch
      [
        ; if any of the neighbors is the destination stop the search process
        ifelse any? neighbors4 with [ (pxcor = [ pxcor ] of destination-patch) and (pycor = [pycor] of destination-patch)]
        [ set search-done? true ]
        [
          ; the neighbors should not be obstacles or already explored patches (part of the closed list)
          ask neighbors4 with [ (not member? self closed) and (self != parent-patch) and (distance destination-patch < (curDist + max_backtrack)) ]
          [
            ; the neighbors to be explored should also not be the source or
            ; destination patches or already a part of the open list (unexplored patches list)
            if not member? self open and self != source-patch and self != destination-patch
            [
              ; add the eligible patch to the open list
              set open lput self open

              ; update the path finding variables of the eligible patch
              set parent-patch current-patch
              let dist distance source-patch
              set g ([g] of parent-patch)

              ; Consider path difficulty only for patches in sight radius.
              if(dist < sight_radius)
              [
                 if debug [set pcolor cyan]
                 set g (g + difficulty)
              ]

              set h distance destination-patch
              set f (g + h)
            ]
          ]
        ]
      ]
    ]
    [
      ; The only time we get here is when we are on the destination patch, since we have no impassable obstacles.
      ; In this case, do nothing.
      report []
    ]
  ]

  ; if a path is found (search completed) add the current patch
  ; (node adjacent to the destination) to the search path.
  set search-path lput current-patch search-path

  ; trace the search path from the current patch
  ; all the way to the source patch using the parent patch
  ; variable which was set during the search for every patch that was explored
  let temp first search-path
  while [ temp != source-patch ]
  [
    set search-path lput [parent-patch] of temp search-path
    set temp [parent-patch] of temp
  ]

  ; add the destination patch to the front of the search path
  set search-path fput destination-patch search-path

  ; reverse the search path so that it starts from a patch adjacent to the
  ; source patch and ends at the destination patch
  set search-path reverse search-path

  ; report the search path
  report search-path
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Map Generation

to makemap
  ;; Create points for Biomes (Voronoi Points
  ;; Voronoi Code Copyright 2006 Uri Wilensky.
  ;; More info: http://ccl.northwestern.edu/netlogo/models/Voronoi
  ask n-of biome_count patches [ sprout-points 1 [ set biome random (length biome-list) ] ]

  ;; First, setup memoization arrays.
  ask patches [
    set msmoothnoise []
    repeat map_noise_octaves [set msmoothnoise lput -1 msmoothnoise]
    set mbasicnoise []
    repeat map_noise_octaves [set mbasicnoise lput -1 mbasicnoise]
  ]

  ask patches [
    set integrity (perlin_noise pxcor pycor) ; Generate the integrity of the patch based on perlin noise.
    set integrity (integrity / max-integrity) ; Make the integrity a float between 0 and 1.
    set initial_integrity integrity
    set diff_mult item 1 (item biome biome-list)
    set biome [biome] of min-one-of points [distance myself] ;; Assign biomes based on voronoi point distance
    set basecolor item 2 (item biome biome-list) ;; Color the biome
  ]

  updatemap
end

; Updates the map colours based on their difficulty.
to updatemap
  ask patches [
    if ticks > last_trampled + terrain_regrowth_delay [
      set integrity integrity + ((1 - integrity) * terrain_regrowth)
      if integrity > initial_integrity [ set integrity initial_integrity ]
    ]
    let addedColor ((1 - integrity) * 6 + 1) ; Max integrity = black.
    set pcolor basecolor + addedColor
    set difficulty diff_mult * integrity
    set f 0
    set g 0
    set h 0
    set parent-patch nobody
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Code for Perlin Noise Generation
; http://freespace.virgin.net/hugo.elias/models/m_perlin.htm

; Generates the integrity values of all patches.
; Uses perlin noise to generate.
; Called for each patch.
to-report perlin_noise [x y]
  let total 0
  let i 0

  repeat map_noise_octaves [
    let freq 2 ^ i
    let amp map_noise_persistence ^ i

    set total total + (interpolated_noise x y freq i) * amp
    set i (i + 1)
  ]

  ; Keep track of what our maximum integrity is.
  if total > max-integrity [ set max-integrity total ]

  report total
end

; Linearly Interpolates between a0 and a1 with weight w
to-report lerp [a0 a1 w]
  report ( 1.0 - w ) * a0 + w * a1;
end

; A basic noise generation function. This could be better
; But netlogo doesn't seem to have native bitwise operators.
; Generates values in the range (0.0, 1.0)
to-report basic_noise [x y freq i]
  if patch x y = nobody
  [
    let n (x * freq * 7 + y * freq * 57) + seed
    random-seed n
    report random-float 1
  ]

  let memnoise [ item i mbasicnoise ] of patch x y
  ifelse memnoise != -1 [ report memnoise ]
  [
    let n (x * freq * 7 + y * freq * 57) + seed
    random-seed n
    let total random-float 1
    ask patch x y [ set mbasicnoise replace-item i mbasicnoise total]
    report total
  ]
end

; Smoothed noise. Used pre-interpolation
to-report smooth_noise [x y freq i]
  if patch x y = nobody [report 0.5] ; Don't bother computing noise for out of bounds patches.
  let memnoise [ item i msmoothnoise ] of patch x y

  ifelse memnoise != -1 [ report memnoise ]
  [
    let corners ( basic_noise (x - 1) (y - 1) freq i + basic_noise(x + 1) (y - 1) freq i + basic_noise(x - 1) (y + 1) freq i + basic_noise(x + 1) (y + 1) freq i ) / 16
    let sides  ( basic_noise (x - 1) (y) freq i  + basic_noise (x + 1) (y) freq i + basic_noise (x) (y - 1) freq i + basic_noise (x) (y + 1) freq i ) /  8
    let center  (basic_noise x y freq i) / 4
    let total corners + sides + center
    ask patch x y [ set msmoothnoise replace-item i msmoothnoise total]
    report total
  ]
end

to-report interpolated_noise [x y freq i]
  let v1 (smooth_noise x y freq i)
  let v2 (smooth_noise (x + 1) y freq i)
  let v3 (smooth_noise x (y + 1) freq i)
  let v4 (smooth_noise (x + 1) (y + 1) freq i)

  let i1 lerp v1 v2 0.5
  let i2 lerp v3 v4 0.5

  report lerp i1 i2 0.5
end
@#$#@#$#@
GRAPHICS-WINDOW
236
10
891
686
64
64
5.0
1
10
1
1
1
0
0
0
1
-64
64
-64
64
1
1
1
ticks
30.0

BUTTON
96
16
170
49
Setup
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
20
16
87
49
Go
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
12
70
223
103
biome_count
biome_count
1
40
31
1
1
NIL
HORIZONTAL

INPUTBOX
20
202
181
262
map_seed
-1
1
0
Number

SLIDER
12
110
223
143
map_noise_octaves
map_noise_octaves
1
10
1
1
1
NIL
HORIZONTAL

SLIDER
12
153
223
186
map_noise_persistence
map_noise_persistence
0
1
0.2
0.1
1
NIL
HORIZONTAL

TEXTBOX
24
268
203
286
Enter -1 for a random seed
11
0.0
1

SLIDER
12
288
224
321
max_turtles
max_turtles
0
100
70
1
1
NIL
HORIZONTAL

SLIDER
11
324
223
357
spawn_frequency
spawn_frequency
0
100
20
1
1
ticks
HORIZONTAL

TEXTBOX
902
17
1052
35
Green - Forest
12
55.0
1

TEXTBOX
903
34
1053
52
Brown - Rocky\n
12
35.0
1

TEXTBOX
903
51
1053
69
Gray - Snow
12
5.0
1

TEXTBOX
904
68
1054
86
Sky Blue - Ferns
12
95.0
1

TEXTBOX
903
88
1053
106
Violet - Jungle
12
115.0
1

TEXTBOX
904
105
1054
123
Magenta - Swamp
12
125.0
1

SWITCH
13
562
124
595
debug
debug
1
1
-1000

SLIDER
11
364
223
397
sight_radius
sight_radius
1
128
30
1
1
NIL
HORIZONTAL

SLIDER
12
440
224
473
max_backtrack
max_backtrack
0
20
0
1
1
NIL
HORIZONTAL

SLIDER
11
401
224
434
path_recalc_interval
path_recalc_interval
1
128
30
1
1
NIL
HORIZONTAL

MONITOR
902
132
1034
177
Number of Agents
count turtles
17
1
11

PLOT
903
184
1244
400
Travel Distance
Agent
Distance
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "if update_trip_plot = true [\nplot lasttrip\n]"

PLOT
903
409
1244
634
Average Travel Distance
NIL
NIL
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "if update_trip_plot = true [\nplot totaldist / numtrips\n]"

MONITOR
1047
132
1242
177
Number of Completed Trips
numtrips
17
1
11

SLIDER
12
478
224
511
terrain_regrowth
terrain_regrowth
0
0.02
0.001
0.001
1
NIL
HORIZONTAL

SLIDER
12
517
224
550
terrain_regrowth_delay
terrain_regrowth_delay
0
100
50
1
1
NIL
HORIZONTAL

@#$#@#$#@
## WHAT IS IT?

This model simulates the generation of wilderness terrain, as well as the destructive movement of agents through this terrain. This creates pathways through the terrain that other agents are more likely to use.

## HOW IT WORKS

Terrain is generated through a mix of Perlin noise and Voronoi tesselation. The Voronoi tessellation decides the biome of the tile, which dictates the difficulty of moving through the tile, as well as the resistance of the tile to being trampled. The perlin noise governs the integrity of the tile, which is a measure of how trampled the tile has become. For example, a snow tile with high integrity can be thought of as deep snow, where it is very difficult to navigate, but rapidly becomes very easy to navigate as agents pass through it.

Each type of biome has a difficulty multiplier, and the actual difficult of a tile is given by

	difficulty = integrity * diff_mult

Each type of biome also has a deterioration value. When an agent steps on the patch, its integrity is set according to:

	integrity = old_integrity * deterioration_value

When agents are spawned in, they appear on one edge of the simulation, and are given a random destination point on another edge of it. They attempt to traverse the terrain to get to their destination, after which point they leave the simulation.

## HOW TO USE IT

Clicking 'setup' will generate a new map based on the supplied map seed. If the seed is set to -1, then a random seed will be used. biome_count sets the number of points used for Voronoi tessellation, and consequently the number of biomes in the simulation. map_noise_octaves governs the number of successive noise functions that are added together to create the final perlin noise. map_noise_persistence specifics the amplitude of each octave of noise.

Clicking 'go' will run the simulation continually, auto-advancing the simulation clock, spawning agents, and moving them through the simulation.

max_turtles governs the maximum number of agents allowed to move around the simulation at a single time. If this many agents are present in the simulation, no new agents will spawn.

spawn_frequency sets the number of ticks between the creation of each new agent.

sight_radius sets the sight range of the agents. Tiles within the sight range of an agent will have their difficulty taken into account when the agent is computing a new path. Tiles outside this radius will be assumed uniformly easy, as the agent does not make any assumption about them.

path_recalc_interval sets the frequency at which agents re-compute their paths. Generally, it is good to set this less than or equal to their sight radius, otherwise agents will continue blindly through their terrain and take sub-optimal paths.

max_backtrack sets the maximum number of backwards spaces that an agent can consider moving to. If this is set to zero, agents will move towards their destination with every step. Use caution with this, as agents do not have memory of where they have been, and so if a large amount of backtracking is allowed, agents may end up going in circles trying to find a better way around a difficult obstacle.

terrain_regrowth sets the factor by which terrain regrows each tick. Every time step, integrity = integrity * ((1-integrity) * terrain_regrowth)

terrain_regrowth_delay sets the number of ticks that a terrain tile waits after being walked on before it starts to regrow itself.

Finally, the debug flag allows the user to see a visualization of the A* pathfinding algorithm used by the agents. With this flag enabled, every time an agent recomputes their path, the tiles in the agent's sight range are outlined in light blue.


## THINGS TO NOTICE

Under default settings, the simulation matures fairly fast, and creates an interesting network of paths that seems to stabilize after a while. Once the network of paths is mature, agents rarely deviate from them.

Further, on the initial undeveloped map, pathfinding is a very slow and expensive process, even with a small sight radius. As the path network gets more mature, pathfinding becomes very fast. The implication of this is that not only do these path networks reduce the travel time of the agents, they also make travel simpler for agents.

## THINGS TO TRY

Generating the map based on a seed does not affect the randomness of how agents spawn through the terrain. Try running the simulation on the same map multiple times, and seeing how the emergent network of paths differs based on the paths that the early members take.

## EXTENDING THE MODEL

Adding elevation maps to the terrain generation and allowing it to govern the sight radius of agents could produce interesting changes in generated maps.

Currently, agents do not communicate. If agents could communicate using pheromones, in a manner similar to ants, then could this change the development of pathway networks? Or at least quicken the formation of mature networks?

## RELATED MODELS

The Voronoi tessellation model packaged with netlogo, under models/mathematics, is the code that I used for voronoi tessellation in this model.

## CREDITS AND REFERENCES

Perlin Noise Generation adapted from code written by Hugo Elias:
http://freespace.virgin.net/hugo.elias/models/m_perlin.htm

NetLogo A* pathfinding adapted from code written by Meghendra Singh:
http://ccl.northwestern.edu/netlogo/models/community/Astardemo1

Voronoi Tessellation code written by Uri Wilensky, and can be found in the Mathematics/Voronoi model included with netlogo.
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 5.3.1
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
0
@#$#@#$#@
