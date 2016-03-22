; global variables used
globals 
[  
  open ; the open list of patches
  closed ; the closed list of patches
  optimal-path ; the optimal path, list of patches from source to destination
]

; patch variables used
patches-own 
[ 
  parent-patch ; patch's predecessor
  f ; the value of knowledge plus heuristic cost function f()
  g ; the value of knowledge cost function g()
  h ; the value of heuristic cost function h()
]

; turtle variables used
turtles-own
[
  path ; the optimal path from source to destination
  current-path ; part of the path that is left to be traversed 
]

; setup the world 
to Setup
  clear-all ;; clear everything (the view and all the variables)
  create-source-and-destination
end

; create the source and destination at two random locations on the view
to create-source-and-destination
  ask one-of patches with [pcolor = black]
  [ 
    set pcolor blue 
    set plabel "source"
    sprout 1 
    [ 
      set color red 
      pd
    ] 
  ]
  ask one-of patches with [pcolor = black]
  [ 
    set pcolor green 
    set plabel "destination"
  ]
end

; draw the selected maze elements on the view
to draw
  if mouse-inside?
    [
      ask patch mouse-xcor mouse-ycor
      [
        sprout 1
        [
          set shape "square"
          die
        ]        
      ]
      
      ;draw obstacles      
      if Select-element = "obstacles"
      [      
        if mouse-down?
        [         
          if [pcolor] of patch mouse-xcor mouse-ycor = black or [pcolor] of patch mouse-xcor mouse-ycor = brown or [pcolor] of patch mouse-xcor mouse-ycor = yellow
          [
            ask patch mouse-xcor mouse-ycor 
            [
              set pcolor white
            ]
          ]      
        ]
      ]
      
      ;erase obstacles      
      if Select-element = "erase obstacles"
      [
        if mouse-down?
        [ 
          if [pcolor] of patch mouse-xcor mouse-ycor = white
          [
            ask patch mouse-xcor mouse-ycor 
            [
              set pcolor black
            ]
          ]      
        ]
      ]
      
      ;draw source patch     
      if Select-element = "source"
      [   
        if mouse-down?
        [ 
          let m-xcor mouse-xcor
          let m-ycor mouse-ycor
          if [plabel] of patch m-xcor m-ycor != "destination"
          [
            ask patches with [plabel = "source"]
            [
              set pcolor black
              set plabel ""
            ]
            ask turtles
            [
              die
            ]          
            ask patch m-xcor m-ycor
            [   
              set pcolor blue 
              set plabel "source"
              sprout 1 
              [ 
                set color red 
                pd
              ] 
            ]
          ]
        ]
      ]
      
      ;draw destination patch     
      if Select-element = "destination"
        [      
          if mouse-down?
            [ 
              let m-xcor mouse-xcor
              let m-ycor mouse-ycor
              
              if [plabel] of patch m-xcor m-ycor != "source"
              [
                ask patches with [plabel = "destination"]
                [
                  set pcolor black
                  set plabel ""
                ]              
                ask patch m-xcor m-ycor 
                [   
                  set pcolor green 
                  set plabel "destination"
                ]
              ]
            ]
        ]
    ]
end

; call the path finding procedure, update the turtle (agent) variables, output text box
; and make the agent move to the destination via the path found
to find-shortest-path-to-destination
  reset-ticks
  ask one-of turtles 
  [
    move-to one-of patches with [plabel = "source"]
    set path find-a-path one-of patches with [plabel = "source"] one-of patches with [plabel = "destination"]
    set optimal-path path
    set current-path path
  ]
  output-show (word "Shortest path length : " length optimal-path)
  move
end


; the actual implementation of the A* path finding algorithm
; it takes the source and destination patches as inputs
; and reports the optimal path if one exists between them as output
to-report find-a-path [ source-patch destination-patch] 
  
  ; initialize all variables to default values
  let search-done? false
  let search-path []
  let current-patch 0
  set open []
  set closed []  
  
  ; add source patch in the open list
  set open lput source-patch open
  
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
        [
          set search-done? true
        ]
        [
          ; the neighbors should not be obstacles or already explored patches (part of the closed list)          
          ask neighbors4 with [ pcolor != white and (not member? self closed) and (self != parent-patch) ]     
          [
            ; the neighbors to be explored should also not be the source or 
            ; destination patches or already a part of the open list (unexplored patches list)
            if not member? self open and self != source-patch and self != destination-patch
            [
              set pcolor 45
              
              ; add the eligible patch to the open list
              set open lput self open
              
              ; update the path finding variables of the eligible patch
              set parent-patch current-patch 
              set g [g] of parent-patch  + 1
              set h distance destination-patch
              set f (g + h)
            ]
          ]
        ]
        if self != source-patch
        [
          set pcolor 35
        ]
      ]
    ]
    [
      ; if a path is not found (search is incomplete) and the open list is exhausted 
      ; display a user message and report an empty search path list.
      user-message( "A path from the source to the destination does not exist." )
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
    ask temp
    [
      set pcolor 85
    ]
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

; make the turtle traverse (move through) the path all the way to the destination patch
to move
  ask one-of turtles 
  [
    while [length current-path != 0]
    [
      go-to-next-patch-in-current-path
      pd
      wait 0.05
    ]
    if length current-path = 0
    [
      pu
    ]
  ]   
end

to go-to-next-patch-in-current-path  
  face first current-path
  repeat 10
  [
    fd 0.1
  ]
  move-to first current-path
  if [plabel] of patch-here != "source" and  [plabel] of patch-here != "destination"
  [
    ask patch-here
    [
      set pcolor black
    ]
  ]
  set current-path remove-item 0 current-path
end

; clear the view of everything but the source and destination patches 
to clear-view
  cd
  ask patches with[ plabel != "source" and plabel != "destination" ]
  [
    set pcolor black
  ]
end

; load a maze from the file system
to load-maze [ maze ]  
  if maze != false
  [
    ifelse (item (length maze - 1) maze = "g" and item (length maze - 2) maze = "n" and item (length maze - 3) maze = "p" and item (length maze - 4) maze = ".")
    [
      save-maze "temp.png"
      clear-all
      import-pcolors maze  
      ifelse count patches with [pcolor = blue] = 1 and count patches with [pcolor = green] = 1
      [
        ask patches 
        [
          set plabel ""
        ]
        ask turtles
        [
          die
        ]    
        ask one-of patches with [pcolor = blue]
        [
          set plabel "source"
          sprout 1 
          [ 
            set color red 
            pd
          ] 
        ] 
        ask one-of patches with [pcolor = green]
        [
          set plabel "destination"
        ] 
      ]
      [
        clear-all
        user-message "The selected image is not a valid maze."
        load-maze "temp.png"
        ;;clear-view
      ]
    ]
    [
      user-message "The selected file is not a valid image."
    ]
  ]
end

; save a maze as a PNG image into the system 
to save-maze [filename]
  if any? patches with [pcolor != black]
  [
    clear-unwanted-elements
    export-view (filename)
    restore-labels
  ]
end

; clear the view of everything but the obstacles, source and destination patches
; so that the view can be saved as a PNG image
to clear-unwanted-elements
  if any? patches with [pcolor = brown or pcolor = yellow  ]
  [
    ask patches with [pcolor = brown or pcolor = yellow  ]
    [
      set pcolor black
    ]
  ]
  if any? patches with [pcolor = blue]
  [
    ask one-of patches with [pcolor = blue]
    [
      set plabel ""
    ] 
  ]
  if any? patches with [pcolor = green]
  [
    ask one-of patches with [pcolor = green]
    [
      set plabel ""
    ] 
  ]
  clear-drawing
  ask turtles
  [
    die
  ]
end

; re-label the source and destination patches ones
; the maze image file has been saved
to restore-labels
  ask one-of patches with [pcolor = blue]
  [
    set plabel "source"
    sprout 1 
    [ 
      set color red 
      pd
    ] 
  ] 
  ask one-of patches with [pcolor = green]
  [
    set plabel "destination"
  ] 
end
@#$#@#$#@
GRAPHICS-WINDOW
190
39
651
521
10
10
21.5
1
10
1
1
1
0
0
0
1
-10
10
-10
10
0
0
1
ticks

BUTTON
6
39
181
76
NIL
Setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

BUTTON
8
160
181
199
Find an optimal path
find-shortest-path-to-destination
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

TEXTBOX
662
149
886
177
patch-color = yellow : patch is in open list
11
44.0
0

TEXTBOX
658
184
808
202
NIL
11
0.0
1

TEXTBOX
662
168
890
196
patch-color = brown : patch is in closed list
11
34.0
1

TEXTBOX
663
188
876
216
patch-color = cyan : patch is in search path
11
84.0
1

TEXTBOX
660
90
913
118
patch-color = green : patch is the destination patch
11
54.0
1

TEXTBOX
660
70
884
98
patch-color = blue : patch is the source patch
11
104.0
1

TEXTBOX
661
110
898
138
patch-color = black : patch is not yet explored
11
0.0
1

BUTTON
8
125
181
158
Draw elements
draw
T
1
T
OBSERVER
NIL
NIL
NIL
NIL

CHOOSER
7
78
181
123
Select-element
Select-element
"source" "destination" "obstacles" "erase obstacles"
3

TEXTBOX
661
130
897
158
patch-color = white : patch is an obstacle
11
124.0
1

BUTTON
8
201
181
234
Clear view
clear-view
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

TEXTBOX
193
10
967
35
Implementation of A* path finding algorithm in NetLogo using exact heuristic function.
16
0.0
1

OUTPUT
660
208
960
249
12

BUTTON
9
271
181
304
Load a maze
load-maze user-file
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

BUTTON
9
236
181
269
Save this maze
save-maze word user-new-file \".png\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

@#$#@#$#@
WHAT IS IT?
-----------
This model simulates an implementation of the A* path finding algorithm, for finding a path from the source patch to the destination patch. The heuristic function used (h(n)) always gives the exact distance from an intermediate patch to the destination patch by employing the NetLogo primitive "distance". This exact heuristic replaces the admissible heuristic in the standard A* which drives path finding exactly in the direction of the destination. This takes place by choosing the next patches (nodes) for exploration which are not yet explored and have the least cost i.e. the smallest distance to the destination as compared to other patches that have not yet been explored. This enables the algorithm to always find an optimal path (least cost path) to the destination, if one exists.  

THE INTERFACE
-------------
The elements of the user interface from top to bottom along with their functions are as follows:

-> Setup (button): Clicking this button resets the simulation. This clears the view and places the source and destination patches at two random locations in the view.

-> Select-element (chooser): This is used to select a particular maze element that can then be drawn on to the view of the model using the Draw elements button and the mouse. The different maze elements are:
	+> source: Select this to place the source patch at a different location in the 
		   view.

	+> destination: Select this to place the destination patch at a different
			location in the view.

	+> obstacles: Select this to place obstacles in the view. This can be used to 
		      design a maze in the view, which can then be navigated using A* to
		      find a path between the source patch and the destination patch. 

	+> erase obstacles: Select this to erase existing obstacles in the view. This can 			    be used to remove an unneeded obstacle from the view, and can 			    also be used in designing a maze in the view. 

-> Draw elements (button): This is used to draw the maze element that is selected in the Select-element chooser on to the view. This can be done by clicking (activating) this button once and then using the mouse to draw the element onto the view. Once the elements have been drawn this button can again be clicked (deactivated) to stop drawing the maze elements on to the view.

-> Find an optimal path (button): Clicking this button will execute the A* path finding algorithm, for finding a path between the source patch and the destination patch. This will result in drawing the shortest path between the source and the destination, if one exists or notifying that such a path does not exist.

-> Clear View (button): Clicking this button clears the view but keeps the source and destination patches at their original locations.

-> Save this maze (button): Clicking this button saves the maze as a ".png" (Portable Network Graphics) image file at a user defined location in the file system.

-> Load a maze (button): Clicking this button loads a maze (Portable Network Graphics) image file from the user defined location in the file system to the view, with the precondition that it should be a valid maze. An image is a valid maze if it has exactly one source patch and destination patch.

-> The notes to the right of the view define what the different colors of the patches mean. For example: White colored patches are obstacles, the blue and green patches are the source and destination respectively, etc.

-> The output text box: Displays the "shortest path length" after clicking the find an optimal path in terms of number of patches that had to be covered to reach the destination via the optimal path. 

HOW IT WORKS
------------
The maze elements are drawn on the view by placing them at the X and Y coordinates of the mouse pointer once it is clicked and is inside the view. 

The Path finding uses the A* algorithm to calculate the shortest path between the source and the destination patch, by avoiding obstacles, as a list of patches and storing it as a variable named "path" with the path finding agent (turtle 0). Once the path is found it is traced by the agent to reach to the destination patch. As A* path finding is used the concept of "open" and "closed" lists are used, these lists contain unexplored and explored patches (nodes) respectively. 

With each step a patch is removed ("explored") from the open list (unexplored patches) and placed in the closed list (explored patches) with a precondition that the value of its knowledge plus heuristic cost function f(), is minimum. After this the parent patch (predecessor-node), g() (knowledge cost function), h() (heuristic cost function) and f() = g() + h() knowledge plus heuristic cost function) values of the removed patch's neighbors are updated accordingly, and these neighboring patches are added to the open list, if they are not already a part of the open list.  

Once the destination patch is reached (a node neighboring to the destination patch is explored) the search process ends. Search path is calculated by traversing through the parent patch of the neighbor of the destination which triggered the end of the search, all the way till the source patch. This path forms a list of patches, which is stored in the path variable of the agent.

If a path does not exist from the source to the destination, ultimately all patches (reachable from the source) end up in the explored (close list) and the open list becomes empty. In which case we conclude that a path from the source to the destination does not exist.

The save maze operation saves a copy of the view as a ".png" image to the filesystem and load maze operation loads (imports) and existing maze ".png" image on the file system into the view.

HOW TO USE IT
-------------
Press the setup button, choose elements from the Select-element chooser, click on the Draw elements button and design your own maze on the view using your mouse and clicking wherever you wish to place the selected element.

Click on the "Find an optimal path" button to compute the shortest path from the source patch to the destination patch through your designed maze, once this is done the agent will navigate through this path from the source patch to the destination patch.

Use "Save this maze" and "Load a maze" to export and import mazes to and from the file system respectively.


WHAT IS ITS PURPOSE?
--------------------
The purpose of this model is to show how the A* path finding algorithm works when we can accurately guide the search towards the goal by using an exact heuristic function, instead of an approximate (admissible) heuristic function used in the standard A* search specification. 

THINGS TO TRY
-------------
Design different mazes and test the path finding on them. 
Change the locations of the source and destination patches for a maze and see its effect on the computed search path.
There can be more than one optimal paths for reaching the destination, try making a maze in which this happens and observe the behavior of the algorithm in that case. 

EXTENDING THE MODEL
-------------------
Change/modify the path finding algorithm used (the "find-a-path" procedure) and see how the model behaves while navigating through various mazes. 

NETLOGO FEATURES
----------------
mouse-inside?, mouse-xcor, mouse-ycor, mouse-down? are used to track the mouse pointer motions in the view.

sort-by was used to sort the "open" list of patches by their f() (knowledge plus heuristic cost function) values for selecting the one with the minimum value of f().

neighbors4 was used to access the "Von Neumann" neighbors of the patch being explored.

user-message, user-file, user-new-file were used for displaying user message as a dialog box, and loading and saving maze image files from and to the file system by interacting with the user.

import-pcolors, export-view were the NetLogo primitives used to import and export maze image files to and from the view of the simulation.

CREDITS AND REFERENCES
----------------------
Russell, S. J.; Norvig, P. (2003). Artificial Intelligence: A Modern Approach. Upper Saddle River, N.J.: Prentice Hall. pp. 97–104. ISBN 0-13-790395-2

http://en.wikipedia.org/wiki/A*_search_algorithm
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

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 4.1.3
@#$#@#$#@
setup-corner
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
