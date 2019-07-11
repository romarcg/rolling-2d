import sys                

import pyglet
import pymunk
from pymunk.pyglet_util import DrawOptions

import numpy as np
from collections import namedtuple

from scipy.spatial import ConvexHull

from simulator_tools import *
from input_tools import *

Tau = 2*np.pi
Phi = 1.61803398874989


###
#  pyglet setup and calls
###

window = pyglet.window.Window(800, 600, "Window", resizable=False)
pyglet.gl.glClearColor(0.0,43/255.0,54/255.0,1) 
options = DrawOptions()

# setup the reference lines

cross_lines = pyglet.graphics.vertex_list(4,
('v2f', (400, 100, 400, 500, 100, 300, 700, 300)),
('c3B', (147, 161, 161, 147, 161, 161, 147, 161, 161, 147, 161, 161))
)

bounding_hull = []

@window.event
def on_key_press(symbol, modifiers):
    print('A key was pressed', symbol)
    if symbol == 100:
        #rolling_trial.set_vel_roll1((5.0, 0))
        #rolling_trial.set_vel_roll2((-5.0, 0))
        rolling_trial.apply_rolls_velocity(1)
    if symbol == 97:
        #rolling_trial.set_vel_roll1((-5.0, 0))
        #rolling_trial.set_vel_roll2((5.0, 0))
        rolling_trial.apply_rolls_velocity(-1)
    if symbol == 115:
        #rolling_trial.set_vel_roll1((0, 0))
        #rolling_trial.set_vel_roll2((0, 0))
        rolling_trial.apply_rolls_velocity(0)
    
    if symbol == 99:
        #compute convex hull and draw the polygon
        bounding = rolling_trial.workpiece_convex_hull()
        #bounding.insert(0, bounding[0])
        bounding.append(bounding[0])
        #print (bounding)
        pyglet_segments = [p for seg in bounding for p in seg]
        print ("lenght of hull", len(pyglet_segments))
        pyglet_segment_colors = [ [203, 75, 22] for seg in bounding]
        
        global bounding_hull
        bounding_hull = pyglet.graphics.vertex_list(len(bounding), 
            ('v2f', pyglet_segments ), 
            ('c3B', [ v for color in pyglet_segment_colors for v in color] ) 
            )
        global draw_bounding_hull 
        draw_bounding_hull  = True
    
@window.event
def on_draw():
    window.clear()
    cross_lines.draw(pyglet.gl.GL_LINES)    

    #space.debug_draw(options)    
    rolling_trial.pyglet_draw(options)

    if draw_bounding_hull:
        pyglet.gl.glLineWidth(2)
        bounding_hull.draw(pyglet.gl.GL_LINE_STRIP)
        pyglet.gl.glLineWidth(1)

def update(dt):
    #space.step(dt)
    rolling_trial.step(dt)
    #print_mass_moment(body)

###
###


#rolling_trial = Rolling( Workpiece(100.0, 400.0, 300.0), Roll(100.0, 190.0, 300.0, 25.0, 1.0), Roll(100.0, 610.0, 300.0, -25.0, 1.0))

# examples of horizontal cylinders

cyl1 = Cylinder(280, 300, 
                [ [0, 200], [-10, 180], [10, 100], [-50, 0], [10, -100], [-10, -180], [0, -200] ], 
                (5, 0), (0, 0))
cyl2 = Cylinder(520, 300, 
                [ [0, 200], [10, 180], [-10, 100], [0, 0], [-10, -100], [10, -180], [0, -200] ], 
                (-5, 0), (0, 0))

# examples of vertical cylinders

cyl1 = None #Cylinder(280, 300, 
          #      [ [0, 200], [-10, 180], [10, 100], [-50, 0], [10, -100], [-10, -180], [0, -200] ], 
          #      (5, 0), (0, 0))
cyl2 = None #Cylinder(520, 300, 
          #      [ [0, 200], [10, 180], [-10, 100], [0, 0], [-10, -100], [10, -180], [0, -200] ], 
          #      (-5, 0), (0, 0))


#rolling_trial = Rolling( Workpiece(100.0, 400.0, 300.0), cyl1, cyl2 )

draw_bounding_hull = False

if __name__ == "__main__":
    
    wp_radius = 300
    wp_center = (100.0, 400.0) # only for simulator frame (transform when input and output ing the data)
    # Iteration over all arguments:
    print ("Parameters: ", len(sys.argv))
    if len(sys.argv) <= 1:
        cyl1 = Cylinder(280, 300, 
                        [ [0, 200], [-10, 180], [10, 100], [-50, 0], [10, -100], [-10, -180], [0, -200] ], 
                        (5, 0), (0, 0))
        cyl2 = Cylinder(520, 300, 
                        [ [0, 200], [10, 180], [-10, 100], [0, 0], [-10, -100], [10, -180], [0, -200] ], 
                        (-5, 0), (0, 0))
    else:
        file_name = sys.argv[1]        
        data_path = "some_data/"
        outlet, cylinder_imgs, cylinder_contours = convert_mat_to_contours(data_path+file_name, cylinders_indx=[1,3])
        print (len(cylinder_imgs))
        cyl1 = Cylinder(wp_center[0], wp_center[1], 
                        (cylinder_contours[0][0][:,0]*0.1).tolist(), 
                        (0, -5), (0, 0))
        cyl2 = Cylinder(wp_center[0], wp_center[1], 
                        (cylinder_contours[1][0][:,0]*0.1).tolist(), 
                        (0, 5), (0, 0))
        #quit()

        
    rolling_trial = Rolling( Workpiece(wp_center[0], wp_center[1], wp_radius), cyl1, cyl2 )
            
    update_rate = 1.0/100
    pyglet.clock.schedule_interval(update, update_rate)
    pyglet.app.run()
    
