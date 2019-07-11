import pyglet
import pymunk
from pymunk.pyglet_util import DrawOptions

import numpy as np
from collections import namedtuple

from scipy.spatial import ConvexHull

Tau = 2*np.pi
Phi = 1.61803398874989

Workpiece = namedtuple ('Workpiece', ['radius', 'x', 'y'])
#Roll = namedtuple ('Roll', ['radius', 'x', 'y', 'move_x', 'speed_x'])

### concave hull
from shapely.ops import cascaded_union, polygonize
import shapely.geometry as geometry
from scipy.spatial import Delaunay
import numpy as np
import math
def alpha_shape(points, alpha):
    """
    Compute the alpha shape (concave hull) of a set
    of points.
    @param points: Iterable container of points.
    @param alpha: alpha value to influence the
        gooeyness of the border. Smaller numbers
        don't fall inward as much as larger numbers.
        Too large, and you lose everything!
    """
    if len(points) < 4:
        # When you have a triangle, there is no sense
        # in computing an alpha shape.
        return geometry.MultiPoint(list(points)).convex_hull
    def add_edge(edges, edge_points, coords, i, j):
        """
        Add a line between the i-th and j-th points,
        if not in the list already
        """
        if (i, j) in edges or (j, i) in edges:
            # already added
            return
        edges.add( (i, j) )
        edge_points.append(coords[ [i, j] ])
    coords = np.array([point.coords[0]
                       for point in points])
    tri = Delaunay(coords)
    edges = set()
    edge_points = []
    # loop over triangles:
    # ia, ib, ic = indices of corner points of the
    # triangle
    for ia, ib, ic in tri.vertices:
        pa = coords[ia]
        pb = coords[ib]
        pc = coords[ic]
        # Lengths of sides of triangle
        a = math.sqrt((pa[0]-pb[0])**2 + (pa[1]-pb[1])**2)
        b = math.sqrt((pb[0]-pc[0])**2 + (pb[1]-pc[1])**2)
        c = math.sqrt((pc[0]-pa[0])**2 + (pc[1]-pa[1])**2)
        # Semiperimeter of triangle
        s = (a + b + c)/2.0
        # Area of triangle by Heron's formula
        area = math.sqrt(s*(s-a)*(s-b)*(s-c))
        circum_r = a*b*c/(4.0*area)
        # Here's the radius filter.
        #print circum_r
        if circum_r < 1.0/alpha:
            add_edge(edges, edge_points, coords, ia, ib)
            add_edge(edges, edge_points, coords, ib, ic)
            add_edge(edges, edge_points, coords, ic, ia)
    m = geometry.MultiLineString(edge_points)
    triangles = list(polygonize(m))
    return cascaded_union(triangles), edge_points
###

points = [[-1, 1], [0, 0], [1, 1], [1,-1], [-1,-1]]
tmp_points = geometry.MultiPoint(list(points))

concave_hull, edge_points = alpha_shape( tmp_points, 0.5)

print (">> ", concave_hull, edge_points, np.array(edge_points)[:,0])



class Cylinder:
    def __init__(self, x, y, segments = [[-1, 0], [0, 1], [1, 0]], velocity = (0, 0), delta_pos = (0, 0)):
        # center of cilynder
        self.x = x
        self.y = y
        self.velocity = velocity
        # defined by vertices in relative pose (center)
        self.segments = segments
        # bodies an shapes for pymunk
        self.bodies_segments = []
        self.shapes_segments = []
        
        self.segment_mass = 0.1
        self.segment_thickness = 3
        self.create_segments()
    
    def create_segments(self):
        segment_thickness = 5.0
        center = [self.x, self.y]
        for i in range(len(self.segments)-1):
            # radius is the thickness of the segment
            sa = (np.array(center) + np.array(self.segments[i])).tolist()
            sb = (np.array(center) + np.array(self.segments[i+1])).tolist()
            moment_piece = pymunk.moment_for_segment(mass=self.segment_mass, a=sa, b=sb, radius=self.segment_thickness)
            body = pymunk.Body(mass=self.segment_mass, moment=moment_piece, body_type=pymunk.Body.KINEMATIC)
            piece = pymunk.Segment(body, sa, sb, radius=self.segment_thickness)
            piece.color = (133, 153, 0)
            #body.position = x, y
            self.bodies_segments.append(body)
            self.shapes_segments.append(piece)
     
    def apply_velocity(self, dir = 1):
        for seg in self.bodies_segments:
            seg.velocity = (dir*self.velocity[0], dir*self.velocity[1])
            


class Rolling:
    def __init__(self, wp = Workpiece(1.0, 0.0, 0.0), cylinder1 = Cylinder(5, 0, [ [0, 1], [0, 0], [0, 1] ], (1, 0), (0, 0)), cylinder2 = Cylinder(5, 0, [ [0, 1], [0, 0], [0, 1] ], (-1, 0), (0, 0)) ):
        self.scale = 1 # scaling shape of everything (while I find a zoming option)
        print ("Workpiece", wp)
        self.workpiece = Workpiece (wp.radius * self.scale, wp.x, wp.y)
        self.workpiece_bodies = []
        self.workpiece_shapes = []
        self.mass_particle_ws = 0.5
        self.radius_particle_ws = 3.0
        
        #self.roll1 = Roll(roll1.radius, roll1.x, roll1.y, roll1.move_x, roll1.speed_x)
        #self.roll2 = Roll(roll2.radius, roll2.x, roll2.y, roll2.move_x, roll2.speed_x)
        self.cylinder1 = cylinder1
        self.cylinder2 = cylinder2
        
        self.body_roll1 = None
        self.shape_roll1 = None
        self.body_roll2 = None
        self.shape_roll2 = None
        
        self.setup_space()
        
        self.create_worpiece()
        self.attach_workpiece()
        
        self.attach_cylinders()
        #self.create_rolls()
        #self.attach_rolls()
        
    def setup_space(self):
        self.space = pymunk.Space()
        # these values can be set individually to each body
        self.space.gravity = 0, 0
        self.space.damping = 0.0
        
    def step(self, dt):
        self.space.step(dt)
    
    def add_to_space(self, element):
        self.space.add(element)

    def pyglet_draw(self, options):
        self.space.debug_draw(options)
    
    def generate_particle_ws(self, x, y):        
        moment_piece = pymunk.moment_for_circle(mass=self.mass_particle_ws, inner_radius=0, outer_radius=self.radius_particle_ws)        
        body = pymunk.Body(mass=self.mass_particle_ws, moment=moment_piece, body_type=pymunk.Body.DYNAMIC)
        piece = pymunk.Circle(body, radius=self.radius_particle_ws)
        piece.color = (38, 139, 210, 100)
        piece.elasticity = 0.0
        #piece.friction = 0.0
        body.position = x, y
        
        return body, piece

    def create_worpiece(self):
        body, shape = self.generate_particle_ws(self.workpiece.x, self.workpiece.y)
        self.workpiece_bodies.append(body)
        self.workpiece_shapes.append(shape)
        dth = Tau * 1/Phi
        #for th in np.arange(0, Tau, dth):
        keep_going = True
        th = 0.0
        r = 2*self.radius_particle_ws #+ (0.1*self.radius_particle_ws)
        temp_th = 0.0
        
        while (keep_going):
            if r <= self.workpiece.radius:
                body, shape = self.generate_particle_ws(self.workpiece.x + r*np.cos(th), self.workpiece.y + r*np.sin(th))
                #print ("p", (self.workpiece.x + r*np.cos(th), self.workpiece.y + r*np.sin(th)))
                self.workpiece_bodies.append(body)
                self.workpiece_shapes.append(shape)
                th += dth
                temp_th += dth
                if temp_th >= Tau:
                    temp_th = 0.0
                    r += self.radius_particle_ws/15
            else:
                keep_going = False
    
    def attach_workpiece(self):
        for i, e in enumerate(self.workpiece_bodies):
            self.add_to_space(self.workpiece_bodies[i])
            self.add_to_space(self.workpiece_shapes[i])

    def attach_cylinders(self):
        self.add_to_space(self.cylinder1.bodies_segments)
        self.add_to_space(self.cylinder1.shapes_segments)
        self.add_to_space(self.cylinder2.bodies_segments)
        self.add_to_space(self.cylinder2.shapes_segments)
        
    def apply_rolls_velocity(self, dir = 1):
        self.cylinder1.apply_velocity(dir)
        self.cylinder2.apply_velocity(dir)
        
    def workpiece_convex_hull(self):
        points = [(body.position.x, body.position.y ) for body in self.workpiece_bodies]
        points = np.array(points)
        vertices = []
        
        '''
        hull = ConvexHull(points, qhull_options='')
        self.current_hull = hull
        vertices = [ [points[v,0], points[v,1] ]  for v in hull.vertices]
        '''
        
        # concave hull and its edges
        tmp_points = geometry.MultiPoint(list(points))
        concave_hull, edge_points = alpha_shape( tmp_points, 0.1)
        
        #print (">> ", concave_hull, edge_points, np.array(edge_points)[:,0])
        exterior_vertices = list(concave_hull.exterior.coords)
        print ("concave hull ext vertices", len (exterior_vertices))
        if len(exterior_vertices)>0:
            #print ("list exterior", exterior_vertices)
            vertices = exterior_vertices
        
        return vertices

    '''
    # cyclinders as circles
    def generate_roll(self, roll, mass=1.0):
        moment_piece = pymunk.moment_for_circle(mass=mass, inner_radius=0, outer_radius=roll.radius)        
        body = pymunk.Body(mass=mass, moment=moment_piece, body_type=pymunk.Body.KINEMATIC)
        piece = pymunk.Circle(body, radius=roll.radius)
        piece.color = (100, 100, 255)
        body.position = roll.x, roll.y
        return body, piece

    def create_rolls(self):
        self.body_roll1, self.shape_roll1 = self.generate_roll(self.roll1, mass=1.0)
        self.body_roll2, self.shape_roll2 = self.generate_roll(self.roll2, mass=1.0)
        
    def attach_rolls(self):
        self.add_to_space(self.body_roll1)
        self.add_to_space(self.shape_roll1)
        self.add_to_space(self.body_roll2)
        self.add_to_space(self.shape_roll2)
        
    def set_vel_roll1(self, vel=(0, 0)):
        self.body_roll1.velocity = vel
     
    def set_vel_roll2(self, vel=(0, 0)):
        self.body_roll2.velocity = vel
    '''

