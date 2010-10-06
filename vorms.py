#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 2010-06-01
@author: Marek "Marek" Rogalski
TODO: kolizje na linie
TODO: spadające skrzynki
TODO: dźwięk
TODO: tryby gry
'''

from math import sin, cos, atan2, pi, sqrt, pow, ceil, fmod, fabs
import gtk.gdk
import cairo, random
import pygame.mixer
from Box2D import *
from gtk.gtkgl.apputils import *

pygame.mixer.init()

clock = 0

class Camera:
    def __init__(self):
        self.eye_position = [0,0,3]
        self.target_position = [0,0,0]
        
    def set(self):
        gluLookAt(self.eye_position[0], self.eye_position[1], self.eye_position[2],
                  self.target_position[0], self.target_position[1], self.target_position[2],
                  0,1,0)
        
    def step(self, time=0.033):
        vorm = self.vorm
        self.eye_position[0] += (vorm.body.position.x - self.eye_position[0])*0.1
        self.eye_position[1] += (vorm.body.position.y - self.eye_position[1])*0.1
        self.eye_position[2] = 3
        target = [vorm.body.position.x + cos(vorm.alpha), vorm.body.position.y + sin(vorm.alpha), 0]
        for i in range(3):
            self.target_position[i] += (target[i] - self.target_position[i])*0.1
        
    
class TextureManager:
    def __init__(self):
        self.surface = cairo.ImageSurface (cairo.FORMAT_ARGB32, 256, 256)
        self.ctx = cairo.Context (self.surface)
        self.ctx.scale (256, 256)
        
    def clear_ctx(self):
        self.ctx.set_source_rgba(0,0,0,0)
        self.ctx.rectangle(0,0,1,1)
        self.ctx.set_operator(cairo.OPERATOR_SOURCE)
        self.ctx.fill()
        self.ctx.set_operator(cairo.OPERATOR_OVER)
        return self.ctx
        
    def render_cairo(self):
        return self.load_texture(str(self.surface.get_data()), 256, 256)
        
    def free_texture(self, texture):
        glDeleteTextures(texture)
        
    def load_texture_from_file(self, fileName):
        image = gtk.gdk.pixbuf_new_from_file(fileName)
        ix = image.get_width()
        iy = image.get_height()
        pixels = image.get_pixels()
        return self.load_texture(pixels, ix, iy)
    
    def load_texture(self, pixels, width, height):
        id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, id)
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR)
        gluBuild2DMipmaps(GL_TEXTURE_2D, 4, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels)
        return id

class myContactListener(b2ContactListener):
    def Add(self, point):
        pass
    def Persist(self, point):
        pass
    def Remove(self, point):
        pass
    def Result(self, point):
        for shape in [point.shape1, point.shape2]:
            dict = shape.GetUserData()
            if dict['type'] == 'vorm':
                dict['vorm'].wall_clock = clock
                dict['vorm'].sound_hit.play()

texturer = TextureManager()
worldAABB=b2AABB()
worldAABB.lowerBound.Set(-100, -100)
worldAABB.upperBound.Set(100, 100)
gravity = (0, -10)
doSleep = True
world = b2World(worldAABB, gravity, doSleep)
listener = myContactListener()
world.SetContactListener(listener)

def unp(elem, attr):
    return [float(i) for i in elem.getAttribute(attr).split(',')]

def frange(start, stop, n):
    L = [0.0] * n
    nm1 = n - 1
    nm1inv = 1.0 / nm1
    for i in range(n):
        L[i] = nm1inv * (start*(nm1 - i) + stop*i)
    return L

def arange(start, stop, n):
    start = fmod(start, 2*pi)
    stop = fmod(start, 2*pi)
    if(fabs(start - stop) > pi):
        if start > stop: start -= 2*pi
        else: stop -= 2*pi
    return frange(start, stop, n)

class Map:
    def __init__(self, file):
        self.rects = []
        from xml.dom.minidom import parse
        self.xml = parse(file)

    def setup_environment(self):
        top = self.xml.documentElement
        glClearColor(*unp(top, 'bg'))
        self.lights = self.xml.getElementsByTagName("light")
        if self.lights:
            glEnable(GL_LIGHTING)
            n = GL_LIGHT0
            for light in self.lights:
                glLightfv(n, GL_AMBIENT, unp(light, 'ambient'))
                glLightfv(n, GL_DIFFUSE, unp(light, 'diffuse'))
                glLightfv(n, GL_SPECULAR, unp(light, 'specular'))
                glEnable(n)
                n += 1
        rects = self.xml.getElementsByTagName("rect")
        for node in self.xml.getElementsByTagName("random"):
            n = int(node.getAttribute('number'))
            c = node.getAttribute('color')
            b = unp(node, 'bounds')
            s = unp(node, 'size')
            for i in range(n):
                rect = self.xml.createElement("rect")
                rect.setAttribute('color', c)
                l = b[3]+random.random()*(b[1] - b[3] - s[0])
                t = b[2]+random.random()*(b[0] - b[2] - s[1])
                bounds = str(t)+ ',' + str(l+s[0]) + ',' + str(t-s[1]) + ',' + str(l)
                rect.setAttribute('bounds', bounds)
                rects.append(rect)
        for rect in rects:
            self.register(rect)
        
    def register(self, rect):
        b = unp(rect, 'bounds')
        groundBodyDef = b2BodyDef()
        groundBodyDef.position = ((b[1]+b[3])/2, (b[0]+b[2])/2)
        groundBody = world.CreateBody(groundBodyDef)
        groundShapeDef = b2PolygonDef()
        groundShapeDef.SetAsBox((b[1]-b[3])/2, (b[0]-b[2])/2)
        groundBody.CreateShape(groundShapeDef)
        for shape in groundBody:
            shape.SetUserData({'type': 'ground'})
        self.rects.append(groundBody)
        
    def draw(self):
        global lists
        n = GL_LIGHT0
        for light in self.lights:
            glLightfv(n, GL_POSITION, unp(light, 'position'))
            n += 1
            
        glDisable(GL_TEXTURE_2D)
        glColor4f(1,1,1,1)
        for rect in self.rects:
            glPushMatrix()
            glTranslatef(rect.position.x, rect.position.y, 0)
            obb = rect.GetShapeList()[0].GetOBB()
            glScalef(obb.extents.x, obb.extents.y, 10)
            glTranslatef(0, 0, -1)
            gtk.gdkgl.draw_cube(True,2)
            glPopMatrix()
        glEnable(GL_TEXTURE_2D)

class Vorm:    
    shapeDef = b2CircleDef()
    shapeDef.radius = 0.04
    shapeDef.density = 1
    shapeDef.restitution = 1
    shapeDef.friction = 0
    shapeDef.filter.groupIndex = -8
    
    sound_hook = pygame.mixer.Sound('sounds/hook.wav')
    sound_unhook = pygame.mixer.Sound('sounds/unhook.wav')
    sound_hit = pygame.mixer.Sound('sounds/hit.wav')
    sound_rope_fire = pygame.mixer.Sound('sounds/rope_fire.wav')
    sound_rope_unfire = pygame.mixer.Sound('sounds/rope_unfire.wav')
            
    def __init__(self, map, position):
        self.map = map
        self.alpha = 0
        
        bodyDef = b2BodyDef()
        bodyDef.position = (position[0], position[1])
        bodyDef.linearDamping = 0.5
        self.body = world.CreateBody(bodyDef)
        shape = self.body.CreateShape(Vorm.shapeDef)
        shape.SetUserData({'type': 'vorm', 'vorm': self})
        self.body.SetMassFromShapes()
        self.body.fixedRotation = True
        
        self.rope_keys = [0,0]
        self.rope_length = 0
        self.rope_alpha = 0
        self.rope_state = 'hidden'
        self.rope_segments = []
        self.rope_joint = None
        self.rope_hook = None
        self.target = b2Vec2(50,50)
        
        self.wall_clock = 0
        self.jump_clock = 0
        self.move = 'jump'
        self.weapon = 'bomb'
        
        self.unhook_rope()
        
    def fix_rope_alpha(self):
        arm = self.rope_hook - self.body.position
        return atan2(arm.y, arm.x)
        
    def check_rope(self):
        self.rope_alpha = self.fix_rope_alpha()
        simplified = False
        
        if len(self.rope_segments):
            nalpha = self.rope_alpha
            ndir = b2Cross(-b2Vec2(cos(nalpha),sin(nalpha)), self.body.GetLinearVelocity())
            odir, oalpha, ohook = self.rope_segments[-1]
            
            
            def can_i(dir, n, o):
                n, o = fmod(n,2*pi), fmod(o,2*pi)
                if dir < 0: return oalpha < nalpha < oalpha+pi or oalpha < nalpha+2*pi < oalpha+pi
                else: return nalpha < oalpha < nalpha+pi or nalpha < oalpha+2*pi < nalpha+pi
            
            while ndir * odir < 0 and can_i(odir, nalpha, oalpha):
                simplified = True 
                self.rope_segments.pop()
                world.DestroyJoint(self.rope_joint)
                self.hook_joint(ohook)
                if len(self.rope_segments): odir, oalpha, ohook = self.rope_segments[-1]
                else: break
  
        # add intermediate nodes
        ray = b2Segment()
        delta = b2Vec2(cos(self.rope_alpha), sin(self.rope_alpha))
        ray.p2 = self.rope_hook - delta*0.01
        ray.p1 = self.rope_hook - delta*self.rope_length
        lambda_, normal, shape = world.RaycastOne(ray,False,None)
        fastcheck = shape != None
        
        if fastcheck and not simplified:
            # will improve performance & accuracy if instead of ray queries
            # we used PolygonShape of triangle formed from last and current rope angle
            # used to make queries of rectangles, then its points, and sorting them
            # barycentric 
            alphas = arange(self.rope_last_alpha,self.rope_alpha,200)
            i = 0
            while i < len(alphas):
                alpha = alphas[i]
                i += 1
                ray = b2Segment()
                delta = b2Vec2(cos(alpha), sin(alpha))
                ray.p2 = self.rope_hook - delta*0.01
                ray.p1 = self.rope_hook - delta*self.rope_length
                lambda_, normal, shape = world.RaycastOne(ray,False,None)
                if shape and shape.GetUserData()['type'] == 'ground':
                    hit_point = (1-lambda_)*ray.p1+lambda_*ray.p2
                    mpoint = hit_point
                    mdist = 0.5
                    for vertex in shape.asPolygon().getVertices_b2Vec2():
                        vertex = vertex + shape.GetBody().position
                        if (hit_point - vertex).Length() < mdist: # (self.rope_hook - vertex).Length() > 0.1 and
                            mdist = (hit_point - vertex).Length()
                            mpoint = vertex
                    hit_point = mpoint
                    world.DestroyJoint(self.rope_joint)
                    cross = b2Cross(-delta, self.body.GetLinearVelocity())
                    arm = self.rope_hook - hit_point
                    bend_alpha = atan2(arm.y, arm.x)
                    self.rope_segments.append((cross, bend_alpha, self.rope_hook))
                    self.hook_joint(hit_point, shape.GetBody())
                    
                    alphas = arange(bend_alpha,self.rope_alpha,50)
                    i = 0
                    
        self.rope_last_alpha = self.rope_alpha
            
    def hook_joint(self, hit_point, body= None):
        body = body or self.map.rects[0]
        self.rope_hook = hit_point
        self.rope_length = (self.body.position - hit_point).Length()
        jointDef = b2DistanceJointDef()
        jointDef.Initialize(self.body, body, self.body.position, hit_point)
        jointDef.collideConnected = True
        self.rope_joint = world.CreateJoint(jointDef).getAsType()
        self.rope_alpha = self.fix_rope_alpha()
        
    def hook_rope(self, hit_point, body):
        self.rope_state = 'hooked'
        self.move = 'rope'
        for shape in self.body:
            shape.restitution = 1
            shape.friction = 0
        self.body.linearDamping = 0.5
        self.hook_joint(hit_point, body)
        self.sound_hook.play()
        
    def unhook_rope(self):
        del self.rope_segments[:]
        self.rope_state = 'hidden'
        self.move = 'jump'
        for shape in self.body:
            shape.restitution = 0.5
            shape.friction = 1
        self.body.linearDamping = 0.5
        if self.rope_joint: world.DestroyJoint(self.rope_joint)
        self.sound_unhook.play()
        
    def fire_rope(self):
        if self.rope_state == 'hidden':
            self.rope_length = 0.0
            self.rope_alpha = self.alpha
            self.rope_last_alpha = self.alpha
            self.rope_state = 'extending'
            self.sound_rope_fire.play()
        elif self.rope_state == 'extending':
            self.rope_state = 'hidden'
            self.sound_rope_unfire.play()
        elif self.rope_state == 'hooked':
            self.unhook_rope()
        
    def fire_weapon(self):
        pass

    def draw(self, active):
        glDisable(GL_TEXTURE_2D)
        glPushMatrix()
        glColor4f(1,0,0,1)
        glTranslatef(self.body.position.x, self.body.position.y, 0)
        gtk.gdkgl.draw_torus(True,.02,.04,12,12)
        
        if active:
            glPushMatrix()
            glTranslatef(self.target.x,self.target.y,0)
            glColor4f(1,1,1,1)
            gtk.gdkgl.draw_torus(True,.02,.04,12,12)
            glPopMatrix()
            
        if self.rope_state == 'extending':
            glRotatef(self.rope_alpha * 180 / pi, 0, 0, 1)
            glTranslatef(self.rope_length, 0, 0)
            glRotatef(90, 0, 1, 0)
            glColor4f(1,1,1,1)
            glPushMatrix()
            glTranslatef(0, 0, -0.1)
            gtk.gdkgl.draw_cone(True,0.05,0.1,10,10)
            gtk.gdkgl.draw_cone(True,0.05,0,10,10)
            glPopMatrix()
            glColor4f(.8,.4,0,1)
            glTranslatef(0, 0, -0.1)
            glScalef(0.025,0.025,self.rope_length-0.04-0.1)
            glTranslatef(0, 0, -0.5)
            gtk.gdkgl.draw_cube(True,1)
            
        glPopMatrix()
        
        if self.rope_state == 'hooked':
            self.rope_segments.append((None,None,self.rope_hook))
            self.rope_segments.append((None,None,self.body.position))
            first, second = True, False
            for segment in self.rope_segments:
                if first:
                    B, first, second = segment[2], False, True
                    continue
                
                glPushMatrix()
                A, B = B, segment[2]
                delta = A-B
                alpha = atan2(delta.y, delta.x)
                glTranslatef(A.x, A.y, 0)
                glRotatef(alpha * 180 / pi, 0, 0, 1)
                glRotatef(90, 0, 1, 0)
                
                if second:
                    glColor4f(1,1,1,1)
                    glPushMatrix()
                    glTranslatef(0, 0, -0.1)
                    gtk.gdkgl.draw_cone(True,0.05,0.1,10,10)
                    gtk.gdkgl.draw_cone(True,0.05,0,10,10)
                    glPopMatrix()
                
                glColor4f(.8,.4,0,1)
                if second: glTranslatef(0, 0, -0.1)
                length = delta.Length()
                if second: length -= 0.1
                if second and len(self.rope_segments) == 1: length -= 0.04
                glScalef(0.025,0.025,length)
                glTranslatef(0, 0, -0.5)
                gtk.gdkgl.draw_cube(True,1)
                
                glPopMatrix()
                if second: first, second = False, False
            self.rope_segments.pop()
            self.rope_segments.pop()
            
        glEnable(GL_TEXTURE_2D)
        
    def look(self, x, y):
        self.target += b2Vec2(-x,y)/100
        self.alpha = atan2(self.target.y, self.target.x)
    
    def can_jump(self):
        return self.jump_clock < self.wall_clock and self.jump_clock + 1 < clock
        
    def jump(self):
        p, a = 0.02, self.alpha
        self.body.ApplyImpulse((p*cos(a), p*sin(a)), (0,0))
        global clock
        self.jump_clock = clock

    def step(self, time = 0.033):
        
        circle = self.target / max(self.target.Length(),0.01)
        self.target += (circle - self.target)*(circle - self.target).Length()*0.01
        
        self.alpha = fmod(self.alpha, 2*pi)
        if self.rope_state == 'extending':
            speed = 8
            if self.rope_length > speed:
                self.rope_state = 'hidden'
            else: 
                self.rope_length += time*speed*2
                dx = cos(self.rope_alpha)*self.rope_length
                dy = sin(self.rope_alpha)*self.rope_length
                ray = b2Segment()
                ray.p1 = self.body.position
                ray.p2 = (dx, dy)
                ray.p2 += ray.p1
                lambda_, normal, shape = world.RaycastOne(ray,False,None)
                if shape:
                    hit_point = (1-lambda_)*ray.p1+lambda_*ray.p2
                    self.hook_rope(hit_point, shape.GetBody())
        elif self.rope_state == 'hooked':
            dx = self.rope_hook.x - self.body.position.x
            dy = self.rope_hook.y - self.body.position.y
            self.rope_alpha = atan2(dy, dx)
            self.check_rope()
            if self.rope_keys[0] != 0:
                self.body.ApplyForce((self.rope_keys[0]/30.0,0), (0,0))#self.body.position)
            v = self.body.GetLinearVelocity()
            l1 = self.rope_length   
            self.rope_length += self.rope_keys[1] * time * 2
            self.rope_length = max(self.rope_length, 0.1)
            l2 = self.rope_length
            self.body.SetLinearVelocity(v*l1/l2)
            self.body.setPosition((self.rope_hook.x - cos(self.rope_alpha)*self.rope_length,
                                   self.rope_hook.y - sin(self.rope_alpha)*self.rope_length))
            self.rope_joint.length = self.rope_length
                
    def step2(self, time = 0.033):
        if self.rope_state == 'hooked':
            self.check_rope()
            self.rope_length = (self.body.position - self.rope_hook).Length()

class Vorms(EmptyScene):
    def __init__(self):
        GLScene.__init__(self,
                         gtk.gdkgl.MODE_RGBA   |
                         gtk.gdkgl.MODE_DEPTH |
                         gtk.gdkgl.MODE_DOUBLE)
        GLSceneTimeout.__init__(self)
        
        self.keys = set()
        self.camera = Camera()
        self.map = Map('map.xml')
        self.vorms = [Vorm(self.map, [i*1.0, 0.0, 0.0]) for i in range(-1,2)]
        self.game_state = 'loading'
        self.active = -1
        self.switch_vorm()
        self.last_x = 0
        self.last_y = 0
        self.round_clock = 0
        self.round_second_clock = 0
        
    def init(self):
        global lists
        lists = glGenLists(1)
        glNewList(lists, GL_COMPILE)
        glBegin(GL_QUADS)
        glTexCoord2f(0,0)
        glVertex2f(0,1)
        glTexCoord2f(1,0)
        glVertex2f(1,1)
        glTexCoord2f(1,1)
        glVertex2f(1,0)
        glTexCoord2f(0,1)
        glVertex2f(0,0)
        glEnd()
        glEndList()
        
        self.map.setup_environment()
        glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_BLEND)
        glBlendFunc (GL_ONE, GL_ONE)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        glFogi(GL_FOG_MODE, GL_LINEAR)
        glFogfv(GL_FOG_COLOR, [.5,.5,1,1])
        glFogf(GL_FOG_DENSITY, 0.03)
        glHint(GL_FOG_HINT, GL_NICEST)
        glFogf(GL_FOG_START, 5)
        glFogf(GL_FOG_END, 10)
        glEnable(GL_FOG)
        
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        
        ctx = texturer.clear_ctx()
        ctx.set_source_rgba (0, 0, 1, 0.6)
        ctx.arc (0.5, 0.5, 0.3, 0, 2*pi)
        ctx.fill()
        self.texture_dot = texturer.render_cairo()
        self.texture_dot_alpha = 0.0
        
        self.texture_clock = None
        self.render_clock()
        pygame.mixer.Sound('sounds/startup.wav').play()
        
    def render_clock(self):
        if self.texture_clock: texturer.free_texture(self.texture_clock)
        ctx = texturer.clear_ctx()
        
        ctx.select_font_face ("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
        ctx.set_font_size(0.7)
        start = 30
        if self.game_state == 'switch':
            start = 5
        text = str(int(ceil(start - clock + self.round_clock)))
        ext = ctx.text_extents (text)
        ctx.move_to (.5 - (ext[2]/2 + ext[0]),
                     .5 - (ext[3]/2 + ext[1]))
        ctx.text_path (text)
        a = (clock - self.round_clock)/30
        ctx.set_source_rgba (0,1*(1-a),1*a,1)
        ctx.fill_preserve()
        ctx.set_source_rgb (0, 0, 0)
        ctx.set_line_width (0.02)
        ctx.stroke()
        self.texture_clock = texturer.render_cairo()
            
    def draw_environment(self, width, height):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(90.0, (float)(width)/height, 0.1, 1000)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.camera.set()
        c = self.vorms[self.active]
        for vorm in self.vorms:
            vorm.draw(c == vorm)
        self.map.draw()
            
    def draw_gui(self, width, height):
        global lists
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-1, 1, -1, 1, -10, 10)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0,0,1,
                  0,0,0,
                  0,1,0)
        
        glDisable(GL_LIGHTING)
        glBindTexture(GL_TEXTURE_2D, self.texture_clock)
        glPushMatrix()
        glScaled(float(height)/width,1,1)
        #glTranslatef(0,1,0)
        glTranslatef(0,0.5,0)
        if self.game_state == 'play': glTranslatef(0,0.5,0)
        if self.game_state == 'play': glScaled(.3,.3,1)
        glTranslatef(-0.5,-1,0)
        glColor4f(1,1,1,1)
        glCallList(lists)
        glPopMatrix()
        
        glColor4f(1,1,1,self.texture_dot_alpha)
        glBindTexture(GL_TEXTURE_2D, self.texture_dot)
        glScalef(1,float(width)/height,1)
        glTranslatef(1,0,0)
        glScalef(.3,.3,1)
        glTranslatef(-0.5,0,0)
        glScalef(self.texture_dot_alpha,self.texture_dot_alpha,1)
        glTranslatef(-0.5,-0.5,0)
        glCallList(lists)
        glEnable(GL_LIGHTING)
    
    def display(self, width, height):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.draw_environment(width, height)
        self.draw_gui(width, height)
        
    def timeout(self, width, height):
        global clock
        delta = 1.0/60.0
        clock += delta
        
        vorm = self.vorms[self.active]
        vorm.rope_keys[0] = 0
        vorm.rope_keys[1] = 0
        
        if clock - self.round_second_clock > 1:
            self.round_second_clock += 1
            if self.game_state == 'play' and clock - self.round_clock > 30:
                self.round_clock += 30
                self.switch_vorm()
            if self.game_state == 'switch' and clock - self.round_clock > 5:
                self.game_state = 'play'
                self.round_clock += 5
            self.render_clock()
        
        if self.game_state == 'play':
            if 'a' in self.keys: vorm.rope_keys[0] -= 1
            if 'e' in self.keys: vorm.rope_keys[0] += 1
            if 'comma' in self.keys: vorm.rope_keys[1] -= 1
            if 'o' in self.keys:     vorm.rope_keys[1] += 1            
            if 'mouse1' in self.keys and vorm.can_jump(): vorm.jump()
        
        if vorm.can_jump(): self.texture_dot_alpha += (1-self.texture_dot_alpha)*0.2
        else: self.texture_dot_alpha *= 0.8
        
        for vorm in self.vorms: vorm.step(delta)
        
        velocityIterations = 10
        positionIterations = 8
        world.Step(delta, velocityIterations, positionIterations)
        for vorm in self.vorms: vorm.step2(delta)
        
        self.camera.step(delta)
        self.queue_draw()
        
    def switch_vorm(self, mod = 1):
        self.active = (self.active + mod)%len(self.vorms)
        self.camera.vorm = self.vorms[self.active]
        self.game_state = 'switch'
        self.round_clock = clock

    def button_press(self, width, height, event):
        if event.button == 1 and not 'mouse1' in self.keys: self.keys.add('mouse1')

    def motion(self, widget, event):
        self.vorms[self.active].look(self.last_x - event.x, self.last_y - event.y)
        self.last_x = event.x
        self.last_y = event.y

    def button_motion(self, width, height, event):
        self.motion(None, event)

    def button_release(self, width, height, event):
        if event.button == 1: self.keys.remove('mouse1') 

    def key_press(self, width, height, event):
        key = gtk.gdk.keyval_name (event.keyval)
        if not key in self.keys: self.keys.add(key)
        vorm = self.vorms[self.active]
        
        if self.game_state == 'play':
            if key == 'space':    vorm.fire_rope()  
            elif key == 'Return': vorm.fire_weapon()
        if key == 'Tab':    self.switch_vorm()
        elif key == 'Escape': gtk.main_quit()

    def key_release(self, width, height, event):
        key = gtk.gdk.keyval_name (event.keyval)
        self.keys.remove(key) 
        #print 'Released', key
    
    def reshape(self, width, height):
        glViewport(0, 0, width, height)

if __name__ == '__main__':
    glscene = Vorms()
    glapp = GLApplication(glscene, 640, 480, 'Vorms')
    glapp.add_events(gtk.gdk.POINTER_MOTION_MASK)
    glapp.connect("motion-notify-event", glscene.motion)
    glapp.run()
