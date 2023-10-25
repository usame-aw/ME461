#!/usr/bin/env python3

import pygame
import cv2
import time
import numpy as np
from .tangui import *
from .fruit import *

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64MultiArray


GAMEMODE = 0
FULLSCREEN = 0

def make_surface_rgba(array):
    """Returns a surface made from a [w, h, 4] numpy array with per-pixel alpha
    """
    shape = array.shape
    if len(shape) != 3 and shape[2] != 4:
        raise ValueError("Array not RGBA")

    # Create a surface the same width and height as array and with
    # per-pixel alpha.
    surface = pygame.Surface(shape[0:2], pygame.SRCALPHA, 32)

    # Copy the rgb part of array to the new surface.
    pygame.pixelcopy.array_to_surface(surface, array[:,:,0:3])

    # Copy the alpha part of array to the surface using a pixels-alpha
    # view of the surface.
    surface_alpha = np.array(surface.get_view('A'), copy=False)
    surface_alpha[:,:] = array[:,:,3]

    return surface

def render(display, index):
    surface = pygame.display.get_surface()
    if GAMEMODE == 0:
        for obj in menutexts + buttons + selectors:
            obj.draw(surface)
        
    else:
        for obj in texts + progressbars + [particles] + fruits:
            obj.draw(surface)

    pygame.display.flip()

def eventcheck():
    global GAMEMODE, gametime, score
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
        elif event.type == pygame.MOUSEBUTTONDOWN:
            #print(event.button)
            #print(np.array(event.pos)/mouse_pos_scale)
            #Check for left mouse click
            if event.button == 1:
                if GAMEMODE == 0:
                    for button in buttons:
                        if button.checkHover(np.array(event.pos)):
                            #Get which button is pressed
                            result = button.clickFunction()
                            if result == "play":
                                GAMEMODE = int(selectors[0].getValue())
                                gametime = 1800
                                score = 0
                    for selector in selectors:
                        if selector.checkHover(np.array(event.pos)):
                            selector.clickFunction()
    return False

def main(args=None):
    global texts, menutexts, buttons, selectors, progressbars, particles, fruits, resolution, GAMEMODE, gametime, score
    texts = []
    menutexts = []
    buttons = []
    selectors = []
    progressbars = []
    fruits = []
    fruit_choices = []
    particles = Particles()
    score = 0

    rclpy.init(args=args)
    node = Node("fruit_game_cam_handler")
    pub = node.create_publisher(Float64MultiArray, "/fruit_game/finger_pos", 10)
    mp_controller = MP_Controller(GAMEMODE)

    msg = Float64MultiArray()
    msg.data = [0.0, 0.0]
    
    pygame.init()
    pygame.font.init()

    if FULLSCREEN:
        info = pygame.display.Info()
        resolution = (info.current_w, info.current_h)
    else:
        resolution = (640, 360)

    tomato = cv2.imread("assets/tomato.png",cv2.IMREAD_UNCHANGED)
    tomato = cv2.cvtColor(tomato,cv2.COLOR_BGRA2RGBA)
    tomato = cv2.resize(tomato,(np.array(tomato.shape[1::-1])*(resolution[0]/640)).astype(int))
    tomato = make_surface_rgba(tomato.swapaxes(0, 1))
    fruit_choices.append(tomato)

    watermelon = cv2.imread("assets/watermelon.png",cv2.IMREAD_UNCHANGED)
    watermelon = cv2.cvtColor(watermelon,cv2.COLOR_BGRA2RGBA)
    watermelon = cv2.resize(watermelon,(np.array(watermelon.shape[1::-1])*(resolution[0]/(640*4.5))).astype(int))
    watermelon = make_surface_rgba(watermelon.swapaxes(0, 1))
    fruit_choices.append(watermelon)

    apple = cv2.imread("assets/apple.png",cv2.IMREAD_UNCHANGED)
    apple = cv2.cvtColor(apple,cv2.COLOR_BGRA2RGBA)
    apple = cv2.resize(apple,(np.array(apple.shape[1::-1])*(resolution[0]/(640*20))).astype(int))
    apple = make_surface_rgba(apple.swapaxes(0, 1))
    fruit_choices.append(apple)

    pineapple = cv2.imread("assets/pineapple.png",cv2.IMREAD_UNCHANGED)
    pineapple = cv2.cvtColor(pineapple,cv2.COLOR_BGRA2RGBA)
    pineapple = cv2.resize(pineapple,(np.array(pineapple.shape[1::-1])*(resolution[0]/(640*20))).astype(int))
    pineapple = make_surface_rgba(pineapple.swapaxes(0, 1))
    fruit_choices.append(pineapple)

    orange = cv2.imread("assets/orange.png",cv2.IMREAD_UNCHANGED)
    orange = cv2.cvtColor(orange,cv2.COLOR_BGRA2RGBA)
    orange = cv2.resize(orange,(np.array(orange.shape[1::-1])*(resolution[0]/(640*8))).astype(int))
    orange = make_surface_rgba(orange.swapaxes(0, 1))
    fruit_choices.append(orange)

    pear = cv2.imread("assets/pear.png",cv2.IMREAD_UNCHANGED)
    pear = cv2.cvtColor(pear,cv2.COLOR_BGRA2RGBA)
    pear = cv2.resize(pear,(np.array(pear.shape[1::-1])*(resolution[0]/(640*10))).astype(int))
    pear = make_surface_rgba(pear.swapaxes(0, 1))
    fruit_choices.append(pear)

    cap = cv2.VideoCapture(0)

    index = None

    try:
        display = pygame.display.set_mode(
            resolution,
            pygame.HWSURFACE | pygame.DOUBLEBUF | (pygame.FULLSCREEN if FULLSCREEN else 0))

        clock = pygame.time.Clock()
        pygame.display.set_caption('Robot Değilim')
        iconsurf = pygame.image.load("assets/appicon.png").convert_alpha()
        pygame.display.set_icon(iconsurf)
        
        font = "freemono"
        bigfont = pygame.font.SysFont(font, int(resolution[1]*(40/360)))
        midfont = pygame.font.SysFont(font, int(resolution[1]*(26/360)))
        smallfont = pygame.font.SysFont(font, int(resolution[1]*(20/360)))
        bigfont.bold = True
        midfont.bold = True
        smallfont.bold = True

        menutexts.append(Text(resolution[0]*(320/640),resolution[1]*(80/360),"Fruit Game",bigfont))
        menutexts.append(Text(resolution[0]*(320/640),resolution[1]*(240/360),f"Last Score: {score}",midfont))
        menutexts.append(Text(resolution[0]*(320/640),resolution[1]*(320/360),"By Osama Awad, Tan Çağatay Acar and Zülal Uludoğan",smallfont))
        texts.append(Text(resolution[0]*(320/640),resolution[1]*(320/360),f"Score: {score}",bigfont))

        buttons.append(Button("play",resolution[0]*(240/640),resolution[1]*(160/360),width=resolution[0]*(60/640),height=resolution[1]*(30/360),text="Play Game"))

        selectors.append(Selector(resolution[0]*(400/640), resolution[1]*(160/360), width=resolution[0]*(
            40/640), height=resolution[1]*(20/360), choices=(1, 2)))
        
        gameProgressBar = ProgressBar(resolution[0]*(320/640), resolution[1]*(20/360), width=resolution[0]*(300/640), height=resolution[1]*(20/360), max=1800)
        progressbars.append(gameProgressBar)
        
        done = False
        while not done:
            done = eventcheck()

            ret, frame = cap.read()
            frame = cv2.flip(frame, 1)

            surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1)[:,:,::-1])
            display.blit(surface, (0,0))

            if GAMEMODE == 0:
                menutexts[1].setText(f"Last Score: {score}")
            else:
                for i,fruit in enumerate(fruits[::-1]):
                    fruit.update()
                    if fruit.pos[1] > 410:
                        fruits.pop(i)

                for i in range(np.random.poisson(0.01)):
                    fruits.append(Fruit(random.choice(fruit_choices)))

                mp_controller.detect_async(frame, GAMEMODE)

                try:
                    index = mp_controller.get_index_tip_coordinates()
                except:
                    pass
                    
                if index:
                    msg.data[0] = index[0]
                    msg.data[1] = index[1]
                    pub.publish(msg)
                    index = (int(index[0]*resolution[0]),int(index[1]*resolution[1]*4/3))
                    n = len(fruits)-1
                    for i,fruit in enumerate(fruits[::-1]):
                        if fruit.checkHover(index):
                            fruits.pop(n-i)
                            score += 1
                            texts[0].setText(f"Score: {score}")
                if fruits:
                    node.get_logger().info(f"{len(fruits)}")

                particles.set_source_pos(index)

                gametime -= 1
                if gametime == 0:
                    GAMEMODE = 0
                gameProgressBar.setValue(gametime)
            render(display,index)
            clock.tick(30)
    except:
        mp_controller.close()
        cap.release()
        pygame.quit()

if __name__ == '__main__':
    main()
