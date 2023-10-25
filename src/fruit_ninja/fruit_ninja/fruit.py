#!/usr/bin/env python3

import time, random
try:
    import pygame
    import mediapipe as mp
except:
    print("Pygame and Mediapipe should be installed. pip install pygame")
    raise ImportError

class Fruit:
    def __init__(self, surface:pygame.Surface):
        self.surface = surface
        self.pos = [random.randint(160,480),400]
        self.targety = random.randint(80,160)
        self.distx = random.randint(100,240) * random.choice([-1,1])
        self.centerx = self.pos[0] + self.distx/2
        self.duration = random.uniform(2,3)
        self.vx = self.distx/(self.duration*30)
        self.height = self.surface.get_height()
        self.width = self.surface.get_width()

    def update(self):
        self.pos[0] += self.vx
        self.pos[1] = (4.00*(400-self.targety)/self.distx**2)*(self.pos[0]-self.centerx)**2 + self.targety

    def checkHover(self,pos):
        return ((self.pos[0]-self.width/2-pos[0])**2+(self.pos[1]-self.width/2-pos[1])**2)**0.5 < self.width/2

    def draw(self, surface):
        surface.blit(self.surface,(int(self.pos[0]-self.width),int(self.pos[1]-self.height)))

class MP_Controller:
    def __init__(self, mode=1):
        self.hand_result = mp.tasks.vision.HandLandmarkerResult
        self.hand_landmarker = mp.tasks.vision.HandLandmarker
        self._createHandLandmarker()

        if mode == 2:
            self.face_result = mp.tasks.vision.FaceLandmarkerResult
            self.face_landmarker = mp.tasks.vision.FaceLandmarker
            self._createFaceLandmarker()

    def _createHandLandmarker(self):
        # callback function
        def update_result(
            hand_result: mp.tasks.vision.HandLandmarkerResult,
            output_image: mp.Image,
            timestamp_ms: int,
        ):
            self.hand_result = hand_result

        options_hands = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(
                model_asset_path="hand_landmarker.task"
            ),  # path to model
            running_mode=mp.tasks.vision.RunningMode.LIVE_STREAM,  # running on a live stream
            num_hands=1,  # track both hands
            min_hand_detection_confidence=0.5,  # lower than value to get predictions more often
            min_hand_presence_confidence=0.5,  # lower than value to get predictions more often
            min_tracking_confidence=0.5,  # lower than value to get predictions more often
            result_callback=update_result,
        )

        # initialize landmarker
        self.hand_landmarker = self.hand_landmarker.create_from_options(options_hands)

    def _createFaceLandmarker(self):
        # callback function
        def update_result(
            face_result: mp.tasks.vision.FaceLandmarkerResult,
            output_image: mp.Image,
            timestamp_ms: int,
        ):
            self.face_result = face_result

        # HandLandmarkerOptions (details here: https://developers.google.com/mediapipe/solutions/vision/hand_landmarker/python#live-stream)
        options_face = mp.tasks.vision.FaceLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(
                model_asset_path="face_landmarker.task"
            ),  # path to model
            running_mode=mp.tasks.vision.RunningMode.LIVE_STREAM,  # running on a live stream
            num_faces=1,
            min_face_detection_confidence=0.5,  # lower than value to get predictions more often
            min_face_presence_confidence=0.5,  # lower than value to get predictions more often
            min_tracking_confidence=0.5,  # lower than value to get predictions more often
            result_callback=update_result,
        )

        # initialize landmarker
        self.face_landmarker = self.face_landmarker.create_from_options(options_face)

    def detect_async(self, frame, mode):
        # convert np frame to mp image
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        # detect landmarks
        self.hand_landmarker.detect_async(
            image=mp_image, timestamp_ms=int(time.time() * 1000)
        )

        if mode == 2:
            self.face_landmarker.detect_async(
                image=mp_image, timestamp_ms=int(time.time() * 1000)
            )

    def get_index_tip_coordinates(self):
        if self.hand_result.hand_landmarks != []:
            print(
                "HandLandmark.INDEX_FINGER_TIP result:\n {}".format(
                    self.hand_result.hand_landmarks[0][8]
                )
            )  # (HandLandmark.INDEX_FINGER_TIP=8)

            # GET INDEX_FINGER POSITION
            return (
                self.hand_result.hand_landmarks[0][8].x
                ,self.hand_result.hand_landmarks[0][8].y
                # ,self.hand_result.hand_landmarks[0][8].z
            )

    def close(self):
        # close landmarker
        self.hand_landmarker.close()
        self.face_landmarker.close()

class Particles:
    # [loc, velocity, timer]
    particles = []
    mpos = None

    def _circle_surf(self, radius, color):
        surf = pygame.Surface((radius * 2, radius * 2))
        pygame.draw.circle(surf, color, (radius, radius), radius)
        surf.set_colorkey((0, 0, 0))
        return surf

    def set_source_pos(self, pos=None):
        self.mpos = pos
    
    def draw(self, screen):
        if self.mpos:
            self.particles.append([[self.mpos[0], self.mpos[1]], [random.randint(0, 20) / 10 - 1, -random.randint(0, 30) / 10], random.randint(5, 10)])

        for particle in self.particles:
            particle[0][0] += particle[1][0]
            particle[0][1] += particle[1][1]
            particle[2] -= 0.2
            particle[1][1] += 0.2
            pygame.draw.circle(screen, (255, 255, 255), [int(particle[0][0]), int(particle[0][1])], int(particle[2]))

            radius = particle[2] * 2
            screen.blit(self._circle_surf(radius, (20, 20, 60)), (int(particle[0][0] - radius), int(particle[0][1] - radius)), special_flags=pygame.BLEND_RGB_ADD)

            if particle[2] <= 0:
                self.particles.remove(particle)
                