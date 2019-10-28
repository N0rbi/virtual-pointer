import pygame
import sys
import argparse
import random

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, help="What seed to use for the point generation [important: seed must be the same as the seed of the projector script]", required=False, default=0)
parser.add_argument("--width", type=int, help="What is the width of the projector")
parser.add_argument("--height", type=int, help="What is the height of the projector")
parser.add_argument("--out-dir", type=str, help="Where to save the images", required=False, default="./out")

args = parser.parse_args()

random.seed(args.seed)

pygame.init()
screen=pygame.display.set_mode((args.width, args.height), pygame.FULLSCREEN)

current_index = 0
coordinates = []

while True:
    if current_index < 0:
        current_index = 0
        print("[%d] Cannot go below 0." % current_index)

    if len(coordinates) == current_index:
        coordinates.append([random.randint(0, args.width), random.randint(0, args.height)])

    screen.fill((255,255,255))

    pygame.draw.circle(screen, (0,0,0), tuple(coordinates[current_index]), 5)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_0:
                current_index -= 1
            elif event.key == pygame.K_1:
                pass
            elif event.key == pygame.K_2:
                current_index += 1
            elif event.key == pygame.K_3:
                pygame.quit()
                sys.exit()
    pygame.display.update()