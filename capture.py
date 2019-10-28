import cv2
import random
import argparse
import os

capture_dev_num = sum([1 for f in os.listdir("/dev") if 'video' in f])
print(list(range(capture_dev_num)[-2::-1]))
devices = [('cam_%d' % i, cv2.VideoCapture(i)) for i in [capture_dev_num-1, capture_dev_num-2]] # we only need the last 2 video device

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, help="What seed to use for the point generation [important: seed must be the same as the seed of the projector script]", required=False, default=0)
parser.add_argument("--width", type=int, help="What is the width of the projector")
parser.add_argument("--height", type=int, help="What is the height of the projector")
parser.add_argument("--out-dir", type=str, help="Where to save the images", required=False, default="./out")

args = parser.parse_args()

random.seed(args.seed)

if not os.path.exists(args.out_dir):
    os.mkdir(args.out_dir)


def capture(index):
    if not os.path.exists(os.path.join(args.out_dir, str(index))):
        os.mkdir(os.path.join(args.out_dir, str(index)))
    for name, cap in devices:
        _, frame = cap.read()
        cv2.imwrite(os.path.join(args.out_dir, str(index), name) + ".png", frame)
    open(os.path.join(args.out_dir, str(index), "label.txt"), "w").write(str(coordinates[index]))
    print("[%d] captured data to directory: %s" % (index, os.path.join(args.out_dir, str(index))))


current_index = 0
coordinates = []
while True:
    if current_index < 0:
        current_index = 0
        print("[%d] Cannot go below 0." % current_index)
    if len(coordinates) == current_index:
        coordinates.append([random.randint(0, args.width), random.randint(0, args.height)])
        capture(current_index)

    answer = input("[%d] 0: move one index down; 1: recapture current index; 2: move to the next index; 3: quit >>> " % current_index)
    if answer == '0':
        current_index -= 1
    elif answer == '1':
        capture(current_index)
    elif answer == '2':
        current_index += 1
    elif answer == '3':
        break

