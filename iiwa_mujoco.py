import numpy as np
import matplotlib.pyplot as plt
# import arcpy
import mujoco
import mediapy as media
import time


### performance test
# simulation duration 1000s
# execution time: 1694ms

# filename = "/home/geraldebmer/repos/test_mujoco/iiwa/robot.xml"
filename = "robot.xml"
with open(filename, 'r') as f:
  xml = f.read()


model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)



mujoco.mj_forward(model, data)
renderer.update_scene(data)
media.show_image(renderer.render())

DURATION  = 1000   # seconds
FRAMERATE = 60  # Hz


# mujoco.mj_resetDataKeyframe(model, data, 1)

frames = []
i = 0

# print(data.timestep)
# get time in microseconds
start = time.time()

while data.time < DURATION:
  # Step the simulation.
  mujoco.mj_step(model, data)
  # print(str(i) +": "+ str(data.time), end='\r')
  # i = i +1
  # break
  # Render and save frames.
  # if len(frames) < data.time * FRAMERATE:
  #   renderer.update_scene(data)
  #   pixels = renderer.render()
  #   frames.append(pixels)

end = time.time()
print("time: " + str(end - start))

# Display video.
# media.show_video(frames, fps=FRAMERATE)
