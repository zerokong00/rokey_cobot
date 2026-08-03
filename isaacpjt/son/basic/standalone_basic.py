from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
import omni.timeline

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim2 = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 0.3]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)

# cube_prim3 = DynamicCuboid(                              # 4. Prim
#     prim_path="/World/GreenCube",
#     name="green_cube",
#     position=np.array([0.0, 0.0, 1.0]),
#     scale=np.array([0.1, 0.1, 0.1]),
#     color=np.array([0.0, 1.0, 0.0]),
# )

world.scene.add_default_ground_plane()                  # 5. Scene
red_cube = world.scene.add(cube_prim2)
# world.scene.add(cube_prim3)

world.reset()

timeline = omni.timeline.get_timeline_interface()
prev_is_playing = False
step_count = 0

while simulation_app.is_running():                      # 6. Simulation
    is_playing = timeline.is_playing()

    if is_playing and not prev_is_playing:
        step_count = 0
        print("[리셋] Play 시작 >> step_count = 0")

    prev_is_playing = is_playing

    world.step(render=True)
    time.sleep(0.01)
    step_count += 1

    if step_count % 100 == 0:
        print("step : ", step_count)

    if step_count % 300 == 0:
        print("[이동] 큐브 순간이동")
        cube = world.scene.get_object("red_cube")
        cube.set_world_pose(position=[0.0, 0.0, 1.0])
        

    # if step_count == 500:
    #     print("시뮬레이션 종료")
    #     break

simulation_app.close()
