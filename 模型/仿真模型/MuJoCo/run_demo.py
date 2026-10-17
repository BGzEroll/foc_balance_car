"""Direct joint torque input in N*m; no balance controller is included."""
from pathlib import Path
import time
import mujoco
import mujoco.viewer
from load_model import load_model
model=load_model()
data=mujoco.MjData(model)
with mujoco.viewer.launch_passive(model,data) as viewer:
    while viewer.is_running():
        start=time.perf_counter()
        data.ctrl[:]=0.0  # Replace with controller/policy outputs: left and right torque.
        mujoco.mj_step(model,data)
        viewer.sync()
        time.sleep(max(0,model.opt.timestep-(time.perf_counter()-start)))
