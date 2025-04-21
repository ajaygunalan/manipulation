from pydrake.all import ModelVisualizer, PackageMap, StartMeshcat

# Set running_as_notebook to False since we're not in a notebook
running_as_notebook = False

# Start the visualizer
meshcat = StartMeshcat()

# Choose your robot model
robot = "Kuka LBR iiwa 7"
# robot = "Kuka LBR iiwa 14"
# robot = "Kinova Jaco Gen2 (7 DoF)"
# robot = "Franka Emika Panda"
# robot = "UR3e"

def get_model_url(description):
    # Note: I could download remote model resources here if necessary.
    if description == "Kuka LBR iiwa 7":
        return (
            "package://drake_models/iiwa_description/sdf/iiwa7_with_box_collision.sdf"
        )
    elif description == "Kuka LBR iiwa 14":
        return "package://drake_models/iiwa_description/urdf/iiwa14_primitive_collision.urdf"
    elif description == "Kinova Jaco Gen2 (7 DoF)":
        return "package://drake_models/jaco_description/urdf/j2s7s300.urdf"
    elif description == "Franka Emika Panda":
        return "package://drake_models/franka_description/urdf/panda_arm_hand.urdf"
    elif description == "UR3e":
        return "package://drake_models/ur_description/ur3e_cylinders_collision.urdf"
    raise Exception("Unknown model")

# Create visualizer and add the selected robot model
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.AddModels(url=get_model_url(robot))

# Run visualization
# For a script, we need to either:
# 1. Use loop_once=False to keep the visualization running until manually stopped
# 2. Or add some way to keep the script running (like input() or a loop)
visualizer.Run(loop_once=False)  # Changed from "not running_as_notebook" to explicitly False

# This won't be reached until visualization is stopped
meshcat.DeleteAddedControls()

# To keep the script running and the visualization visible
input("Press Enter to exit...")