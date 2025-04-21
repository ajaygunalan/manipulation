from pydrake.all import ModelVisualizer, PackageMap, StartMeshcat

# Set running_as_notebook to False since we're not in a notebook
running_as_notebook = False

# Start the visualizer
meshcat = StartMeshcat()

# First section: Drake robot models
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

# Run visualization for the first model
visualizer.Run(loop_once=True)  # Use loop_once=True to proceed to the next section
meshcat.DeleteAddedControls()

# Second section: dm_control model (only if appropriate conditions are met)
# Since we're not in a notebook, we may want to conditionally run this part
# For demonstration purposes, let's create a new variable to control this
# visualize_dm_control = True  # Set to True or False as needed

# if visualize_dm_control:  # Changed from running_as_notebook
#     meshcat.Delete()
#     visualizer = ModelVisualizer(meshcat=meshcat)
#     visualizer.parser().package_map().AddRemote(
#         package_name="dm_control",
#         params=PackageMap.RemoteParams(
#             urls=[
#                 f"https://github.com/google-deepmind/dm_control/archive/refs/tags/1.0.15.tar.gz"
#             ],
#             sha256=("bac091b18689330a99b7c18ddf86baa916527f5e4ab8e3ded0c8caff1dab2048"),
#             strip_prefix="dm_control-1.0.15/",
#         ),
#     )
#     visualizer.AddModels(url="package://dm_control/dm_control/suite/cheetah.xml")
#     plant = visualizer.parser().plant()
#     visualizer.Run(loop_once=False)  # Keep visualization running until closed
#     meshcat.DeleteAddedControls()

# # Keep the script running to maintain the visualization


input("Press Enter to exit...")