import numpy as np

from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import ConnectMeshcatVisualizer
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import FirstOrderTaylorApproximation
from pydrake.systems.drawing import plot_system_graphviz
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import PrismaticJoint, LinearSpringDamper
from pydrake.common import FindResourceOrThrow
import matplotlib.pyplot as plt

def xyz_rpy_deg(xyz, rpy_deg):
    """Shorthand for defining a pose."""
    rpy_deg = np.asarray(rpy_deg)
    return RigidTransform(RollPitchYaw(rpy_deg * np.pi / 180), xyz)

def UprightState():
    state = (0, np.pi, 0, 0)
    return state

def BalancingLQR(plant):
    # Design an LQR controller for stabilizing the CartPole around the upright.
    # Returns a (static) AffineSystem that implements the controller (in
    # the original CartPole coordinates).

    context = plant.CreateDefaultContext()
    plant.get_actuation_input_port().FixValue(context, [0])

    context.get_mutable_continuous_state_vector().SetFromVector(UprightState())

    Q = np.diag((10., 10., 1., 1.))
    R = [1]

    # MultibodyPlant has many (optional) input ports, so we must pass the
    # input_port_index to LQR.
    return LinearQuadraticRegulator(
        plant,
        context,
        Q,
        R,
        input_port_index=plant.get_actuation_input_port().get_index())

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
cartpole = Parser(plant).AddModelFromFile("cart_pole.sdf")
plant.Finalize()
# I tried this line but as we can see from the diagram the input port is already connected
#builder.Connect(scene_graph.get_query_output_port(), plant.get_geometry_query_input_port())

# Setup visualization
visualizer = ConnectMeshcatVisualizer(
    builder,
    scene_graph=scene_graph,
    zmq_url="new")
visualizer.vis.delete()
visualizer.set_planar_viewpoint(xmin=-2.5, xmax=2.5, ymin=-1.0, ymax=2.5)

#The below lines (as well as the BalancingLQR definition) come from the chapter three colab ntbk, they give the following error
#RuntimeError: The geometry query input port (see MultibodyPlant::get_geometry_query_input_port()) of this MultibodyPlant is not connected. Please connect thegeometry query output port of a SceneGraph object (see SceneGraph::get_query_output_port()) to this plants input port in a Diagram.
"""
controller = builder.AddSystem(BalancingLQR(plant))
builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
builder.Connect(controller.get_output_port(0),
                plant.get_actuation_input_port())
"""

# This line is only needed when linearizing the whole diagram
builder.ExportInput(plant.get_actuation_input_port(), "command")
diagram = builder.Build()
# Set up a simulator to run this diagram
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
plant_context = plant.GetMyContextFromRoot(context)

# This is following the linearization of the ballbot
# The error that comes up here is
# RuntimeError: The object named [] of type drake::systems::Diagram<double> does not support ToAutoDiffXd.
"""
diag_context = diagram.CreateDefaultContext()
diagram.get_input_port().FixValue(diag_context, [0])
result = FirstOrderTaylorApproximation(diagram, context)
"""

# This is another attempt of linearization - just of the plant not the whole diagram
# If I use "dummy_context" in the linearization I get the error written above the LQR component
# If we use "plant_context" in the linearization I get the error
#RuntimeError: System::FixInputPortTypeCheck(): expected value of type drake::geometry::QueryObject<drake::AutoDiffXd> for input port 'geometry_query' (index 0) but the actual type was drake::geometry::QueryObject<double>. (System ::plant)
"""
dummy_context = plant.CreateDefaultContext()
dummy_context.SetContinuousState([0, np.pi, 0, 0])
result = FirstOrderTaylorApproximation(plant, plant_context,
       input_port_index=plant.get_actuation_input_port().get_index(),
       output_port_index=plant.get_state_output_port().get_index())
"""

plant.get_actuation_input_port(cartpole).FixValue(plant_context, np.array([0]))
plot_system_graphviz(diagram)
plt.show()
# Set the initial conditions
context.SetContinuousState([0, np.pi, 0, 0.00]) # x, theta, xdot, thetadot
context.SetTime(0.0)

visualizer.start_recording()
simulator.AdvanceTo(10.0)
visualizer.publish_recording()
visualizer.vis.render_static()

input()
