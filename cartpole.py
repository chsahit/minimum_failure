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

def BalancingLQR(diagram):
    # Design an LQR controller for stabilizing the CartPole around the upright.
    # Returns a (static) AffineSystem that implements the controller (in
    # the original CartPole coordinates).

    context = diagram.CreateDefaultContext()
    diagram.get_input_port().FixValue(context, [0])

    context.get_mutable_continuous_state_vector().SetFromVector(UprightState())

    Q = np.diag((10., 10., 1., 1.))
    R = [1]

    # MultibodyPlant has many (optional) input ports, so we must pass the
    # input_port_index to LQR.
    return LinearQuadraticRegulator(
        diagram,
        context,
        Q,
        R,
        input_port_index=diagram.get_input_port().get_index())

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
cartpole = Parser(plant).AddModelFromFile("cart_pole.sdf")
plant.Finalize()

# Setup visualization
visualizer = ConnectMeshcatVisualizer(
    builder,
    scene_graph=scene_graph,
    zmq_url="new")
visualizer.vis.delete()
visualizer.set_planar_viewpoint(xmin=-2.5, xmax=2.5, ymin=-1.0, ymax=2.5)

builder.ExportInput(plant.get_actuation_input_port(), "command")
diagram = builder.Build()

# problem block 1: trying to do LQR with the diagram
# returns an error about the diagram not supporting toAutoDiffXd
# same issue when trying to call diagram.toAutoDiffXd() directly
"""
controller = builder.AddSystem(BalancingLQR(diagram))
builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
builder.Connect(controller.get_output_port(0),
                plant.get_actuation_input_port())
"""
# Set up a simulator to run this diagram
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
plant_context = plant.GetMyContextFromRoot(context)

# Problem block 2: This is following the linearization of the ballbot
# The error that comes up here is
# RuntimeError: The object named [] of type drake::systems::Diagram<double> does not support ToAutoDiffXd.
# I also get the same error calling diagram.ToAutoDiffXd()
"""
diag_context = diagram.CreateDefaultContext()
diagram.get_input_port().FixValue(diag_context, [0])
result = FirstOrderTaylorApproximation(diagram, context)
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
