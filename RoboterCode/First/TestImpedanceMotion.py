#from franky import Affine, CartesianMotion, Robot, ReferenceType, CartesianImpedanceMotion, ExponentialImpedanceMotion
import franky
robot = franky.Robot("192.168.1.11")
robot.relative_dynamics_factor = 0.05

motionStop = franky.CartesianPoseStopMotion()
robot.move(motionStop)

motion = franky.CartesianImpedanceMotion(franky.Affine([0.1, 0.1, 0.0]), franky.ReferenceType.Relative, force_constraints=[5,5,5,5,5,5])
robot.move(motion)



print("End of File")

