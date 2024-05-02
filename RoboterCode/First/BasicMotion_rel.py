from franky import Affine, CartesianMotion, Robot, ReferenceType

robot = Robot("192.168.1.11")
robot.relative_dynamics_factor = 0.05

motion = CartesianMotion(Affine([0.05, 0.05, 0.0]), ReferenceType.Relative)
robot.move(motion)