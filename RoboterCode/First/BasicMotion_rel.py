from franky import Affine, CartesianMotion, Robot, ReferenceType, CartesianImpedanceMotion, ExponentialImpedanceMotion

robot = Robot("192.168.1.11")
robot.relative_dynamics_factor = 0.05

motion = CartesianMotion(Affine([0.0, 0.1, 0.0]), ReferenceType.Relative)
robot.move(motion)



#print("End of File")

