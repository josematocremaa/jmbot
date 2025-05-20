import numpy as np 
import PyKDL as kdl 

a, b, c, d, e  = 140, 106, 140, 100, 0

chain = kdl.Chain()
#                  A alpha D theta
H_0_1 = kdl.Frame.DH(0,np.deg2rad(-90),a,0)
H_1_2 = kdl.Frame.DH(b,0,0,np.deg2rad(-90))
H_2_3 = kdl.Frame.DH(c,0,0,np.deg2rad(+90))
H_3_4 = kdl.Frame.DH(0,np.deg2rad(+90),-e,np.deg2rad(+90))
H_4_5 = kdl.Frame.DH(0,0,d,0)

axis = kdl.Joint(kdl.Joint.RotZ)

chain.addSegment(kdl.Segment(axis, H_0_1))
chain.addSegment(kdl.Segment(axis, H_1_2))
chain.addSegment(kdl.Segment(axis, H_2_3))
chain.addSegment(kdl.Segment(axis, H_3_4))
chain.addSegment(kdl.Segment(axis, H_4_5))

fk_solver = kdl.ChainFkSolverPos_recursive(chain)
ik_solver_vel = kdl.ChainIkSolverVel_pinv(chain)
ik_solver = kdl.ChainIkSolverPos_NR(chain, fk_solver, ik_solver_vel,100, 1e-6)
#posicion inicial del robot
q = kdl.JntArray(5)
q[0] = 0
q[1] = 0
q[2] = 0
q[3] = 0
q[4] = 0
# q_a = (0, 24.83, 37.83, 0, 27.34, 0)
# for i in range(5):
#     q[i] = np.deg2rad(q_a[i])

# H_0_5=kdl.Frame()

#solver cinematica directa y inversa

# ik_solver_vel = kdl.ChainIkSolverVel_pinv(chain)
# fk_solver_pos= kdl.ChainFkSolverPos_recursive(chain)
# ik_solver = kdl.ChainIkSolverPos_NR(chain, fk_solver_pos, ik_solver_vel, 100, 1e-6)


H_0_5 = kdl.Frame()
ret=fk_solver.JntToCart(q,H_0_5)
print(H_0_5)

p_in=kdl.Frame(kdl.Vector(c, 0, a + d + b));  
q_dot = kdl.JntArray(5)

ret = ik_solver.CartToJnt(q,p_in, q_dot)





print("Resultado de CartToJnt:", ret)
print("Valores de q_out:", [q_dot[i] for i in range(q_dot.rows())])


q_out_list = [q_dot[i] for i in range(q_dot.rows())]  # Convertir JntArray a lista
q_out_deg = np.degrees(q_out_list)  # Convertir radianes a grados

print(q_out_deg)