import time
import numpy as np
import scipy.sparse as spa
import matplotlib.pyplot as plt

import tsid
import plot_utils as plut
import ex_4_conf as conf

from numpy import nan
from numpy.linalg import norm as norm
from tsid_biped import TsidBiped


print("".center(conf.LINE_WIDTH,'#'))
print(" Test Walking ".center(conf.LINE_WIDTH, '#'))
print("".center(conf.LINE_WIDTH,'#'), '\n')


try:
    data = np.load(conf.DATA_FILE_TSID)
except FileNotFoundError as e:
    print("To run this walking example, you need to first execute ex_4_plan_LIPM_romeo.py and ex_4_LIPM_to_TSID.")
    raise e
    
nruns = 20
input(f"Press enter to start benchmarking, each solver {nruns} runs")

timings_proxqp = 0
timings_eiquadprog_fast = 0
timings_osqp = 0

for solver_name in ["PROXQP", "EIQUADPROG_FAST", "OSQP"]:
    print(f"Using solver {solver_name}")
    for _ in range(nruns):

        if solver_name == "PROXQP":
            solver = tsid.SolverProxQP("qp solver")
            solver.set_epsilon_absolute(1e-1)
        elif solver_name == "EIQUADPROG_FAST":
            solver = tsid.SolverHQuadProgFast("qp soNlver")
        elif solver_name == "OSQP":
            solver = tsid.SolverOSQP("qp solver")
            solver.set_epsilon_absolute(1e-1)


        tsid_biped = TsidBiped(conf, viewer=None)
        solver.resize(tsid_biped.formulation.nVar, tsid_biped.formulation.nEq, tsid_biped.formulation.nIn)

        N = data['com'].shape[1]
        N_pre  = int(conf.T_pre/conf.dt)
        N_post = int(conf.T_post/conf.dt)

        com_pos = np.empty((3, N+N_post))*nan
        com_vel = np.empty((3, N+N_post))*nan
        com_acc = np.empty((3, N+N_post))*nan
        x_LF   = np.empty((3, N+N_post))*nan
        dx_LF  = np.empty((3, N+N_post))*nan
        ddx_LF = np.empty((3, N+N_post))*nan
        ddx_LF_des = np.empty((3, N+N_post))*nan
        x_RF   = np.empty((3, N+N_post))*nan
        dx_RF  = np.empty((3, N+N_post))*nan
        ddx_RF = np.empty((3, N+N_post))*nan
        ddx_RF_des = np.empty((3, N+N_post))*nan
        f_RF = np.zeros((6, N+N_post))
        f_LF = np.zeros((6, N+N_post))
        cop_RF = np.zeros((2, N+N_post))
        cop_LF = np.zeros((2, N+N_post))
        tau    = np.zeros((tsid_biped.robot.na, N+N_post))
        q_log  = np.zeros((tsid_biped.robot.nq, N+N_post))
        v_log  = np.zeros((tsid_biped.robot.nv, N+N_post))

        contact_phase = data['contact_phase']
        com_pos_ref = np.asarray(data['com'])
        com_vel_ref = np.asarray(data['dcom'])
        com_acc_ref = np.asarray(data['ddcom'])
        x_RF_ref    = np.asarray(data['x_RF'])
        dx_RF_ref   = np.asarray(data['dx_RF'])
        ddx_RF_ref  = np.asarray(data['ddx_RF'])
        x_LF_ref    = np.asarray(data['x_LF'])
        dx_LF_ref   = np.asarray(data['dx_LF'])
        ddx_LF_ref  = np.asarray(data['ddx_LF'])
        cop_ref     = np.asarray(data['cop'])
        com_acc_des = np.empty((3, N+N_post))*nan # acc_des = acc_ref - Kp*pos_err - Kd*vel_err

        x_rf   = tsid_biped.get_placement_RF().translation
        offset = x_rf - x_RF_ref[:,0]
        for i in range(N):
            com_pos_ref[:,i] += offset + np.array([0.,0.,0.0])
            x_RF_ref[:,i] += offset
            x_LF_ref[:,i] += offset

        t = -conf.T_pre
        q, v = tsid_biped.q, tsid_biped.v

        for i in range(-N_pre, N+N_post):
            time_start = time.time()
            
            if i==0:
                # print("Starting to walk (remove contact left foot)")
                tsid_biped.remove_contact_LF()
            elif i>0 and i<N-1:
                if contact_phase[i] != contact_phase[i-1]:
                    # print("Time %.3f Changing contact phase from %s to %s"%(t, contact_phase[i-1], contact_phase[i]))
                    if contact_phase[i] == 'left':
                        tsid_biped.add_contact_LF()
                        tsid_biped.remove_contact_RF()
                    else:
                        tsid_biped.add_contact_RF()
                        tsid_biped.remove_contact_LF()
            
            
            if i<0:
                tsid_biped.set_com_ref(com_pos_ref[:,0], 0*com_vel_ref[:,0], 0*com_acc_ref[:,0])
            elif i<N:
                tsid_biped.set_com_ref(com_pos_ref[:,i], com_vel_ref[:,i], com_acc_ref[:,i])
                tsid_biped.set_LF_3d_ref(x_LF_ref[:,i], dx_LF_ref[:,i], ddx_LF_ref[:,i])
                tsid_biped.set_RF_3d_ref(x_RF_ref[:,i], dx_RF_ref[:,i], ddx_RF_ref[:,i])
            
            HQPData = tsid_biped.formulation.computeProblemData(t, q, v)

            tic = time.perf_counter_ns()
            sol = solver.solve(HQPData)
            toc = time.perf_counter_ns()

            if solver_name == "PROXQP":
                timings_proxqp += (toc - tic)
            elif solver_name == "EIQUADPROG_FAST":
                timings_eiquadprog_fast += (toc - tic)
            elif solver_name == "OSQP":
                timings_osqp += (toc - tic)

            
            if(sol.status!=0):
                print("QP problem could not be solved! Error code:", sol.status)
                break
            if norm(v,2)>40.0:
                print("Time %.3f Velocities are too high, stop everything!"%(t), norm(v))
                break
            
            if i>0:
                q_log[:,i] = q
                v_log[:,i] = v
                tau[:,i] = tsid_biped.formulation.getActuatorForces(sol)
            dv = tsid_biped.formulation.getAccelerations(sol)
            
            if i>=0:
                com_pos[:,i] = tsid_biped.robot.com(tsid_biped.formulation.data())
                com_vel[:,i] = tsid_biped.robot.com_vel(tsid_biped.formulation.data())
                com_acc[:,i] = tsid_biped.comTask.getAcceleration(dv)
                com_acc_des[:,i] = tsid_biped.comTask.getDesiredAcceleration
                x_LF[:,i], dx_LF[:,i], ddx_LF[:,i] = tsid_biped.get_LF_3d_pos_vel_acc(dv)
                if not tsid_biped.contact_LF_active:
                    ddx_LF_des[:,i] = tsid_biped.leftFootTask.getDesiredAcceleration[:3]
                x_RF[:,i], dx_RF[:,i], ddx_RF[:,i] = tsid_biped.get_RF_3d_pos_vel_acc(dv)
                if not tsid_biped.contact_RF_active:
                    ddx_RF_des[:,i] = tsid_biped.rightFootTask.getDesiredAcceleration[:3]
                
                if tsid_biped.formulation.checkContact(tsid_biped.contactRF.name, sol):
                    T_RF = tsid_biped.contactRF.getForceGeneratorMatrix
                    f_RF[:,i] = T_RF @ tsid_biped.formulation.getContactForce(tsid_biped.contactRF.name, sol)
                    if(f_RF[2,i]>1e-3): 
                        cop_RF[0,i] = f_RF[4,i] / f_RF[2,i]
                        cop_RF[1,i] = -f_RF[3,i] / f_RF[2,i]
                if tsid_biped.formulation.checkContact(tsid_biped.contactLF.name, sol):
                    T_LF = tsid_biped.contactRF.getForceGeneratorMatrix
                    f_LF[:,i] = T_LF @ tsid_biped.formulation.getContactForce(tsid_biped.contactLF.name, sol)
                    if(f_LF[2,i]>1e-3): 
                        cop_LF[0,i] = f_LF[4,i] / f_LF[2,i]
                        cop_LF[1,i] = -f_LF[3,i] / f_LF[2,i]


            q, v = tsid_biped.integrate_dv(q, v, dv, conf.dt)
            t += conf.dt


N_sum = (N + N_post + N_pre) * nruns
print("Timings in ms")
print(f"Proxqp              {timings_proxqp * 1e-6 / N_sum}")
print(f"Eiquadprog fast     {timings_eiquadprog_fast * 1e-6 / N_sum}")
print(f"Osqp                {timings_osqp * 1e-6 / N_sum}")
