#  Copyright: 2022 Alex Karavaev <alexkaravev@gmail.com>
#  License: MIT
#   Permission is hereby granted, free of charge, to any person obtaining a
#   copy of this software and associated documentation files (the "Software"),
#   to deal in the Software without restriction, including without limitation
#   the rights to use, copy, modify, merge, publish, distribute, sublicense,
#   and/or sell copies of the Software, and to permit persons to whom the
#   Software is furnished to do so, subject to the following conditions:
#   .
#   The above copyright notice and this permission notice shall be included
#   in all copies or substantial portions of the Software.
#   .
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
#   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
#   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
#   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
#   CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
#   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

import casadi as cs
import opengen as og
import numpy as np

N = 10  # The MPC horizon length
NX = 3  # The number of elements in the state vector
NU = 2  # The number of elements in the control vector
sampling_time = 0.06
NSim = 100

Q = cs.DM.eye(NX) * [50.0, 50.0, 100.]
R = cs.DM.eye(NU) * [10.0, 50.0]
QN = cs.DM.eye(NX) * [50.0, 50.0, 50.]

def dynamics_ct(_x, _u):
    return cs.vcat([_u[0] * cs.cos(_x[2]),
                    _u[0] * cs.sin(_x[2]),
                    _u[1]])


def dynamics_dt(x, u):

    dx = dynamics_ct(x, u)

    return cs.vcat([x[i] + sampling_time * dx[i] for i in range(NX)])


# The stage cost for x and u
def stage_cost(_x, _u, _x_ref, _u_ref=None):
    if _u_ref is None:
        _u_ref = cs.DM.zeros(_u.shape)
    dx = _x - _x_ref
    du = _u - _u_ref
    return cs.mtimes([dx.T, Q, dx]) + cs.mtimes([du.T, R, du])


# The terminal cost for x
def terminal_cost(_x, _x_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    dx = _x - _x_ref
    return cs.mtimes([dx.T, QN, dx])


states = [cs.MX.sym('x_0_' + str(i), NX) for i in range(N)]
x_ref = [cs.MX.sym('x_ref_' + str(i), NX) for i in range(N)]
u_k = [cs.MX.sym('u_' + str(i), NU) for i in range(N)]
u_ref = [cs.MX.sym('u_ref_' + str(i), NU) for i in range(N)]

total_cost = 0

init = states[0] - x_ref[0]
total_cost += cs.mtimes([init.T, Q, init])

for t in range(0, N-1):
    x_next = dynamics_dt(states[t], u_k[t])  # update state
    dx = x_next-states[t+1]
    total_cost+=cs.mtimes([dx.T, QN, dx])

for t in range(0, N-1):
    total_cost += stage_cost(states[t+1], u_k[t+1], x_ref[t], u_ref[t+1])


optimization_variables = []
optimization_parameters = []

optimization_variables += u_k
optimization_variables += states

optimization_parameters += u_ref
optimization_parameters += x_ref

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters)

umin = [0.0, -3.1] * N  # - cs.DM.ones(NU * N) * cs.inf
umax = [2.0, 3.1] * N  # cs.DM.ones(NU * N) * cs.inf

bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(optimization_variables,
                             optimization_parameters,
                             total_cost) \
    .with_constraints(bounds)

ros_config = og.config.RosConfiguration() \
    .with_package_name("open_nmpc_controller") \
    .with_node_name("open_mpc_controller_node") \
    .with_rate((int)(1.0/sampling_time)) \
    .with_description("Cool ROS node.")

build_config = og.config.BuildConfiguration() \
    .with_build_directory("../src/") \
    .with_build_mode("release") \
    .with_build_c_bindings() \
    .with_ros(ros_config)

meta = og.config.OptimizerMeta() \
    .with_optimizer_name("mpc_controller")

solver_config = og.config.SolverConfiguration() \
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()

