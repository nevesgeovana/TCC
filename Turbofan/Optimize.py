# Optimize.py
# Created:  Feb 2016, M. Vegh
# Modified: Aug 2017, E. Botero
#           Aug 2018, T. MacDonald

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
import Vehicles
import Analyses
import Missions
import Procedure
import Plot_Mission
import matplotlib.pyplot as plt
from SUAVE.Optimization import Nexus, carpet_plot
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
from copy import deepcopy
from timeit import time
# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():

    t0 = time.time()

    problem = setup()

    ## Base Input Values
    output = problem.objective()
    print output
    Plot_Mission.plot_mission(problem, -1)

    # # Uncomment to view contours of the design space
    # variable_sweep(problem)

    output = scipy_setup.SciPy_Solve(problem, solver='SLSQP')
    print output

    print 'fuel burn   = ', problem.summary.base_mission_fuelburn
    print 'Constraints = ', problem.all_constraints()

    Plot_Mission.plot_mission(problem, 0)

    # ------------------------------------------------------------------
    # Pareto

    fuelburn = []
    allconstraints = []
    MTOW = []
    finalvalue = []
    grad = []
    tofl = []

    betas = [1., 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.]

    fid = open('Pareto_results.txt', 'w')  # Open output file
    fid.write(' Pareto results \n')

    for i, beta in enumerate(betas):
        fid.write('Pareto Frontier. Run number: ' + str(i) + ' \n')
        print('Pareto Frontier. Run number: ' + str(i))

        # updating Beta value
        design_vars = problem.optimization_problem.inputs[:, 1]
        design_vars[-1] = design_vars[-1] * beta
        problem.optimization_problem.inputs[:, 1] = design_vars

        bounds = problem.optimization_problem.inputs[:, 2]
        bounds[-1] = list(bounds[-1])
        bounds[-1][0] = bounds[-1][0] * beta
        bounds[-1][1] = bounds[-1][1] * beta
        bounds[-1] = tuple(bounds[-1])
        problem.optimization_problem.inputs[:, 2] = bounds

        output = scipy_setup.SciPy_Solve(problem, solver='SLSQP')
        print output
        print 'fuel burn   = ', problem.summary.base_mission_fuelburn
        print 'MTOW        = ', problem.summary.MTOW
        print 'Constraints = ', problem.all_constraints()

        Plot_Mission.plot_mission(problem, i)

        finalvalue.append(output)
        fuelburn.append(problem.summary.base_mission_fuelburn)
        allconstraints.append(problem.all_constraints())
        grad.append(problem.summary.second_segment_climb_gradient_takeoff)
        tofl.append(problem.summary.takeoff_field_length)
        MTOW.append(problem.summary.MTOW)

        fid.write(str(fuelburn[-1])+' \n')
        fid.write(str(grad[-1]) + ' \n')
        fid.write(str(tofl[-1]) + ' \n')
        fid.write(str(MTOW[-1]) + ' \n')
        fid.write(str(allconstraints[-1]) + ' \n')
        fid.write(str(finalvalue[-1]) + ' \n')
        fid.write('\n \n')

    elapsed = time.time() - t0

    fid.write('Total run time: ' + str(elapsed))
    print('Total run time: ' + str(elapsed))

    fid.close()

    return

# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  

def setup():

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    #   [ tag                            , initial, (lb,ub)             , scaling , units ]
    problem.inputs = np.array([
        ['wing_area',     72.72, (60.0, 85.0),     100.,       Units.meter**2],
        ['aspect_ratio',    8.6,     (6.6, 10.6),  100.0,           Units.less],
        # ['taper_ratio',  0.3275, (0.2875, 0.4275),   1.,           Units.less],
        ['t_c_ratio',      0.11,   (0.09, 0.15),     1.,           Units.less],
        ['sweep_angle',    23.0,   (15.0, 35.0),  100.0,            Units.deg],
        ['cruise_range',  1078., (800., 1350.), 10000.0, Units.nautical_miles],
        ['beta',             1., (1., 1.), 1.,                  Units.less],
    ])

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # throw an error if the user isn't specific about wildcards
    # [ tag, scaling, units ]
    problem.objective = np.array([
        # ['fuel_burn', 10000., Units.kg],
        ['objective', 10., Units.less],
    ])
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    # CONSTRAINTS ARE SET TO BE BIGGER THAN ZERO, SEE PROCEDURE (SciPy's SLSQP optimization algorithm assumes this form)
    problem.constraints = np.array([
        ['fuel_margin',              '>', 0., 1000.,  Units.kg],   #fuel margin defined here as fuel
        ['Throttle_min',             '>', 0., 1.,   Units.less],
        ['Throttle_max',             '>', 0., 1.,   Units.less],
        ['tofl_mtow_margin',         '>', 0., 100.,    Units.m],  # take-off field length
        ['mzfw_consistency',         '>', 0., 1000.,  Units.kg],  # MZFW consistency
        ['design_range_ub',          '>', 0., 1., Units.nautical_miles],  # Range consistency
        ['design_range_lb',          '>', 0., 1., Units.nautical_miles],  # Range consistency
        ['time_to_climb',            '>', 0., 10.,  Units.min],  # Time to climb consistency
        ['climb_gradient',           '>', 0., 1.,  Units.less],  # second segment climb gradient
        ['lfl_mlw_margin',           '>', 0., 100.,   Units.m],  # landing field length
        ['max_fuel_margin',          '>', 0., 1000., Units.kg],  # max fuel margin
        ['range_HH_margin',          '>', 0., 1000., Units.nautical_miles],  # Range for Hot and High
    ])
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        ['wing_area',                  ['vehicle_configurations.*.wings.main_wing.areas.reference',
                                        'vehicle_configurations.*.reference_area'                     ]],
        ['aspect_ratio',                'vehicle_configurations.*.wings.main_wing.aspect_ratio'        ],
        # ['taper_ratio',                 'vehicle_configurations.*.wings.main_wing.taper'               ],
        ['t_c_ratio',                   'vehicle_configurations.*.wings.main_wing.thickness_to_chord'  ],
        ['sweep_angle',                 'vehicle_configurations.*.wings.main_wing.sweeps.quarter_chord'],
        ['cruise_range',                'missions.base.segments.cruise.distance'                       ],
        # ['fuel_burn',                   'summary.base_mission_fuelburn'                                ],
        ['fuel_margin',                 'summary.fuel_margin'                                          ],
        ['Throttle_min',                'summary.throttle_min'                                         ],
        ['Throttle_max',                'summary.throttle_max'                                         ],
        ['tofl_mtow_margin',            'summary.takeoff_field_length_margin'                          ],
        ['mzfw_consistency',            'summary.mzfw_consistency'                                     ],
        ['design_range_ub',             'summary.design_range_ub'                                      ],
        ['design_range_lb',             'summary.design_range_lb'                                      ],
        ['time_to_climb',               'summary.time_to_climb'                                        ],
        ['climb_gradient',              'summary.climb_gradient'                                       ],
        ['lfl_mlw_margin',              'summary.lfl_mlw_margin'                                       ],
        ['max_fuel_margin',             'summary.max_fuel_margin'                                      ],
        ['range_HH_margin',             'summary.range_HH_margin'],
        ['beta',                        'vehicle_configurations.base.wings.main_wing.beta'],
        ['objective',                   'summary.objective'],
    ]    
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.setup()
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations)
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.setup(nexus.analyses)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()    
    nexus.total_number_of_iterations = 0
    return nexus
    
def variable_sweep(problem):    
    number_of_points = 5
    outputs     = carpet_plot(problem, number_of_points, 0, 0)  #run carpet plot, suppressing default plots
    inputs      = outputs.inputs
    objective   = outputs.objective
    constraints = outputs.constraint_val
    plt.figure(0)
    CS   = plt.contourf(inputs[0,:],inputs[1,:], objective, 20, linewidths=2)
    cbar = plt.colorbar(CS)
    
    cbar.ax.set_ylabel('fuel burn (kg)')
    CS_const = plt.contour(inputs[0,:],inputs[1,:], constraints[0,:,:])
    plt.clabel(CS_const, inline=1, fontsize=10)
    cbar = plt.colorbar(CS_const)
    cbar.ax.set_ylabel('fuel margin')
    
    plt.xlabel('Wing Area (m^2)')
    plt.ylabel('Cruise Altitude (km)')
    
    plt.legend(loc='upper left')  
    plt.show(block=True)    
    
    return

if __name__ == '__main__':
    main()