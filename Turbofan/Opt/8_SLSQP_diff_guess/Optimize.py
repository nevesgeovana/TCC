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
import matplotlib
from SUAVE.Optimization import Nexus, carpet_plot
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
import SUAVE.Optimization.Package_Setups.pyopt_setup as pyopt_setup
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

    print ' '
    print ' Initial Guess Results '
    print ' '
    print 'Fuel Burn   = ', float(problem.summary.base_mission_fuelburn)
    print 'Cruise Fuel = ', float(problem.summary.cruise_fuel)
    print 'Block  Fuel = ', float(problem.summary.block_fuel)
    print 'MTOW        = ', float(problem.summary.MTOW)
    print 'BOW         = ', float(problem.summary.BOW)
    print 'TOFL        = ', float(problem.summary.takeoff_field_length)
    print 'GRAD        = ', float(problem.summary.second_segment_climb_gradient_takeoff)
    print 'Cruise Alt  = ', float(problem.summary.cruise_altitude)
    print 'Design Ran  = ', float(problem.summary.design_range)
    print 'Cruise Ran  = ', float(problem.summary.cruise_range)
    print 'Total Ran   = ', float(problem.summary.total_range)
    print 'Time To Cli = ', float(problem.summary.time_to_climb_value)
    print 'TOW HH      = ', float(problem.summary.TOW_HH)
    print 'Fuel HH     = ', float(problem.summary.FUEL_HH)
    print ' '
    print 'Constraints = ', problem.all_constraints()
    print ' '
    last_inputs = problem.last_inputs[:, 1]
    print 'S           = ', last_inputs[0]
    print 'AR          = ', last_inputs[1]
    print 't/c         = ', last_inputs[2]
    print 'Sweep Ang   = ', last_inputs[3]
    # ------------------------------------------------------------------
    # Pareto

    fuelburn = []
    allconstraints = []
    MTOW = []
    finalvalue = []
    grad = []
    tofl = []

    betas = [1., 0.75, 0.5, 0.25, 0.]
    # betas = [0., 0.25, 0.5, 0.75, 1.]
    # betas = [1.]

    fid = open('Pareto_results.txt', 'w')  # Open output file
    fid.write(' Pareto results \n')
    fid.close()

    for i, beta in enumerate(betas):
        fid = open('Pareto_results.txt', 'ab')  # Open output file
        fid.write('Pareto Frontier. Run number: ' + str(i) + ' \n')
        print('Pareto Frontier. Run number: ' + str(i))

        # updating beta value
        problem.beta = beta

        output = scipy_setup.SciPy_Solve(problem, solver='SLSQP', sense_step=1.4901161193847656e-05)
        # output = pyopt_setup.Pyopt_Solve(problem, solver='ALPSO')

        finalvalue.append(output)

        print output
        print ' '
        print ' Final Results '
        print ' '
        print 'Fuel Burn   = ', float(problem.summary.base_mission_fuelburn)
        print 'Cruise Fuel = ', float(problem.summary.cruise_fuel)
        print 'Block  Fuel = ', float(problem.summary.block_fuel)
        print 'MTOW        = ', float(problem.summary.MTOW)
        print 'BOW         = ', float(problem.summary.BOW)
        print 'TOFL        = ', float(problem.summary.takeoff_field_length)
        print 'GRAD        = ', float(problem.summary.second_segment_climb_gradient_takeoff)
        print 'Cruise Alt  = ', float(problem.summary.cruise_altitude)
        print 'Design Ran  = ', float(problem.summary.design_range)
        print 'Cruise Ran  = ', float(problem.summary.cruise_range)
        print 'Total Ran   = ', float(problem.summary.total_range)
        print 'Time To Cli = ', float(problem.summary.time_to_climb_value)
        print 'TOW HH      = ', float(problem.summary.TOW_HH)
        print 'Fuel HH     = ', float(problem.summary.FUEL_HH)
        print ' '
        print 'Constraints = ', problem.all_constraints()
        print ' '
        last_inputs = finalvalue[-1]
        print 'S           = ', last_inputs[0]*75.
        print 'AR          = ', last_inputs[1]*9.8
        print 't/c         = ', last_inputs[2]*0.13
        print 'Sweep Ang   = ', last_inputs[3]*30.0
        # -----------------------------------------
        Plot_Mission.plot_mission(problem, i)

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
        fid.close()

        if i < len(betas)-1:
            print(' ')
            print('-------------------------------------------------------')
            print('Setting up initial values again')
            print(' ')
            problem = setup()

            # Base Input Values
            output = problem.objective()
            print output
            print(' ')
            print('-------------------------------------------------------')
            print(' ')

    fid = open('Pareto_results.txt', 'ab')  # Open output file
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
        ['wing_area',      72.72*0.95,  (65.0, 80.0),       75.,       Units.meter**2],
        ['aspect_ratio',     8.6*0.95,    (7.6, 9.8),       9.8,           Units.less],
        ['t_c_ratio',       0.11*0.95,  (0.09, 0.15),      0.13,           Units.less],
        ['sweep_angle',      23.*0.95,  (18.0, 30.0),      30.0,            Units.deg],
        ['cruise_range',   1103., (1000., 1160.),   1130.0, Units.nautical_miles],
        ['MTOW',          37200., (35000., 39000.), 37200.,             Units.kg],
        ['MZFW_ratio',  0.810215, (0.7, 0.9),          0.9,           Units.less],
    ])

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # throw an error if the user isn't specific about wildcards
    # [ tag, scaling, units ]
    problem.objective = np.array([
        # ['fuel_burn', 10000., Units.kg],
        ['objective', 1., Units.less],
    ])
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    # CONSTRAINTS ARE SET TO BE BIGGER THAN ZERO, SEE PROCEDURE (SciPy's SLSQP optimization algorithm assumes this form)
    problem.constraints = np.array([
        ['mzfw_consistency',         '=', 0., 100.,   Units.kg],           # MZFW consistency
        ['fuel_margin',              '=', 0., 100.,   Units.kg],           # fuel margin defined here as fuel
        ['Throttle_min',             '>', 0., 1.,   Units.less],
        ['Throttle_max',             '>', 0., 1.,   Units.less],
        ['tofl_mtow_margin',         '>', 0., 100.,    Units.m],           # take-off field length
        ['design_range_ub',          '>', 0., 10., Units.nautical_miles],  # Range consistency
        ['design_range_lb',          '>', 0., 10., Units.nautical_miles],  # Range consistency
        ['time_to_climb',            '>', 0., 10.,  Units.min],            # Time to climb consistency
        ['climb_gradient',           '>', 0., 1.,  Units.less],            # second segment climb gradient
        ['lfl_mlw_margin',           '>', 0., 100.,   Units.m],            # landing field length
        ['max_fuel_margin',          '>', 0., 100.,  Units.kg],            # max fuel margin
        ['TOW_HH_margin',            '>', 0., 100.,  Units.kg],            # TOW for Hot and High
    ])
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        ['wing_area',                  ['vehicle_configurations.*.wings.main_wing.areas.reference',
                                        'vehicle_configurations.*.reference_area'                     ]],
        ['aspect_ratio',                'vehicle_configurations.*.wings.main_wing.aspect_ratio'        ],
        ['t_c_ratio',                   'vehicle_configurations.*.wings.main_wing.thickness_to_chord'  ],
        ['sweep_angle',                 'vehicle_configurations.*.wings.main_wing.sweeps.quarter_chord'],
        ['cruise_range',                'missions.base.segments.cruise.distance'                       ],
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
        ['TOW_HH_margin',               'summary.TOW_HH_margin'                                        ],
        ['MTOW',                       ['vehicle_configurations.*.mass_properties.max_takeoff',
                                        'vehicle_configurations.*.mass_properties.takeoff'             ]],
        ['MZFW_ratio',                  'summary.MZFW_ratio'                                            ],
        ['beta',                        'vehicle_configurations.base.wings.main_wing.beta'              ],
        ['objective',                   'summary.objective'                                             ],
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

    nexus.beta = Data()
    nexus.beta = 1.
    return nexus
    
def variable_sweep(problem):
    from matplotlib import rcParams
    rcParams['font.family'] = 'times new roman'
    # rcParams['font.times-new-roman'] = ['times new roman']

    number_of_points = 5
    outputs     = carpet_plot(problem, number_of_points, 0, 0)  #run carpet plot, suppressing default plots
    inputs      = outputs.inputs
    objective   = outputs.objective
    constraints = outputs.constraint_val
    plt.figure(0)
    CS   = plt.contourf(inputs[0,:],inputs[1,:], objective, 20, linewidths=2)
    cbar = plt.colorbar(CS)
    
    cbar.ax.set_ylabel('Fuel Burn (kg)')
    CS_const = plt.contour(inputs[0,:],inputs[1,:], constraints[-1,:,:],cmap=plt.get_cmap('hot'))
    plt.clabel(CS_const, inline=1, fontsize=12, family='times new roman')
    cbar = plt.colorbar(CS_const)
    # plt.FontProperties(family='times new roman', style='italic', size=12)
    cbar.ax.set_ylabel('BOW (kg)')
    # font = matplotlib.font_manager.FontProperties(family='times new roman', style='italic', size=12)

    # CS_const.font_manager.FontProperties.set_family(family='times new roman')

    plt.xlabel('Wing Area (m^2)')
    plt.ylabel('Aspect Ratio (-)')

    plt.legend(loc='upper left')  
    # plt.show(block=True)
    plt.show()

    number_of_points = 5
    outputs = carpet_plot(problem, number_of_points, 0, 0, sweep_index_0=1, sweep_index_1=3)  # run carpet plot, suppressing default plots
    inputs = outputs.inputs
    objective = outputs.objective
    constraints = outputs.constraint_val
    plt.figure(0)
    CS = plt.contourf(inputs[0, :], inputs[1, :], objective, 20, linewidths=2)
    cbar = plt.colorbar(CS)

    cbar.ax.set_ylabel('Fuel Burn (kg)')
    CS_const = plt.contour(inputs[0, :], inputs[1, :], constraints[-1, :, :], cmap=plt.get_cmap('hot'))
    plt.clabel(CS_const, inline=1, fontsize=10)
    cbar = plt.colorbar(CS_const)
    cbar.ax.set_ylabel('BOW (kg)')

    plt.xlabel('AR (-)')
    plt.ylabel('Sweep Angle (Deg)')

    plt.legend(loc='upper left')
    plt.show()

    number_of_points = 5
    outputs = carpet_plot(problem, number_of_points, 0, 0, sweep_index_0=2,
                          sweep_index_1=3)  # run carpet plot, suppressing default plots
    inputs = outputs.inputs
    objective = outputs.objective
    constraints = outputs.constraint_val
    plt.figure(0)
    CS = plt.contourf(inputs[0, :], inputs[1, :], objective, 20, linewidths=2)
    cbar = plt.colorbar(CS)

    cbar.ax.set_ylabel('Fuel Burn (kg)')
    CS_const = plt.contour(inputs[0, :], inputs[1, :], constraints[-1, :, :], cmap=plt.get_cmap('hot'))
    plt.clabel(CS_const, inline=1, fontsize=10)
    cbar = plt.colorbar(CS_const)
    cbar.ax.set_ylabel('BOW (kg)')

    plt.xlabel('t/c (-)')
    plt.ylabel('Sweep Angle (Deg)')

    plt.legend(loc='upper left')
    plt.show(block=True)

    return

if __name__ == '__main__':
    main()