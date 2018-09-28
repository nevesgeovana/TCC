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
    variations = []

    fid = open('DoE_results.txt', 'w')  # Open output file
    fid.write(' DoE results \n')
    fid.close()

    deltas = [0.95, 1.05]
    original_values = deepcopy(problem.optimization_problem.inputs[:, 1])
    original_bounds = deepcopy(problem.optimization_problem.inputs[:, 2])
    for k, delta in enumerate(deltas):
        for i in range(len(problem.optimization_problem.inputs)-2):
            fid = open('DoE_results.txt', 'ab')  # Open output file
            fid.write('DoE - parameter study. Run number: '+str(i+(len(problem.optimization_problem.inputs)-2)*k)+' \n')
            print ('DoE - parameter study. Run number: '+str(i+(len(problem.optimization_problem.inputs)-2)*k))
            inputs = [1., 1., 1., 1., 1., 1., 1.]
            inputs[i] = delta
            variations.append(inputs)
            print ('Scaling Inputs: '+str(inputs))

            # reset values
            problem.optimization_problem.inputs[:, 1] = deepcopy(original_values)
            problem.optimization_problem.inputs[:, 2] = deepcopy(original_bounds)

            # changing parameters values
            scaling = problem.optimization_problem.inputs[:, 1]
            scaled_inputs = np.multiply(inputs, scaling)
            problem.optimization_problem.inputs[:, 1] = scaled_inputs

            # changing parameters bounds
            bounds = problem.optimization_problem.inputs[:, 2]
            bounds[i] = list(bounds[i])
            bounds[i][0] = bounds[i][0] * inputs[i]
            bounds[i][1] = bounds[i][1] * inputs[i]
            bounds[i] = tuple(bounds[i])
            problem.optimization_problem.inputs[:, 2] = bounds

            output = scipy_setup.SciPy_Solve(problem, solver='SLSQP')
            print output
            finalvalue.append(output)
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
            last_inputs = finalvalue[-1]
            print 'S           = ', last_inputs[0]
            print 'AR          = ', last_inputs[1]
            print 't/c         = ', last_inputs[2]
            print 'Sweep Ang   = ', last_inputs[3]
            Plot_Mission.plot_mission(problem, i+(len(problem.optimization_problem.inputs)-2)*k)

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
            fid.write(str(variations[-1]) + ' \n')
            fid.write(str(finalvalue[-1]) + ' \n')
            fid.write('\n \n')
            fid.close()

    fid = open('DoE_results.txt', 'ab')  # Open output file
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
        ['wing_area',      61., (61.0, 61.0),   100., Units.meter**2],
        ['aspect_ratio',   12.,   (12., 12.),  100.0,     Units.less],
        ['t_c_ratio',     0.15, (0.15, 0.15),     1.,     Units.less],
        ['sweep_angle',     3.,   (3.,  3.0),  100.0,      Units.deg],
        ['taper_ratio',   0.53, (0.53, 0.53),     1.,     Units.less],
        ['cruise_range', 331.3, (300., 350.), 1000.0, Units.nautical_miles],
        ['delta_fuel',     8.8, (0., 250.),   1000.0,       Units.kg],  # MTOW consistency
        # ['beta',            1., (1., 1.), 1.,                   Units.less],
    ])

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # throw an error if the user isn't specific about wildcards
    # [ tag, scaling, units ]
    problem.objective = np.array([
        ['Nothing', 1., Units.kg],
    ])
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    # CONSTRAINTS ARE SET TO BE BIGGER THAN ZERO, SEE PROCEDURE (SciPy's SLSQP optimization algorithm assumes this form)
    problem.constraints = np.array([
        # ['design_range_margin',      '=', 0., 100., Units.nautical_miles],  # Range consistency
        ['fuel_margin_ub',              '>', 0., 1000.,  Units.kg],   # MTOW consistency
        ['fuel_margin_lb',              '>', 0., 1000.,  Units.kg],   # MTOW consistency
        ['Throttle_min',             '>', 0., 1.,   Units.less],
        # ['Throttle_max',             '>', 0., 1.,   Units.less],
        # ['tofl_mtow_margin',         '>', 0., 100.,    Units.m],  # take-off field length
        # ['mzfw_consistency',         '>', 0., 1000.,  Units.kg],  # MZFW consistency
        ['design_range_ub',          '>', 0., 1., Units.nautical_miles],  # Range consistency
        ['design_range_lb',          '>', 0., 1., Units.nautical_miles],  # Range consistency
        # ['time_to_climb',            '>', 0., 10.,  Units.min],  # Time to climb consistency
        # ['climb_gradient',           '>', 0., 1.,  Units.less],  # second segment climb gradient
        # ['lfl_mlw_margin',           '>', 0., 100.,   Units.m],  # landing field length
        # ['max_fuel_margin',          '>', 0., 1000., Units.kg],  # max fuel margin
        # ['range_HH_margin',          '>', 0., 1000., Units.nautical_miles],  # Range for Hot and High
        # ['TOW_HH_margin',            '>', 0., 1000., Units.kg],  # TOW for Hot and High
        # ['MTOW',                   '>', 0., 100000., Units.kg],  # TOW for Hot and High
        # ['BOW',                    '>', 0., 1., Units.kg],  # TOW for Hot and High
    ])
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        ['wing_area',                  ['vehicle_configurations.*.wings.main_wing.areas.reference',
                                        'vehicle_configurations.*.reference_area'                     ]],
        ['aspect_ratio',                'vehicle_configurations.*.wings.main_wing.aspect_ratio'        ],
        ['taper_ratio',                 'vehicle_configurations.*.wings.main_wing.taper'               ],
        ['t_c_ratio',                   'vehicle_configurations.*.wings.main_wing.thickness_to_chord'  ],
        ['sweep_angle',                 'vehicle_configurations.*.wings.main_wing.sweeps.quarter_chord'],
        ['cruise_range',                'missions.base.segments.cruise.distance'                       ],
        ['delta_fuel',                  'vehicle_configurations.base.delta_fuel'                       ],
        ['Nothing',                     'summary.nothing'                                              ],
        # ['fuel_burn',                   'summary.base_mission_fuelburn'                                ],
        ['fuel_margin_ub',              'summary.fuel_margin_ub'                                       ],
        ['fuel_margin_lb',              'summary.fuel_margin_lb'                                       ],
        ['Throttle_min',                'summary.throttle_min'                                         ],
        ['Throttle_max',                'summary.throttle_max'                                         ],
        ['tofl_mtow_margin',            'summary.takeoff_field_length_margin'                          ],
        ['mzfw_consistency',            'summary.mzfw_consistency'                                     ],
        # ['design_range_margin',         'summary.design_range_margin'],
        ['design_range_ub',             'summary.design_range_ub'                                      ],
        ['design_range_lb',             'summary.design_range_lb'                                      ],
        ['time_to_climb',               'summary.time_to_climb'                                        ],
        ['climb_gradient',              'summary.climb_gradient'                                       ],
        ['lfl_mlw_margin',              'summary.lfl_mlw_margin'                                       ],
        ['max_fuel_margin',             'summary.max_fuel_margin'                                      ],
        # ['range_HH_margin',             'summary.range_HH_margin'],
        ['TOW_HH_margin',               'summary.TOW_HH_margin'],
        # ['MTOW',                        'summary.MTOW'],
        # ['BOW',                         'summary.BOW'],
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
    nexus.analyses.vehicle = Data()
    nexus.analyses.vehicle = nexus.vehicle_configurations.base
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