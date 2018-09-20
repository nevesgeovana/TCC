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
# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    problem = setup()
    
    ## Base Input Values
    output = problem.objective()
    
    # # Uncomment to view contours of the design space
    # variable_sweep(problem)

    # ------------------------------------------------------------------
    # DoE
    deltas = [1.1, 0.9]
    original_values = deepcopy(problem.optimization_problem.inputs[:, 1])
    original_bounds = deepcopy(problem.optimization_problem.inputs[:, 2])
    for k, delta in enumerate(deltas):
        for i in range(len(problem.optimization_problem.inputs)-1):
            print ('DoE - parameter study. Run number: '+str(i+(len(problem.optimization_problem.inputs)-1)*k))
            inputs = [1., 1., 1., 1., 1., 1.]
            inputs[i] = delta
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

            print scaled_inputs
            print bounds
            output = scipy_setup.SciPy_Solve(problem, solver='SLSQP')
            print output

            print 'fuel burn   = ', problem.summary.base_mission_fuelburn
            print 'Constraints = ', problem.all_constraints()
            Plot_Mission.plot_mission(problem, i+(len(problem.optimization_problem.inputs)-1)*k)
            print ''
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
        ['wing_area',     72.72,   (72.6, 73.8),    100.,       Units.meter**2],
        ['aspect_ratio',    8.6,     (8.6, 8.6),    10.0,           Units.less],
        ['taper_ratio',  0.3275, (0.3275, 0.3275),  10.0,           Units.less],
        ['t_c_ratio',      0.11,   (0.11, 0.11),      1.,           Units.less],
        ['sweep_angle',    23.0,   (23.0, 23.0),   100.0,            Units.deg],
        ['cruise_range',  1088., (1000., 1200.), 10000.0, Units.nautical_miles],
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
        # ['design_range_fuel_margin', '>', 0., 1E-1, Units.less],  #fuel margin defined here as fuel
        ['fuel_margin',              '>', 0., 1000.,  Units.kg],  # fuel tank volume capacity
        # ['CL',                       '>', 0,  1.,   Units.less],
        ['Throttle_min',             '>', 0., 1.,   Units.less],
        ['Throttle_max',             '>', 0., 1.,   Units.less],
        ['tofl_mtow_margin',         '>', 0., 100.,    Units.m],  # take-off field length
        ['mzfw_consistency',         '>', 0., 1000.,  Units.kg],  # MZFW consistency
        ['design_range_ub',          '>', 0., 1000., Units.nautical_miles],  # Range consistency
        ['design_range_lb',          '>', 0., 1000., Units.nautical_miles],  # Range consistency
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
        # ['fuel_burn',                   'summary.base_mission_fuelburn'                               ],
        # ['design_range_fuel_margin',    'summary.max_zero_fuel_margin'                                ],
        ['fuel_margin',                 'summary.fuel_margin'                                          ],
        # ['CL',                          'summary.CL'],
        ['Throttle_min',                'summary.throttle_min'                                         ],
        ['Throttle_max',                'summary.throttle_max'                                         ],
        ['Nothing',                     'summary.nothing'                                              ],
        ['tofl_mtow_margin',            'summary.takeoff_field_length_margin'                          ],
        ['mzfw_consistency',            'summary.mzfw_consistency'                                     ],
        ['design_range_ub',             'summary.design_range_ub'                                      ],
        ['design_range_lb',             'summary.design_range_lb'                                      ],
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