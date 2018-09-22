# Procedure.py
# 
# Created:  Mar 2016, M. Vegh
# Modified: Aug 2017, E. Botero

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Analyses.Process import Process
from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Propulsion.compute_turbofan_geometry import compute_turbofan_geometry
from SUAVE.Methods.Aerodynamics.Fidelity_Zero.Lift.compute_max_lift_coeff import compute_max_lift_coeff
from SUAVE.Optimization.write_optimization_outputs import write_optimization_outputs
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform import wing_planform

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    procedure = Process()
    procedure.simple_sizing = simple_sizing
    
    # find the weights
    procedure.weights = weight
    # finalizes the data dependencies
    procedure.finalize = finalize

    # calculate field lengths
    procedure.takeoff_field_length = takeoff_field_length
    procedure.landing_field_length = landing_field_length

    # performance studies
    procedure.missions                   = Process()
    procedure.missions.design_mission    = design_mission

    # post process the results
    procedure.post_process = post_process
        
    return procedure

# ----------------------------------------------------------------------        
#   Target Range Function
# ----------------------------------------------------------------------    

def find_target_range(nexus,mission):
    
    segments = mission.segments
    cruise_altitude = mission.segments['climb_5'].altitude_end
    climb_1  = segments['climb_1']
    climb_2  = segments['climb_2']
    climb_3  = segments['climb_3']
    climb_4  = segments['climb_4']
    climb_5  = segments['climb_5']
  
    descent_1 = segments['descent_1']
    descent_2 = segments['descent_2']
    descent_3 = segments['descent_3']

    x_climb_1   = climb_1.altitude_end/np.tan(np.arcsin(climb_1.climb_rate/climb_1.air_speed))
    x_climb_2   = (climb_2.altitude_end-climb_1.altitude_end)/np.tan(np.arcsin(climb_2.climb_rate/climb_2.air_speed))
    x_climb_3   = (climb_3.altitude_end-climb_2.altitude_end)/np.tan(np.arcsin(climb_3.climb_rate/climb_3.air_speed))
    x_climb_4   = (climb_4.altitude_end-climb_3.altitude_end)/np.tan(np.arcsin(climb_4.climb_rate/climb_4.air_speed))
    x_climb_5   = (climb_5.altitude_end-climb_4.altitude_end)/np.tan(np.arcsin(climb_5.climb_rate/climb_5.air_speed))
    x_descent_1 = (climb_5.altitude_end-descent_1.altitude_end)/np.tan(np.arcsin(descent_1.descent_rate/descent_1.air_speed))
    x_descent_2 = (descent_1.altitude_end-descent_2.altitude_end)/np.tan(np.arcsin(descent_2.descent_rate/descent_2.air_speed))
    x_descent_3 = (descent_2.altitude_end-descent_3.altitude_end)/np.tan(np.arcsin(descent_3.descent_rate/descent_3.air_speed))
    
    cruise_range = mission.design_range-(x_climb_1+x_climb_2+x_climb_3+x_climb_4+x_climb_5+x_descent_1+x_descent_2+x_descent_3)
  
    segments['cruise'].distance = cruise_range
    
    return nexus

# ----------------------------------------------------------------------        
#   Design Mission
# ----------------------------------------------------------------------    
def design_mission(nexus):
    
    mission = nexus.missions.base
    # mission.design_range = 1500.*Units.nautical_miles
    # find_target_range(nexus, mission)
    results = nexus.results
    results.base = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Sizing
# ----------------------------------------------------------------------    

def simple_sizing(nexus):
    configs = nexus.vehicle_configurations

    #find conditions
    air_speed   = nexus.missions.base.segments['cruise'].air_speed 
    altitude    = nexus.missions.base.segments['climb_5'].altitude_end
    atmosphere  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    
    freestream  = atmosphere.compute_values(altitude)

    #now size engine
    mach_number        = air_speed/freestream.speed_of_sound

    #now add to freestream data object
    freestream.velocity    = air_speed
    freestream.mach_number = mach_number
    freestream.gravity     = 9.81
    
    conditions             = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()   #assign conditions in form for propulsor sizing
    conditions.freestream  = freestream

    HT_volume = 1.42542 * Units.less # E170
    VT_volume = 0.11458 * Units.less # E170
    lht       =   14.24 * Units.m
    lvt       =   13.54 * Units.m
    for config in configs:
        wing_planform(config.wings.main_wing)

        config.wings.horizontal_stabilizer.areas.reference = (HT_volume/lht)*(config.wings.main_wing.areas.reference *
                                                                              3.194)
        config.wings.vertical_stabilizer.areas.reference   = (VT_volume/lvt)*(config.wings.main_wing.areas.reference *
                                                                              26.0)
        for wing in config.wings:
            wing_planform(wing)
            wing.areas.wetted = 2.0 * wing.areas.reference
            wing.areas.exposed = 0.8 * wing.areas.wetted
            wing.areas.affected = 0.6 * wing.areas.wetted

        turbofan_sizing(config.propulsors['turbofan'], mach_number=mach_number, altitude=altitude)
        compute_turbofan_geometry(config.propulsors['turbofan'], conditions)
        config.propulsors['turbofan'].nacelle_diameter = config.propulsors['turbofan'].nacelle_diameter * 1.1462135
        config.propulsors['turbofan'].engine_length    = config.propulsors['turbofan'].engine_length * 1.24868
        config.propulsors['turbofan'].areas.wetted = 1.1*np.pi * (config.propulsors['turbofan'].engine_length *
                                                                  config.propulsors['turbofan'].nacelle_diameter)

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = nexus.vehicle_configurations.landing
    landing_conditions = Data()
    landing_conditions.freestream = Data()

    # landing weight
    landing.mass_properties.landing = 0.863 * config.mass_properties.takeoff
    
    # Landing CL_max
    altitude           = nexus.missions.base.segments[-1].altitude_end
    atmosphere         = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    freestream_landing = atmosphere.compute_values(altitude)

    landing_conditions.freestream.velocity           = nexus.missions.base.segments['descent_3'].air_speed
    landing_conditions.freestream.density            = freestream_landing.density
    landing_conditions.freestream.dynamic_viscosity  = freestream_landing.dynamic_viscosity

    CL_max_landing,CDi = compute_max_lift_coeff(landing, landing_conditions)
    landing.maximum_lift_coefficient = CL_max_landing
    
    #Takeoff CL_max
    takeoff                       = nexus.vehicle_configurations.takeoff
    takeoff_conditions            = Data()
    takeoff_conditions.freestream = Data()    
    altitude                      = nexus.missions.base.airport.altitude
    freestream_takeoff            = atmosphere.compute_values(altitude)
   
    takeoff_conditions.freestream.velocity           = nexus.missions.base.segments.climb_1.air_speed
    takeoff_conditions.freestream.density            = freestream_takeoff.density
    takeoff_conditions.freestream.dynamic_viscosity  = freestream_takeoff.dynamic_viscosity

    max_CL_takeoff, CDi = compute_max_lift_coeff(takeoff, takeoff_conditions)
    takeoff.maximum_lift_coefficient = max_CL_takeoff

    #Base config CL_max
    base                       = nexus.vehicle_configurations.base
    base_conditions            = Data()
    base_conditions.freestream = takeoff_conditions.freestream

    max_CL_base, CDi = compute_max_lift_coeff(base, base_conditions)
    base.maximum_lift_coefficient = max_CL_base    
    
    return nexus

# ----------------------------------------------------------------------        
#   Check Turbofan Design Thrust
# ----------------------------------------------------------------------    
def check_design_thrust(turbofan, nexus):
    mission = nexus.missions.base
    segments = mission.segments
    TOW = 37000 * 9.81
    altitude = []
    speed    = []
    
    for i in range(5):
        altitude.append(segments['climb_'+str(i+1)].altitude_end)
        speed.append(segments['climb_'+str(i+1)].air_speed)
        
    # define reference condition for parasite drag
    ref_condition = Data()
    ref_condition.mach_number = 0.3
    ref_condition.reynolds_number = 12e6
    
    
    results     = turbofan.evaluate_thrust()
    thrust_ref  = results.thrust_force_vector[0, 0]
    
    
    return

# ----------------------------------------------------------------------        
#   Check Cruise Altitude
# ----------------------------------------------------------------------    
    
def check_cruise_altitude(analyses):
    
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_dummy_mission'

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere =  SUAVE.Analyses.Atmospheric.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    atmosphere=SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    planet = SUAVE.Attributes.Planets.Earth()
    base_segment.state.numerics.number_control_points = 35
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(base_segment)
    segment.tag = "climb_dummy"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # define segment attributes
    segment.atmosphere     = atmosphere
    segment.planet         = planet

    segment.altitude_start       = 0.0    * Units.ft
    segment.altitude_end         = 35000. * Units.ft
    segment.equivalent_air_speed = 265.0  * Units.knots
    segment.climb_rate           = 300.   * Units.['ft/min']
    
    # add to mission
    mission.append_segment(segment)
    
    results = mission.evaluate()
    cruise_altitude = 35000 * Units.ft
    for segment in results.segments.values():
        altitude = segment.conditions.freestream.altitude[:,0] / Units.ft
        eta    = segment.conditions.propulsion.throttle[:,0]
        for i in range(len(altitude)):
            if eta[i] > 1.:
                cruise_altitude = altitude[i-1] * Units.ft
            elif eta[i] < 1. and i == len(altitude)-1:
                cruise_altitude = altitude[i] * Units.ft
    
    return
    
    
# ----------------------------------------------------------------------        
#   Weights
# ----------------------------------------------------------------------    

def weight(nexus):
    vehicle = nexus.vehicle_configurations.base

    # weight analysis
    weights = nexus.analyses.base.weights.evaluate()
    weights = nexus.analyses.landing.weights.evaluate()
    weights = nexus.analyses.takeoff.weights.evaluate()
    weights = nexus.analyses.short_field_takeoff.weights.evaluate()
    weights = nexus.analyses.cruise.weights.evaluate()

    vehicle.mass_properties.breakdown = weights

    empty_weight     = vehicle.mass_properties.operating_empty
    passenger_weight = vehicle.passenger_weights.mass_properties.mass 

    for config in nexus.vehicle_configurations:
        config.mass_properties.zero_fuel_center_of_gravity  = vehicle.mass_properties.zero_fuel_center_of_gravity
        config.fuel                                         = vehicle.fuel
       
    return nexus
# ----------------------------------------------------------------------
#   Takeoff Field Length Evaluation
# ----------------------------------------------------------------------

def takeoff_field_length(nexus):

    # import tofl analysis module
    estimate_tofl = SUAVE.Methods.Performance.estimate_take_off_field_length

    # unpack data
    results  = nexus.results
    summary  = nexus.summary
    analyses = nexus.analyses
    missions = nexus.missions
    config   = nexus.vehicle_configurations.takeoff

    # defining required data for tofl evaluation
    takeoff_airport = missions.base.airport
    ref_weight      = config.mass_properties.takeoff
    config.mass_properties.takeoff = 38600.
    takeoff_field_length, second_segment_climb_gradient_takeoff = estimate_tofl(config, analyses, takeoff_airport, 1)

    # pack results
    summary.takeoff_field_length = takeoff_field_length
    summary.second_segment_climb_gradient_takeoff = second_segment_climb_gradient_takeoff


    return nexus

# ----------------------------------------------------------------------
#   landing Field Length Evaluation
# ----------------------------------------------------------------------

def landing_field_length(nexus):

    # import tofl analysis module
    estimate_landing = SUAVE.Methods.Performance.estimate_landing_field_length

    # unpack data
    results  = nexus.results
    summary = nexus.summary
    analyses = nexus.analyses
    missions = nexus.missions
    config   = nexus.vehicle_configurations.landing

    # defining required data for tofl evaluation
    landing_airport = missions.base.airport
    ref_weight      = config.mass_properties.landing

    landing_field_length = estimate_landing(config, analyses, landing_airport)

    # pack results
    summary.landing_field_length = landing_field_length

    return nexus

# ----------------------------------------------------------------------
#   Finalizing Function
# ----------------------------------------------------------------------    

def finalize(nexus):
    
    nexus.analyses.finalize()   
    
    return nexus         

# ----------------------------------------------------------------------
#   Post Process Results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    # Unpack data
    vehicle                           = nexus.vehicle_configurations.base
    results                           = nexus.results
    summary                           = nexus.summary
    missions                          = nexus.missions  
    nexus.total_number_of_iterations +=1
    # Static stability calculations
    CMA = -10.
    for segment in results.base.segments.values():
        max_CMA = np.max(segment.conditions.stability.static.cm_alpha[:, 0])
        if max_CMA > CMA:
            CMA = max_CMA
            
    summary.static_stability = CMA
    
    #throttle in design mission
    max_throttle = 0.
    for segment in results.base.segments.values():
        max_segment_throttle = np.max(segment.conditions.propulsion.throttle[:, 0])
        if max_segment_throttle > max_throttle:
            max_throttle = max_segment_throttle
    min_throttle = 1.
    for segment in results.base.segments.values():
        min_segment_throttle = np.min(segment.conditions.propulsion.throttle[:, 0])
        if min_segment_throttle < min_throttle:
            min_throttle = min_segment_throttle

    summary.throttle_max = 1.00000001 - max_throttle  # strategy to make constraint > 0
    summary.throttle_min = min_throttle

    # Fuel margin and base fuel calculations
    operating_empty          = vehicle.mass_properties.operating_empty
    max_payload              = vehicle.mass_properties.max_payload
    payload                  = vehicle.passenger_weights.mass_properties.mass 
    design_landing_weight    = results.base.segments[-1].conditions.weights.total_mass[-1]
    design_takeoff_weight    = vehicle.mass_properties.takeoff
    max_takeoff_weight       = nexus.vehicle_configurations.takeoff.mass_properties.max_takeoff
    zero_fuel_weight         = vehicle.mass_properties.max_zero_fuel
    
    # summary.max_zero_fuel_margin    = (design_landing_weight - zero_fuel_weight)/zero_fuel_weight
    summary.base_mission_fuelburn = design_takeoff_weight - design_landing_weight
    summary.fuel_margin           = design_landing_weight - operating_empty - payload
    summary.mzfw_consistency = zero_fuel_weight - operating_empty - payload

    summary.takeoff_field_length_margin = 1644. - summary.takeoff_field_length
    summary.nothing = 0.0

    summary.design_range_ub = 1341. - (results.base.segments['descent_3'].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile)
    summary.design_range_lb = -1339. + (results.base.segments['descent_3'].conditions.frames.inertial.position_vector[-1, 0] *
                                       Units.m/Units.nautical_mile)
    #when you run want to output results to a file
    # filename = 'results.txt'
    # write_optimization_outputs(nexus, filename)

    return nexus    
