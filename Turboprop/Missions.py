# Missions.py
# 
# Created:  Mar 2016, M. Vegh
# Modified: Aug 2017, E. Botero

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Data, Units
from copy import deepcopy
import numpy as np

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
    
def setup(analyses):
    
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------
    base_mission = base(analyses)
    missions.base = base_mission 
 
    return missions  
    
def base(analyses):
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row = base_segment.state.ones_row
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position = SUAVE.Methods.skip
    base_segment.process.iterate.unknowns.network = analyses.vehicle.propulsors.turboprop.unpack_unknowns
    base_segment.process.iterate.residuals.network = analyses.vehicle.propulsors.turboprop.residuals
    base_segment.state.unknowns.pitch_command = ones_row(1) * 0. * Units.deg
    base_segment.state.conditions.propulsion = Data()
    base_segment.state.conditions.propulsion.rpm = 1200 * ones_row(1)
    base_segment.state.residuals.net = 0. * ones_row(1)
    base_segment.state.numerics.number_control_points = 10
    base_segment.state.conditions.freestream = Data()
    base_segment.state.conditions.freestream.delta_ISA = airport.delta_isa * ones_row(1)

    climb_segment = deepcopy(base_segment)
    base_segment.state.unknowns.throttle = ones_row(1)
    climb_segment.state.numerics.number_control_points = 3

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_1"

    segment.analyses.extend( analyses.takeoff )

    segment.altitude_start       = 0.0    * Units.ft
    segment.altitude_end         = 1500.0 * Units.ft
    segment.equivalent_air_speed = 170.0  * Units.knots
    segment.climb_rate           = 1280.0 * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_2"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = 1500.0 * Units.ft
    segment.altitude_end   = 4000.0 * Units.ft
    segment.equivalent_air_speed = 170.0  * Units.knots
    segment.climb_rate           = 1200.0 * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_3"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = 4000.0 * Units.ft
    segment.altitude_end   = 7000.0 * Units.ft
    segment.equivalent_air_speed = 170.0  * Units.knots
    segment.climb_rate           = 1050.0 * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fourth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_4"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = 7000.0 * Units.ft
    segment.altitude_end   = 10000.0 * Units.ft
    segment.equivalent_air_speed = 170.0  * Units.knots
    segment.climb_rate           = 870.0 * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)
    # ------------------------------------------------------------------
    #   Fifth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_5"

    segment.analyses.extend(analyses.takeoff)

    segment.altitude_start = 10000.0 * Units.ft
    segment.altitude_end   = 12000.0 * Units.ft
    segment.equivalent_air_speed = 170.0  * Units.knots
    segment.climb_rate           = 745.0 * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Sixth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_6"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = 12000.0 * Units.ft
    segment.altitude_end   = 14000.0 * Units.ft
    segment.equivalent_air_speed = 170.0  * Units.knots
    segment.climb_rate           = 625. * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Seventh Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_7"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = 14000.0 * Units.ft
    segment.altitude_end   = 16000.0 * Units.ft
    segment.equivalent_air_speed = 170.0  * Units.knots
    segment.climb_rate           = 500.0 * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Eighth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_8"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = 16000.0 * Units.ft
    segment.altitude_end   = 18000.0 * Units.ft
    segment.equivalent_air_speed = 170.0  * Units.knots
    segment.climb_rate           = 375.0 * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Ninth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_9"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = 18000.0 * Units.ft
    segment.altitude_end   = 19000.0 * Units.ft
    segment.equivalent_air_speed = 170.0 * Units.knots
    segment.climb_rate           = 300.0 * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Tenth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_EAS_Constant_Rate(climb_segment)
    segment.tag = "climb_10"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = 19000.0 * Units.ft
    segment.altitude_end = 21000.0 * Units.ft
    segment.equivalent_air_speed = 167. * Units.knots
    segment.climb_rate = 300.0 * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend(analyses.cruise)

    segment.air_speed  = 260. * Units.knots
    segment.distance   = 331.3042 * Units.nautical_miles

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCR'

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_end = 18000. * Units.ft
    segment.air_speed    = 290.0  * Units.knots
    segment.descent_rate = 2000. * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCR'
    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_end = 10000.     * Units.ft
    segment.air_speed    = 275.0  * Units.knots
    segment.descent_rate = 2000.  * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCR'
    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3"

    segment.analyses.extend(analyses.landing)

    segment.altitude_end = 0.     * Units.ft
    segment.air_speed    = 260.0  * Units.knots
    segment.descent_rate = 2000.  * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCR'
    # add to mission
    mission.append_segment(segment)
    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    #         Reserve mission
    # ------------------------------------------------------------------
    planet = SUAVE.Attributes.Planets.Earth()
    atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Throttle
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "reserve_climb1"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # define segment attributes
    segment.atmosphere = atmosphere
    segment.planet = planet

    segment.altitude_start = 0.0   * Units.ft
    segment.altitude_end   = 5000. * Units.ft
    segment.air_speed      = 180.0 * Units.knots
    segment.climb_rate     = 1100. * Units['ft/min']
    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Throttle
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "reserve_climb2"

    # connect vehicle configuration
    segment.analyses.extend(analyses.base)

    # define segment attributes
    segment.atmosphere = atmosphere
    segment.planet = planet

    segment.altitude_end   = 10000. * Units.ft
    segment.air_speed      = 190.0  * Units.knots
    segment.climb_rate     = 900.   * Units['ft/min']
    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------
    
    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "reserve_cruise"
    
    segment.analyses.extend( analyses.cruise )
    
    segment.mach      = 0.38
    segment.distance  = 34.0 * Units.nautical_mile
    segment.state.conditions.propulsion.gas_turbine_rating = 'MCR'
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Loiter Segment: constant mach, constant time
    # ------------------------------------------------------------------
    
    segment = Segments.Cruise.Constant_Mach_Constant_Altitude_Loiter(base_segment)
    segment.tag = "reserve_loiter"
    
    segment.analyses.extend( analyses.cruise )
    
    segment.mach = 0.38
    segment.time = 45.0 * Units.minutes
    segment.state.conditions.propulsion.gas_turbine_rating = 'MCR'
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------
    #  Final Descent Segment: consant speed, constant segment rate
    # ------------------------------------------------------------------
    
    segment = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "reserve_descent_1"
    
    segment.analyses.extend( analyses.landing )
    
    segment.altitude_end = 0.0  * Units.ft
    segment.descent_rate = 800. * Units['ft/min']
    segment.mach_end     = 0.2
    segment.mach_start   = 0.3
    segment.state.conditions.propulsion.gas_turbine_rating = 'MCR'
    # append to mission
    mission.append_segment(segment)
    
    #------------------------------------------------------------------
    ###         Reserve mission completed
    #------------------------------------------------------------------
    
    return mission

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    import vehicles
    import analyses
    
    vehicles = vehicles.setup()
    analyses = analyses.setup(vehicles)
    missions = setup(analyses)
    
    vehicles.finalize()
    analyses.finalize()
    missions.finalize()
    
    missions.base.evaluate()