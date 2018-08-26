# tut_mission_ATR72.py
# 
# Created:  Aug 2014, SUAVE Team
# Modified: Aug 2017, SUAVE Team

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# Python Imports
import numpy as np
import pylab as plt

# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Methods.Propulsion.propeller_design import propeller_design
from SUAVE.Input_Output.Results import  print_parasite_drag,  \
     print_compress_drag, \
     print_engine_data,   \
     print_mission_breakdown, \
     print_weight_breakdown
from SUAVE.Methods.Performance  import payload_range
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform import wing_planform

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    # ---------------------------------------------------------------------------------------
    # INITIALIZING AIRCRAFT

    configs, analyses, vehicle = full_setup()
    # analyses.configs.cruise.aerodynamics.settings.drag_coefficient_increment = 0.001

    simple_sizing(configs)

    configs.finalize()
    analyses.finalize()

    # ---------------------------------------------------------------------------------------
    # WEIGHT ANALYSIS
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()

    # ---------------------------------------------------------------------------------------
    # MISSION ANALYSIS
    mission = analyses.missions.base
    results = mission.evaluate()
    configs.cruise.conditions = Data()
    configs.cruise.conditions = results.segments.cruise.conditions
    # print weight breakdown
    print_weight_breakdown(configs.base,filename = 'ATR72_weight_breakdown.dat')

    # print parasite drag data into file - define reference condition for parasite drag
    ref_condition = Data()
    ref_condition.mach_number = 0.3
    ref_condition.reynolds_number = 12e6
    print_parasite_drag(ref_condition,configs.cruise,analyses,'ATR72_parasite_drag.dat')

    # print compressibility drag data into file
    print_compress_drag(configs.cruise,analyses,filename = 'ATR72_compress_drag.dat')

    # print mission breakdown
    print_mission_breakdown(results,filename='ATR72_mission_breakdown.dat')

    # print engine data into file
    print_engine_data(configs.cruise,filename = 'ATR72_engine_data.dat')

    state = Data()
    state.conditions = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()
    state.numerics = SUAVE.Analyses.Mission.Segments.Conditions.Numerics()

    # ---------------------------------------------------------------------------------------
    # PAYLOAD RANGE DIAGRAM

    config = configs.base
    cruise_segment_tag = "cruise"
    reserves = 1600.
    # payload_range_results = payload_range(config, mission, cruise_segment_tag, reserves)

    # ---------------------------------------------------------------------------------------
    # PLOT RESULTS

    plot_mission(results)

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)
    configs_analyses.vehicle = Data()
    configs_analyses.vehicle = vehicle

    # mission analyses
    mission  = mission_setup(configs_analyses)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses, vehicle

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Tube_Wing()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    return analyses    

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'ATR72'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff               = 22800. * Units.kg
    vehicle.mass_properties.operating_empty           = 13800. * Units.kg
    vehicle.mass_properties.takeoff                   = 22800. * Units.kg
    vehicle.mass_properties.max_zero_fuel             = 20800. * Units.kg
    vehicle.mass_properties.cargo                     = 0.0    * Units.kg
    vehicle.mass_properties.max_payload               = 7300.0 * Units.kg
    vehicle.mass_properties.max_fuel                  = 5000.0* Units.kg

    vehicle.mass_properties.center_of_gravity = [11.760, 0., 0.]

    # envelope properties
    vehicle.envelope.ultimate_load = 2.5 * 1.5
    vehicle.envelope.limit_load    = 2.5
    vehicle.maximum_mach_operational = 0.55

    # basic parameters
    vehicle.reference_area         = 61 * Units['meters**2']
    vehicle.passengers             = 72
    vehicle.systems.control        = "fully powered"
    vehicle.systems.accessories    = "short range"



    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 12
    wing.sweeps.quarter_chord    = 3.00 * Units.deg
    wing.thickness_to_chord      = 0.15
    wing.taper                   = 0.53
    wing.span_efficiency         = 1.
    wing.spans.projected         = 27.05 * Units.meter
    wing.chords.root             = 2.942 * Units.meter
    wing.chords.tip              = 1.568 * Units.meter
    wing.chords.mean_aerodynamic = 2.325 * Units.meter
    wing.areas.reference         = 61.00 * Units['meters**2']
    wing.areas.wetted            = 2.0   * wing.areas.reference
    wing.areas.exposed           = 0.8   * wing.areas.wetted
    wing.twists.root             = 2.0   * Units.degrees
    wing.twists.tip              = 0.0   * Units.degrees
    wing.origin                  = [11.0946, 0, 0]
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    wing.flaps.type              = "single_slotted"
    wing.flaps.chord             = 0.250
    wing.dynamic_pressure_ratio  = 1.0
    wing.flaps.span_start        = 0.11
    wing.flaps.span_end          = 0.73

    wing_planform(wing)

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 4.54
    wing.sweeps.quarter_chord    = 4.75 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.65
    wing.span_efficiency         = 0.9
    wing.spans.projected         = 7.3347 * Units.meter
    wing.chords.root             = 1.960  * Units.meter
    wing.chords.tip              = 1.273  * Units.meter
    wing.chords.mean_aerodynamic = 1.6413 * Units.meter
    wing.areas.reference         = 11.86  * Units['meters**2']
    wing.areas.wetted            = 2.0    * wing.areas.reference
    wing.twists.root             = 2.0    * Units.degrees
    wing.twists.tip              = 2.0    * Units.degrees
    wing.origin                  = [24.96, 0, 0]
    wing.vertical                = False
    wing.symmetric               = True
    wing.dynamic_pressure_ratio  = 0.9

    wing_planform(wing)

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'
    # equal to E190 data
    wing.aspect_ratio            = 1.6
    wing.sweeps.quarter_chord    = 30 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.52
    wing.span_efficiency         = 0.90
    wing.spans.projected         = 4.8502 * Units.meter
    wing.chords.root             = 3.9845 * Units.meter
    wing.chords.tip              = 2.0840 * Units.meter
    wing.chords.mean_aerodynamic = 3.1289 * Units.meter
    wing.areas.reference         = 14.72  * Units['meters**2']
    wing.areas.wetted            = 2.0   * wing.areas.reference
    wing.twists.root             = 0.0   * Units.degrees
    wing.twists.tip              = 0.0   * Units.degrees
    wing.origin                  = [21.667, 0, 0]
    wing.vertical                = True
    wing.symmetric               = False
    wing.dynamic_pressure_ratio  = 1.0

    wing_planform(wing)

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'
    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 4
    fuselage.seat_pitch            = 0.762
    fuselage.fineness.nose         = 2.0
    fuselage.fineness.tail         = 3.0
    fuselage.lengths.nose          = 6.00  * Units.meter
    fuselage.lengths.tail          = 6.00  * Units.meter
    fuselage.lengths.cabin         = 14.90 * Units.meter
    fuselage.lengths.total         = 27.17 * Units.meter
    fuselage.lengths.fore_space    = 0.    * Units.meter
    fuselage.lengths.aft_space     = 0.    * Units.meter
    fuselage.width                 = 2.787 * Units.meter
    fuselage.heights.maximum       = 3.020 * Units.meter
    fuselage.areas.side_projected  = 74.000 * Units['meters**2']
    fuselage.areas.wetted          = 240.00 * Units['meters**2']
    fuselage.areas.front_projected = 7.5000 * Units['meters**2']
    fuselage.effective_diameter    = 3.02
    fuselage.differential_pressure = 6.35 * Units.psi
    
    fuselage.heights.at_quarter_length          = 3.02 * Units.meter
    fuselage.heights.at_three_quarters_length   = 3.02 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 3.02 * Units.meter

    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   turboprop Network
    # ------------------------------------------------------------------    
    
    #instantiate the gas turbine network
    turboprop = SUAVE.Components.Energy.Networks.Turboprop()
    turboprop.__defaults__()
    turboprop.tag = 'turboprop'
    
    # setup
    turboprop.number_of_engines = 2
    turboprop.origin            = [[8.483, 4.05, 0], [8.483, -4.05, 0]] # meters
    turboprop.nacelle_diameter  = 1.27  * Units.meter
    turboprop.engine_length     = 4.532 * Units.meter
    turboprop.thrust_angle      = 0.0   * Units.deg
    turboprop.installed_power   = 4050. * Units.hp

    #compute engine areas
    turboprop.areas             = Data()
    turboprop.areas.wetted      = 2.0*np.pi*turboprop.nacelle_diameter*turboprop.engine_length
    
    # working fluid
    turboprop.working_fluid = SUAVE.Attributes.Gases.Air()
    
    # ------------------------------------------------------------------
    #   Component 1 - Gas Turbine
    
    # to convert freestream static to stagnation quantities
    # instantiate
    gas_turbine = SUAVE.Components.Energy.Converters.Gas_Turbine()
    gas_turbine.tag = 'gas_turbine'

    gas_turbine.datum_sea_level_power = 4050.
    gas_turbine.power_scaling_factor = 1.0
    gas_turbine.power_extraction = 'bleed_ECS'
    gas_turbine.load_data('engine.out')
    gas_turbine.bucket = Data()
    gas_turbine.bucket.RMTR = [0.600, 0.680, 0.760, 0.840, 0.920, 1.000]
    gas_turbine.bucket.RSFC = [1.126, 1.086, 1.056, 1.033, 1.014, 1.000]

    # add to the network
    turboprop.gas_turbine = gas_turbine
    # ------------------------------------------------------------------
    #  Component 2 - Propeller
    
    # instantiate
    propeller = SUAVE.Components.Energy.Converters.Propeller()
    propeller.tag = 'propeller'
    
    # setup
    prop_attributes = Data()
    prop_attributes.number_blades       = 6.0
    prop_attributes.tip_radius          = 1.980 * Units.meter
    prop_attributes.hub_radius          = 0.285 * Units.meter
    prop_attributes.angular_velocity    = 1200.*(2.*np.pi/60.0)  # Rotation Rate in rad/s
    prop_attributes.freestream_velocity = 140.0   # Freestream Velocity
    prop_attributes.design_Cl           = 0.43    # Design Lift Coefficient
    prop_attributes.design_altitude     = 25000.0 * Units.ft
    prop_attributes.design_thrust       = 0.00    * Units.N
    prop_attributes.design_power        = 1070.0  * 1e3 * Units.N * Units['m/s']

    prop_attributes = propeller_design(prop_attributes)
    propeller.prop_attributes = prop_attributes
    # add to network
    turboprop.propeller = propeller

    # ------------------------------------------------------------------
    #  Component 3 - Gear Box

    # instantiate
    gearbox = SUAVE.Components.Energy.Converters.Gearbox()
    gearbox.tag = 'gear_box'
    gearbox.efficiency = 0.985

    # setup
    turboprop.gearbox = gearbox

    # add  gas turbine network turboprop to the vehicle
    vehicle.append_component(turboprop)      
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.base = Data()
    configs.base = base_config

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.cruise = Data()
    configs.cruise = config

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg
    configs.takeoff = Data()
    configs.takeoff = config

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].flaps.angle = 30. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg

    configs.landing = Data()
    configs.landing = config

    return configs

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # wing areas
    for wing in base.wings:
        wing.areas.wetted   = 2.0 * wing.areas.reference
        wing.areas.exposed  = 0.8 * wing.areas.wetted

    # diff the new data
    base.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # make sure base data is current
    landing.pull_base()

    # landing weight
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff

    # diff the new data
    landing.store_diff()

    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):

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
    base_segment.state.unknowns.throttle = ones_row(1)
    base_segment.state.unknowns.pitch_command = ones_row(1) * 0. * Units.deg
    base_segment.state.conditions.propulsion = Data()
    base_segment.state.conditions.propulsion.rpm = 1200 * ones_row(1)
    base_segment.state.residuals.net = 0. * ones_row(1)
    base_segment.state.numerics.number_control_points = 10
    base_segment.state.conditions.freestream = Data()
    base_segment.state.conditions.freestream.delta_ISA = airport.delta_isa * ones_row(1)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"

    segment.analyses.extend( analyses.takeoff )

    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 5000 * Units.ft
    segment.air_speed      = 170.0 * Units.knots
    segment.climb_rate     = 1355. * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCL'

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.air_speed  = 245. * Units.knots
    segment.distance   = 100. * Units.nautical_miles

    segment.state.conditions.propulsion.gas_turbine_rating = 'MCR'

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end = 0.0  * Units.km
    segment.air_speed    = 170.0 * Units.knots
    segment.descent_rate = 1355. * Units['ft/min']

    segment.state.conditions.propulsion.gas_turbine_rating = 'FID'

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    return missions  

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):

    axis_font = {'fontname':'Arial', 'size':'14'}    

    # ------------------------------------------------------------------
    #   Aerodynamics
    # ------------------------------------------------------------------


    fig = plt.figure("Aerodynamic Forces",figsize=(8,6))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0] / Units.lbf
        eta    = segment.conditions.propulsion.throttle[:,0]

        axes = fig.add_subplot(2,1,1)
        axes.plot( time , Thrust , line_style )
        axes.set_ylabel('Thrust (lbf)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(2,1,2)
        axes.plot( time , eta , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Throttle',axis_font)
        axes.grid(True)	

        plt.savefig("ATR72_engine.pdf")
        plt.savefig("ATR72_engine.png")

    # ------------------------------------------------------------------
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients",figsize=(8,10))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        aoa = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d = CLift/CDrag

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , CLift , line_style )
        axes.set_ylabel('Lift Coefficient',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , l_d , line_style )
        axes.set_ylabel('L/D',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , aoa , 'ro-' )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('AOA (deg)',axis_font)
        axes.grid(True)

        plt.savefig("ATR72_aero.pdf")
        plt.savefig("ATR72_aero.png")

    # ------------------------------------------------------------------
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Drag Components",figsize=(8,10))
    axes = plt.gca()
    for i, segment in enumerate(results.segments.values()):

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cdc = drag_breakdown.compressible.total[:,0]
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cd  = drag_breakdown.total[:,0]

        if line_style == 'bo-':
            axes.plot( time , cdp , 'ko-', label='CD parasite' )
            axes.plot( time , cdi , 'bo-', label='CD induced' )
            axes.plot( time , cdc , 'go-', label='CD compressibility' )
            axes.plot( time , cdm , 'yo-', label='CD miscellaneous' )
            axes.plot( time , cd  , 'ro-', label='CD total'   )
            if i == 0:
                axes.legend(loc='upper center')            
        else:
            axes.plot( time , cdp , line_style )
            axes.plot( time , cdi , line_style )
            axes.plot( time , cdc , line_style )
            axes.plot( time , cdm , line_style )
            axes.plot( time , cd  , line_style )            

    axes.set_xlabel('Time (min)')
    axes.set_ylabel('CD')
    axes.grid(True)
    plt.savefig("ATR72_drag.pdf")
    plt.savefig("ATR72_drag.png")

    # ------------------------------------------------------------------
    #   Altitude, sfc, vehicle weight
    # ------------------------------------------------------------------

    fig = plt.figure("Altitude_sfc_weight",figsize=(8,10))
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        aoa      = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        mass     = segment.conditions.weights.total_mass[:,0] / Units.lb
        altitude = segment.conditions.freestream.altitude[:,0] / Units.ft
        mdot     = segment.conditions.weights.vehicle_mass_rate[:,0]
        thrust   =  segment.conditions.frames.body.thrust_force_vector[:,0]
        sfc      = (mdot / Units.lb) / (thrust /Units.lbf) * Units.hr

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , altitude , line_style )
        axes.set_ylabel('Altitude (ft)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , sfc , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('sfc (lb/lbf-hr)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , mass , 'ro-' )
        axes.set_ylabel('Weight (lb)',axis_font)
        axes.grid(True)

        plt.savefig("ATR72_mission.pdf")
        plt.savefig("ATR72_mission.png")
        
    # ------------------------------------------------------------------
    #   Velocities
    # ------------------------------------------------------------------
    fig = plt.figure("Velocities",figsize=(8,10))
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        Lift     = -segment.conditions.frames.wind.lift_force_vector[:,2]
        Drag     = -segment.conditions.frames.wind.drag_force_vector[:,0] / Units.lbf
        Thrust   = segment.conditions.frames.body.thrust_force_vector[:,0] / Units.lb
        velocity = segment.conditions.freestream.velocity[:,0]
        pressure = segment.conditions.freestream.pressure[:,0]
        density  = segment.conditions.freestream.density[:,0]
        EAS      = velocity * np.sqrt(density/1.225)
        mach     = segment.conditions.freestream.mach_number[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , velocity / Units.kts, line_style )
        axes.set_ylabel('velocity (kts)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , EAS / Units.kts, line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Equivalent Airspeed',axis_font)
        axes.grid(True)    
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , mach , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Mach',axis_font)
        axes.grid(True)           
        
    return

if __name__ == '__main__': 
    main()    
    plt.show()