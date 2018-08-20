# tut_mission_E170.py
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
from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Propulsion import compute_turbofan_geometry
from SUAVE.Input_Output.Results import  print_parasite_drag,  \
     print_compress_drag, \
     print_engine_data,   \
     print_mission_breakdown, \
     print_weight_breakdown
from SUAVE.Methods.Performance  import payload_range
from SUAVE.Methods.Performance.estimate_take_off_field_length import estimate_take_off_field_length
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform import wing_planform
from SUAVE.Methods.Performance.estimate_landing_field_length import estimate_landing_field_length

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

    # print weight breakdown
    print_weight_breakdown(configs.base,filename = 'E170_weight_breakdown.dat')

    # print engine data into file
    print_engine_data(configs.base,filename = 'E170_engine_data.dat')

    # print parasite drag data into file - define reference condition for parasite drag
    ref_condition = Data()
    ref_condition.mach_number = 0.3
    ref_condition.reynolds_number = 12e6     
    print_parasite_drag(ref_condition,configs.cruise,analyses,'E170_parasite_drag.dat')

    # print compressibility drag data into file
    print_compress_drag(configs.cruise,analyses,filename = 'E170_compress_drag.dat')

    # print mission breakdown
    print_mission_breakdown(results,filename='E170_mission_breakdown.dat')

    state = Data()
    state.conditions = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()
    state.numerics = SUAVE.Analyses.Mission.Segments.Conditions.Numerics()

    # ---------------------------------------------------------------------------------------
    # PAYLOAD RANGE DIAGRAM

    config = configs.base
    cruise_segment_tag = "cruise"
    reserves = 1600.
    payload_range_results = payload_range(config, mission, cruise_segment_tag, reserves)

    # ---------------------------------------------------------------------------------------
    # PLOT RESULTS

    # plot_mission(results)

    # ---------------------------------------------------------------------------------------
    # TAKE OFF FIELD LENGTH
    # ---- Inputs
    analyses.base = analyses.configs.base
    airport = mission.airport
    clb_grad = 1

    # altitude = [0, 2000, 6000, 8000]
    # delta_isa = 0
    # weights_tofl = apmdata_tow_tofl()

    # altitude = [0, 5433, 8000]
    altitude = [0, 6000, 8000]
    delta_isa = 15
    weights_tofl = apmdata_tow_tofl_ISA15()


    # ---- Inputs: FLAP AND SLAT DEFLECTION
    flaps = [19.6, 9.7, 4.9, 0.]  # Deflections inboard local
    slats = [20., 12., 12., 0.]

    # ---- Inputs: FACTOR CLMAX
    configs.takeoff.max_lift_coefficient_factor = 0.9661

    # ---- Open output file
    fid = open('TOFL.txt', 'w')  # Open output file

    # ---- Run
    for j, h in enumerate(altitude):
        airport.altitude = h * Units.ft
        airport.delta_isa = delta_isa
        fid.write('Altitude: %4.0f ft \n' %(h))
        fid.write('TOFL      CLIMB GRADIENT     THRUST    L/D     L/Dv2    CDasym    CDwindm     CL     CD  CG_CORRECT\n')
        tofl                         = np.zeros(len(weights_tofl[j]))
        secsegclbgrad                = np.zeros(len(weights_tofl[j]))
        thrust                       = np.zeros(len(weights_tofl[j]))
        l_over_d                     = np.zeros(len(weights_tofl[j]))
        l_over_d_v2                  = np.zeros(len(weights_tofl[j]))
        asymmetry_drag_coefficient   = np.zeros(len(weights_tofl[j]))
        windmilling_drag_coefficient = np.zeros(len(weights_tofl[j]))
        clv2                         = np.zeros(len(weights_tofl[j]))
        cdv2                         = np.zeros(len(weights_tofl[j]))
        secsegclbgrad_corrected      = np.zeros(len(weights_tofl[j]))

        CLmax_ind = 0
        # configs.takeoff.maximum_lift_coefficient = maximum_lift_coefficient[CLmax_ind]
        configs.takeoff.wings['main_wing'].flaps.angle = flaps[CLmax_ind] * Units.deg
        configs.takeoff.wings['main_wing'].slats.angle = slats[CLmax_ind] * Units.deg

        for i, TOW in enumerate(weights_tofl[j]):
            configs.takeoff.mass_properties.takeoff = TOW * Units.kg
            tofl[i], secsegclbgrad[i], thrust[i], l_over_d[i], l_over_d_v2[i], asymmetry_drag_coefficient[i], \
            windmilling_drag_coefficient[i], clv2[i], cdv2[i], secsegclbgrad_corrected[
                i] = estimate_take_off_field_length(configs.takeoff,
                                                    analyses, airport,
                                                    clb_grad)
            if secsegclbgrad_corrected[i] < 0.024:
                CLmax_ind = CLmax_ind + 1
                if CLmax_ind > 2:
                    CLmax_ind = 2
                    print CLmax_ind,CLmax_ind,CLmax_ind,CLmax_ind,CLmax_ind,CLmax_ind

                configs.takeoff.wings['main_wing'].flaps.angle = flaps[CLmax_ind] * Units.deg
                configs.takeoff.wings['main_wing'].slats.angle = slats[CLmax_ind] * Units.deg

                # configs.takeoff.maximum_lift_coefficient = maximum_lift_coefficient[CLmax_ind]

                tofl[i], secsegclbgrad[i], thrust[i], l_over_d[i], l_over_d_v2[i], asymmetry_drag_coefficient[i], \
                windmilling_drag_coefficient[i], clv2[i], cdv2[i], secsegclbgrad_corrected[
                    i] = estimate_take_off_field_length(configs.takeoff,
                                                        analyses, airport,
                                                        clb_grad)

            fid.write('%4.2f     %4.4f    %4.4f    %4.4f    %4.4f    %4.4f    %4.4f    %4.4f    %4.4f    %4.4f \n'
                      % (tofl[i], secsegclbgrad[i], thrust[i], l_over_d[i], l_over_d_v2[i],
                         asymmetry_drag_coefficient[i], windmilling_drag_coefficient[i], clv2[i], cdv2[i],
                         secsegclbgrad_corrected[i]))
        fid.write('\n')

    fid.close()

    # ---------------------------------------------------------------------------------------
    # LANDING FIELD LENGTH
    # ---- Inputs
    airport.delta_isa = 0
    # weights_lfl = weights_lfl_apm_FLAP5()
    weights_lfl = weights_lfl_apm_FLAPfull()
    flaps = [19.6, 24.2]
    slats = [20., 20.]
    configs.landing.landing_constants = Data()

    configs.landing.landing_constants[0] = 250. * 1.1
    configs.landing.landing_constants[1] = 0.
    configs.landing.landing_constants[2] = 2.485/9.81 * 0.94

    configs.landing.wings['main_wing'].flaps.angle = flaps[1] * Units.deg
    configs.landing.wings['main_wing'].slats.angle = slats[1] * Units.deg
    altitude = [0, 6000, 8000]
    configs.landing.max_lift_coefficient_factor = configs.takeoff.max_lift_coefficient_factor

    fid = open('LFL.txt', 'w')  # Open output file
    for j, h in enumerate(altitude):
        airport.altitude = h * Units.ft
        lfl = np.zeros(len(weights_lfl[j]))
        fid.write('Altitude: %4.0f ft \n' % (h))
        for i, TOW in enumerate(weights_lfl[j]):
            configs.landing.mass_properties.landing = TOW * Units.kg
            lfl[i] = estimate_landing_field_length(configs.landing, analyses, airport)
            fid.write('%4.2f \n' % (lfl[i]))
        fid.write('\n')
    fid.close()

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
    vehicle.tag = 'E170'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff               = 37200. * Units.kg
    vehicle.mass_properties.operating_empty           = 20736. * Units.kg
    vehicle.mass_properties.takeoff                   = 31406. * Units.kg
    vehicle.mass_properties.max_zero_fuel             = 30140. * Units.kg
    vehicle.mass_properties.cargo                     = 0.0    * Units.kg
    vehicle.mass_properties.max_payload               = 9404.0 * Units.kg
    vehicle.mass_properties.max_fuel                  = 9428.0 * Units.kg

    vehicle.mass_properties.center_of_gravity = [11., 0., 0.]

    # envelope properties
    vehicle.envelope.ultimate_load = 2.5 * 1.5
    vehicle.envelope.limit_load    = 2.5

    # basic parameters
    vehicle.reference_area         = 72.72 * Units['meters**2']  
    vehicle.passengers             = 72
    vehicle.systems.control        = "fully powered"
    vehicle.systems.accessories    = "medium range"



    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 8.6
    wing.sweeps.quarter_chord    = 23.0 * Units.deg # 22.5
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.3275 #0.28
    wing.span_efficiency         = 0.96
    wing.spans.projected         = 26.0  * Units.meter
    wing.chords.root             = 4.2138 * Units.meter #5.428 * Units.meter
    wing.chords.tip              = 1.380 * Units.meter
    wing.chords.mean_aerodynamic = 3.806 * Units.meter
    wing.areas.reference         = 72.72 * Units['meters**2']
    wing.areas.wetted            = 2.0   * wing.areas.reference
    wing.areas.exposed           = 0.8   * wing.areas.wetted
    wing.twists.root             = 2.0   * Units.degrees
    wing.twists.tip              = 0.0   * Units.degrees
    wing.origin                  = [10.36122, 0, 0]
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    wing.flaps.type              = "double_slotted"
    wing.flaps.chord             = 0.280
    wing.dynamic_pressure_ratio  = 1.0
    wing.flaps.span_start        = 0.13
    wing.flaps.span_end          = 0.75

    wing_planform(wing)

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 4.3
    wing.sweeps.quarter_chord    = 30.0 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.3707
    wing.span_efficiency         = 0.9
    wing.spans.projected         = 10.000 * Units.meter
    wing.chords.root             = 3.394  * Units.meter
    wing.chords.tip              = 1.258  * Units.meter
    wing.chords.mean_aerodynamic = 2.4895 * Units.meter
    wing.areas.reference         = 23.25  * Units['meters**2'] 
    wing.areas.wetted            = 2.0    * wing.areas.reference
    wing.areas.exposed           = 0.8    * wing.areas.wetted
    wing.twists.root             = 2.0    * Units.degrees
    wing.twists.tip              = 2.0    * Units.degrees
    wing.origin                  = [24.6, 0, 0]
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
    wing.aspect_ratio            = 1.7
    wing.sweeps.quarter_chord    = 35 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.26
    wing.span_efficiency         = 0.90
    wing.spans.projected         = 5.2153 * Units.meter
    wing.chords.root             = 5.5779 * Units.meter
    wing.chords.tip              = 1.4547 * Units.meter
    wing.chords.mean_aerodynamic = 3.9192 * Units.meter
    wing.areas.reference         = 16.0  * Units['meters**2'] 
    wing.areas.wetted            = 2.0   * wing.areas.reference
    wing.areas.exposed           = 0.8   * wing.areas.wetted
    wing.twists.root             = 0.0   * Units.degrees
    wing.twists.tip              = 0.0   * Units.degrees
    wing.origin                  = [23.9, 0, 0]
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
    fuselage.seat_pitch            = 0.7455  #
    fuselage.fineness.nose         = 2.0     #
    fuselage.fineness.tail         = 3.0     #
    fuselage.lengths.nose          = 6.00  * Units.meter
    fuselage.lengths.tail          = 9.00  * Units.meter
    fuselage.lengths.cabin         = 14.90 * Units.meter
    fuselage.lengths.total         = 29.90 * Units.meter
    fuselage.lengths.fore_space    = 0.    * Units.meter
    fuselage.lengths.aft_space     = 0.    * Units.meter
    fuselage.width                 = 3.000 * Units.meter
    fuselage.heights.maximum       = 3.400 * Units.meter
    fuselage.areas.side_projected  = 197.35 * Units['meters**2']
    fuselage.areas.wetted          = 280.00 * Units['meters**2'] # 269.80
    fuselage.areas.front_projected = 8.0110 * Units['meters**2']     # 8.0110
    fuselage.effective_diameter    = 3.2
    fuselage.differential_pressure = 8.94 * Units.psi
    
    fuselage.heights.at_quarter_length          = 3.4 * Units.meter
    fuselage.heights.at_three_quarters_length   = 3.4 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 3.4 * Units.meter

    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   Turbofan Network
    # ------------------------------------------------------------------    
    
    #instantiate the gas turbine network
    turbofan = SUAVE.Components.Energy.Networks.Turbofan()
    turbofan.tag = 'turbofan'
    
    # setup
    turbofan.number_of_engines = 2
    turbofan.bypass_ratio      = 5.0
    turbofan.engine_length     = 3.1  * Units.meter
    turbofan.nacelle_diameter  = 1.64 * Units.meter
    turbofan.origin            = [[9.721, 3.984, -1], [9.721, -3.984, -1]] # meters
    
    #compute engine areas
    turbofan.areas.wetted      = 1.1*np.pi*turbofan.nacelle_diameter*turbofan.engine_length
    
    # working fluid
    turbofan.working_fluid = SUAVE.Attributes.Gases.Air()
    
    # ------------------------------------------------------------------
    #   Component 1 - Ram
    
    # to convert freestream static to stagnation quantities
    # instantiate
    ram = SUAVE.Components.Energy.Converters.Ram()
    ram.tag = 'ram'
    
    # add to the network
    turbofan.append(ram)

    # ------------------------------------------------------------------
    #  Component 2 - Inlet Nozzle
    
    # instantiate
    inlet_nozzle = SUAVE.Components.Energy.Converters.Compression_Nozzle()
    inlet_nozzle.tag = 'inlet_nozzle'
    
    # setup
    inlet_nozzle.polytropic_efficiency = 0.98
    inlet_nozzle.pressure_ratio        = 0.98
    
    # add to network
    turbofan.append(inlet_nozzle)
    
    # ------------------------------------------------------------------
    #  Component 3 - Low Pressure Compressor
    
    # instantiate 
    compressor = SUAVE.Components.Energy.Converters.Compressor()    
    compressor.tag = 'low_pressure_compressor'

    # setup
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 1.90
    
    # add to network
    turbofan.append(compressor)
    
    # ------------------------------------------------------------------
    #  Component 4 - High Pressure Compressor
    
    # instantiate
    compressor = SUAVE.Components.Energy.Converters.Compressor()    
    compressor.tag = 'high_pressure_compressor'
    
    # setup
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 7.5
    
    # add to network
    turbofan.append(compressor)

    # ------------------------------------------------------------------
    #  Component 5 - Low Pressure Turbine
    
    # instantiate
    turbine = SUAVE.Components.Energy.Converters.Turbine()   
    turbine.tag='low_pressure_turbine'
    
    # setup
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93
    
    # add to network
    turbofan.append(turbine)
      
    # ------------------------------------------------------------------
    #  Component 6 - High Pressure Turbine
    
    # instantiate
    turbine = SUAVE.Components.Energy.Converters.Turbine()   
    turbine.tag='high_pressure_turbine'

    # setup
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93
    
    # add to network
    turbofan.append(turbine)  
    
    # ------------------------------------------------------------------
    #  Component 7 - Combustor
    
    # instantiate    
    combustor = SUAVE.Components.Energy.Converters.Combustor()   
    combustor.tag = 'combustor'
    
    # setup
    combustor.efficiency                = 0.99 
    combustor.alphac                    = 1.0     
    combustor.turbine_inlet_temperature = 1500 # K
    combustor.pressure_ratio            = 0.95
    combustor.fuel_data                 = SUAVE.Attributes.Propellants.Jet_A()    
    
    # add to network
    turbofan.append(combustor)

    # ------------------------------------------------------------------
    #  Component 8 - Core Nozzle
    
    # instantiate
    nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    nozzle.tag = 'core_nozzle'
    
    # setup
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.98
    
    # add to network
    turbofan.append(nozzle)

    # ------------------------------------------------------------------
    #  Component 9 - Fan Nozzle
    
    # instantiate
    nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    nozzle.tag = 'fan_nozzle'

    # setup
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.98
    
    # add to network
    turbofan.append(nozzle)
    
    # ------------------------------------------------------------------
    #  Component 10 - Fan
    
    # instantiate
    fan = SUAVE.Components.Energy.Converters.Fan()   
    fan.tag = 'fan'

    # setup
    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio        = 1.625
    
    # add to network
    turbofan.append(fan)
    
    # ------------------------------------------------------------------
    #Component 10 : thrust (to compute the thrust)
    thrust = SUAVE.Components.Energy.Processes.Thrust()       
    thrust.tag ='compute_thrust'
 
    #total design thrust (includes all the engines)
    thrust.total_design             = 27550.0 * Units.N #Newtons
 
    #design sizing conditions
    altitude      = 35000.0*Units.ft
    mach_number   = 0.78
    isa_deviation = 0.
    
    #Engine setup for noise module    
    # add to network
    turbofan.thrust = thrust

    #size the turbofan
    turbofan_sizing(turbofan, mach_number, altitude, isa_deviation)
    
    # add  gas turbine network turbofan to the vehicle 
    vehicle.append_component(turbofan)      
    
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
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.append(config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg
    # config.max_lift_coefficient_factor    = 1.
    config.V2_VS_ratio = 1.2
    # config.maximum_lift_coefficient = 2.62

    configs.append(config)
    # ------------------------------------------------------------------
    #   Cruise with Spoilers Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise_spoilers'
    config.maximum_lift_coefficient = 1.2
    
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 20. * Units.deg
    config.max_lift_coefficient_factor    = 1. #0.95

    configs.append(config)    

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].flaps.angle = 30. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg  
    # config.max_lift_coefficient_factor    = 1. #0.95

    # config.maximum_lift_coefficient = 2.52
    
    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'
    
    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg
    config.max_lift_coefficient_factor    = 1. #0.95
  
    configs.append(config)

    return configs

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # zero fuel weight
    # base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff

    # wing areas
    for wing in base.wings:
        wing.areas.wetted   = 2.0 * wing.areas.reference
        wing.areas.exposed  = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted

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

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"

    segment.analyses.extend( analyses.takeoff )

    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 3.048   * Units.km
    segment.air_speed      = 138.0 * Units['m/s']
    segment.climb_rate     = 3000. * Units['ft/min']

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end = 3.657 * Units.km
    segment.air_speed    = 168.0 * Units['m/s']
    segment.climb_rate   = 2500. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: constant Speed, Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end = 25000. * Units.ft
    segment.air_speed    = 200.0  * Units['m/s']
    segment.climb_rate   = 1800. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Fourth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_4"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    segment.altitude_end = 32000. * Units.ft
    segment.air_speed    = 230.0* Units['m/s']
    segment.climb_rate   = 900. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Fifth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate()
    segment.tag = "climb_5"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    # segment.atmosphere   = atmosphere
    # segment.planet       = planet

    segment.altitude_end = 37000. * Units.ft
    segment.air_speed    = 230.0  * Units['m/s']
    segment.climb_rate   = 300.   * Units['ft/min']

    # add to mission
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.air_speed  = 450. * Units.knots
    segment.distance   = 152. * Units.nautical_miles

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end = 9.31  * Units.km
    segment.air_speed    = 440.0 * Units.knots
    segment.descent_rate = 2600. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"

    segment.analyses.extend( analyses.landing )

    segment.altitude_end = 3.657 * Units.km
    segment.air_speed    = 365.0 * Units.knots
    segment.descent_rate = 2300. * Units['ft/min']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3"

    segment.analyses.extend( analyses.landing )
    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00

    segment.altitude_end = 0.0   * Units.km
    segment.air_speed    = 250.0 * Units.knots
    segment.descent_rate = 1500. * Units['ft/min']

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
#   APM DATA - TOFL
# ----------------------------------------------------------------------
def apmdata_tow_tofl():
    # h = 0, 2000, 6000, 8000
    weights_tofl = [[20000,
                     21009.319,
                     22008.12425,
                     23017.44325,
                     24005.73477,
                     25009.79689,
                     25998.08841,
                     27427.95699,
                     28999.76105,
                     29988.05257,
                     30997.37157,
                     32994.98208,
                     35008.3632,
                     37005.97372,
                     38588.29152,
                     38600.0000,
                     31406.],
                    [22986.61568,
                     23985.65966,
                     24448.37476,
                     25037.28489,
                     25994.26386,
                     26509.56023,
                     26993.30784,
                     27487.5717,
                     27992.35182,
                     28644.35946,
                     29380.49713,
                     29990.43977,
                     30600.38241,
                     31231.35755,
                     32009.56023,
                     32608.98662,
                     33103.25048,
                     33608.03059,
                     34270.55449,
                     35027.72467,
                     35564.05354,
                     35805.92734,
                     36110.89866,
                     36405.35373,
                     36647.22753,
                     36994.26386,
                     37015.29637,
                     37141.4914,
                     37372.84895,
                     37583.174,
                     37804.0153,
                     38003.82409,
                     38214.14914,
                     38392.92543,
                     38550.66922],
                    [21020.07648,
                     21282.98279,
                     21608.98662,
                     21998.08795,
                     22534.41683,
                     23039.19694,
                     23491.39579,
                     23859.46463,
                     24301.14723,
                     24784.89484,
                     25237.09369,
                     25563.09751,
                     26152.00765,
                     26719.88528,
                     27140.53537,
                     27645.31549,
                     28013.38432,
                     28497.13193,
                     28970.36329,
                     29601.33843,
                     29990.43977,
                     30505.73614,
                     31000,
                     31420.6501,
                     31673.04015,
                     31830.78394,
                     31988.52772,
                     32009.56023,
                     32188.33652,
                     32335.56405,
                     32535.37285,
                     32672.08413,
                     32861.37667,
                     33050.66922,
                     33218.92925,
                     33397.70554,
                     33660.61185,
                     33944.55067,
                     34112.81071,
                     34228.48948,
                     34533.4608,
                     34922.56214,
                     35185.46845,
                     35500.95602,
                     35963.67113,
                     36173.99618,
                     36426.38623,
                     36973.23136],
                    [20052.58126,
                     20420.6501,
                     20999.04398,
                     21998.08795,
                     22997.13193,
                     23996.17591,
                     24900.57361,
                     25994.26386,
                     26993.30784,
                     27992.35182,
                     28728.48948,
                     28980.87954,
                     29317.39962,
                     29632.88719,
                     29990.43977,
                     30011.47228,
                     30169.21606,
                     30369.02486,
                     30568.83365,
                     30705.54493,
                     30894.83748,
                     31000,
                     31178.77629,
                     31304.97132,
                     31410.13384,
                     31525.81262,
                     31620.45889,
                     31862.3327,
                     32156.78776,
                     32409.17782,
                     32724.66539,
                     32987.5717,
                     33218.92925,
                     33376.67304,
                     33544.93308,
                     33734.22562,
                     33828.87189,
                     33913.00191,
                     33986.61568,
                     34018.16444,
                     34123.32696,
                     34196.94073,
                     34270.55449,
                     34365.20076,
                     34459.84704,
                     34565.00956,
                     34670.17208,
                     34785.85086,
                     34891.01338,
                     34985.65966]]

    return weights_tofl
def apmdata_tow_tofl_ISA15():
    # h = 0, 6000, 8000 ft
    weights_tofl = [[23100.81223,
                     24015.28906,
                     25003.34448,
                     26001.91113,
                     27042.52269,
                     28051.60057,
                     29239.36933,
                     30132.8237,
                     31141.90158,
                     32182.51314,
                     33012.90014,
                     33538.46154,
                     34074.53416,
                     34568.56187,
                     34989.01099,
                     35293.8366,
                     35598.66221,
                     35998.08887,
                     36597.22886,
                     37175.34639,
                     37795.50884,
                     38016.24462,
                     38205.44673,
                     38415.67129,
                     38562.82848,
                     38594.36216],
                    [21093.1677,
                     21660.77401,
                     22995.69995,
                     23710.46345,
                     24561.87291,
                     25024.36694,
                     26033.44482,
                     27021.50024,
                     27536.55041,
                     27851.88724,
                     28366.93741,
                     28860.96512,
                     29291.92547,
                     29901.57668,
                     30017.20019,
                     30080.26756,
                     30195.89107,
                     30322.0258,
                     30437.64931,
                     30553.27281,
                     30679.40755,
                     30774.0086,
                     30879.12088,
                     31005.25561,
                     31099.85667,
                     31204.96894,
                     31310.08122,
                     31404.68227,
                     31457.23841,
                     31509.79455,
                     31562.35069,
                     31635.92929,
                     31698.99666,
                     31772.57525,
                     31835.64262,
                     31898.70999,
                     31940.7549,
                     31961.77735,
                     31993.31104,
                     32003.82226,
                     32024.84472,
                     32045.86718,
                     32066.88963,
                     32098.42332,
                     32129.957,
                     32193.02437,
                     32224.55805,
                     32308.64787,
                     32350.69279,
                     32413.76015,
                     32466.31629,
                     32508.3612,
                     32539.89489,
                     32581.9398,
                     32634.49594,
                     32676.54085,
                     32750.11945,
                     32813.18681,
                     32865.74295,
                     32928.81032,
                     33002.38892,
                     33065.45628,
                     33202.10225,
                     33443.86049,
                     33801.24224,
                     34043.00048,
                     34694.69661,
                     34999.52222,
                     35136.16818,
                     35220.258,
                     35272.81414,
                     35325.37028,
                     35388.43765,
                     35440.99379,
                     35483.0387,
                     35525.08361,
                     35577.63975,
                     35609.17344,
                     35672.2408,
                     35703.77449,
                     35745.8194,
                     35787.86431,
                     35819.39799,
                     35850.93168,
                     35882.46536,
                     35903.48782,
                     35924.51027,
                     35945.53273,
                     35977.06641,
                     35998.08887,
                     36008.6001,
                     36050.64501,
                     36082.17869,
                     36134.73483,
                     36166.26851,
                     36197.8022,
                     36239.84711,
                     36281.89202,
                     36334.44816,
                     36365.98184,
                     36408.02676,
                     36450.07167,
                     36471.09412,
                     36513.13903,
                     36576.2064,
                     36628.76254,
                     36681.31868,
                     36740.,
                     36754.89728,
                     36817.96464,
                     36870.52078,
                     36996.65552,
                     37000.],
                    [20000,
                     20241.75824,
                     20357.38175,
                     20620.16245,
                     20903.9656,
                     21019.58911,
                     21271.85858,
                     21492.59436,
                     22091.73435,
                     22785.47539,
                     23311.03679,
                     23563.30626,
                     24866.69852,
                     25213.56904,
                     25980.88868,
                     27010.98901,
                     28020.06689,
                     28188.24654,
                     28272.33636,
                     28324.8925,
                     28440.51601,
                     28503.58337,
                     28577.16197,
                     28671.76302,
                     28787.38653,
                     28860.96512,
                     28934.54372,
                     28997.61108,
                     29092.21214,
                     29186.81319,
                     29302.43669,
                     29418.0602,
                     29512.66125,
                     29596.75108,
                     29670.32967,
                     29733.39704,
                     29806.97563,
                     29838.50932,
                     29891.06546,
                     29933.11037,
                     29975.15528,
                     30006.68896,
                     30017.20019,
                     30038.22265,
                     30069.75633,
                     30122.31247,
                     30174.86861,
                     30216.91352,
                     30269.46966,
                     30332.53703,
                     30385.09317,
                     30437.64931,
                     30500.71667,
                     30532.25036,
                     30584.8065,
                     30626.85141,
                     30668.89632,
                     30700.43,
                     30721.45246,
                     30763.49737,
                     30795.03106,
                     30910.65456,
                     31036.7893,
                     31204.96894,
                     31362.63736,
                     31530.81701,
                     31751.5528,
                     32003.82226,
                     32214.04682,
                     32382.22647,
                     32455.80506,
                     32508.3612,
                     32602.96226,
                     32666.02962,
                     32708.07453,
                     32771.1419,
                     32813.18681,
                     32844.7205,
                     32897.27664,
                     32928.81032,
                     32960.344,
                     32981.36646,
                     32991.87769,
                     33023.41137,
                     33075.96751,
                     33139.03488,
                     33191.59102,
                     33265.16961,
                     33317.72575,
                     33359.77066,
                     33433.34926,
                     33496.41663,
                     33569.99522,
                     33654.08505,
                     33780.21978,
                     33853.79838,
                     33906.35452,
                     33979.93311,
                     34000.95557]]

    return weights_tofl
# ----------------------------------------------------------------------
#   APM DATA - TOFL
# ----------------------------------------------------------------------
def weights_lfl_apm_FLAP5():
    weights_lfl = [[20056.80978,
                    20315.16652,
                    20636.15421,
                    21426.87685,
                    22006.21935,
                    22475.95624,
                    23180.56255,
                    23493.72306,
                    24010.43461,
                    24895.1143,
                    25991.16835,
                    26554.85455,
                    27008.9313,
                    28011.0399,
                    28316.37323,
                    29005.32712,
                    29623.81903,
                    30297.11664,
                    31001.72488,
                    32003.84702,
                    32849.3889,
                    33021.62544,
                    33326.96264,
                    33640.12315,
                    33976.77292,
                    33300.00000],
                   [20057.67044,
                    20386.49497,
                    21028.48775,
                    21913.19645,
                    22484.73691,
                    23001.47747,
                    23573.016,
                    24340.29,
                    25029.2729,
                    26000.11148,
                    26493.36085,
                    27401.56655,
                    28012.25449,
                    29022.24442,
                    29492.01806,
                    29930.46753,
                    30384.58875,
                    31003.13868,
                    31566.88485,
                    31895.74032,
                    32318.56058,
                    32741.38278,
                    33070.24405,
                    33547.88549,
                    33680.99775,
                    33845.43033,
                    34002.02799],
                   [20026.67893,
                    20238.07262,
                    20700.01135,
                    21193.26071,
                    21960.53858,
                    22140.6197,
                    22994.02744,
                    23087.97869,
                    23886.58266,
                    24019.67946,
                    24473.78908,
                    24943.55692,
                    25491.62165,
                    25804.79957,
                    25867.43438,
                    26243.25291,
                    26752.17016,
                    27214.11276,
                    27660.3952,
                    27997.06625,
                    28310.25384,
                    28506.00091,
                    28787.87593,
                    29288.98728,
                    29633.50678,
                    30064.14841,
                    30486.97834,
                    30761.03779,
                    31332.64207,
                    31457.92523,
                    31559.71912,
                    32131.32728,
                    32820.38754,
                    33485.96435,
                    33689.55407,
                    33971.44456]]

    return weights_lfl

def weights_lfl_apm_FLAPfull():
    weights_lfl = [[20006.15195,
                    21005.84436,
                    21551.83021,
                    23012.9191,
                    24074.13104,
                    24996.92402,
                    26734.85082,
                    28057.52076,
                    29695.47831,
                    30672.10089,
                    32240.84897,
                    33302.1,
                    33300.00,
                    33978.77576],
                   [19998.46201,
                    21005.84436,
                    21997.84682,
                    24235.61981,
                    27142.41772,
                    29011.07352,
                    31010.45832,
                    32010.15072,
                    32994.46324,
                    33209.78161,
                    33302.0609,
                    33986.4657],
                   [19990.77207,
                    21013.5343,
                    21398.03137,
                    22251.61489,
                    25012.30391,
                    28142.11012,
                    29680.09843,
                    31717.93294,
                    33002.15318,
                    33302.0609,
                    33955.70594]]

    return weights_lfl
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

        plt.savefig("E170_engine.pdf")
        plt.savefig("E170_engine.png")

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

        plt.savefig("E170_aero.pdf")
        plt.savefig("E170_aero.png")

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
    plt.savefig("E170_drag.pdf")
    plt.savefig("E170_drag.png")

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

        plt.savefig("E170_mission.pdf")
        plt.savefig("E170_mission.png")
        
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