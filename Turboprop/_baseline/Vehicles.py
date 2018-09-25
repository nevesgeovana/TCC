# Vehicles.py
# 
# Created:  Feb. 2016, M. Vegh
# Modified: Aug. 2017, E. Botero

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Data, Units
import numpy as np
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform import wing_planform
from SUAVE.Methods.Propulsion.propeller_design import propeller_design
# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)
    
    return configs

def base_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'ATR72'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff               = 23000. * Units.kg
    vehicle.mass_properties.operating_empty           = 13500. * Units.kg
    vehicle.mass_properties.takeoff                   = 23000. * Units.kg
    vehicle.mass_properties.max_zero_fuel             = 20800. * Units.kg
    vehicle.mass_properties.cargo                     = 0.0    * Units.kg
    vehicle.mass_properties.max_payload               = 7500.0 * Units.kg
    vehicle.mass_properties.max_fuel                  = 5000.0 * Units.kg

    vehicle.mass_properties.center_of_gravity = [11.760, 0., 0.]

    # envelope properties
    vehicle.envelope.ultimate_load = 2.5 * 1.5
    vehicle.envelope.limit_load    = 2.5
    vehicle.maximum_mach_operational = 0.55

    # basic parameters
    vehicle.reference_area         = 61 * Units['meters**2']
    vehicle.passengers             = 70
    vehicle.systems.control        = "aerodynamic powered"
    vehicle.systems.accessories    = "short-range"



    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 12
    wing.sweeps.quarter_chord    = 3.00 * Units.deg
    wing.thickness_to_chord      = 0.15
    wing.taper                   = 0.53
    wing.span_efficiency         = 0.907
    wing.calibration_factor      = 1.
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
    fuselage.lengths.nose          = 3.00  * Units.meter
    fuselage.lengths.tail          = 4.26  * Units.meter
    fuselage.lengths.cabin         = 19.91 * Units.meter
    fuselage.lengths.total         = 27.17 * Units.meter
    fuselage.lengths.fore_space    = 0.    * Units.meter
    fuselage.lengths.aft_space     = 0.    * Units.meter
    fuselage.width                 = 2.865 * Units.meter
    fuselage.heights.maximum       = 3.020 * Units.meter
    fuselage.areas.side_projected  = 70.000 * Units['meters**2']
    fuselage.areas.wetted          = 200.00 * Units['meters**2']
    fuselage.areas.front_projected = 7.5000 * Units['meters**2']
    fuselage.effective_diameter    = 3.02
    fuselage.differential_pressure = 6.00 * Units.psi

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
    turboprop.origin            = [[8.483, 4.05, 0], [8.483, -4.05, 0]] * Units.meter
    turboprop.nacelle_diameter  = 0.95  * Units.meter
    turboprop.engine_length     = 2.25  * Units.meter
    turboprop.thrust_angle      = 0.0   * Units.deg
    turboprop.installed_power   = 2750. * Units.hp

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

    gas_turbine.datum_sea_level_power = 2750. * Units.hp
    gas_turbine.power_scaling_factor  = [0.622, 0.696, 0.74]  # NTO, MCL, MCR 0.601
    gas_turbine.sfc_scaling_factor    = [1.135, 1.135, 1.11]  # NTO, MCL, MCR
    gas_turbine.power_extraction      = 'bleed_ECS'
    gas_turbine.load_data('engine.out')
    gas_turbine.bucket = Data()
    # gas_turbine.bucket.RMTR = [0.600, 0.680, 0.760, 0.840, 0.920, 1.000]
    # gas_turbine.bucket.RSFC = [1.126, 1.086, 1.056, 1.033, 1.014, 1.000]
    gas_turbine.bucket.RMTR = [0.600, 0.680, 0.760, 0.840, 0.900, 1.000]
    gas_turbine.bucket.RSFC = [1.136, 1.096, 1.066, 1.043, 1.03, 1.000]

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
    prop_attributes.design_altitude     = 21000.0 * Units.ft
    prop_attributes.design_thrust       = 0.00    * Units.N
    prop_attributes.design_power        = 1007.0  * 1e3 * Units.N * Units['m/s']

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
    turboprop.weight()
    # add  gas turbine network turboprop to the vehicle
    vehicle.append_component(turboprop)  
    # ------------------------------------------------------------------
    # now add weights objects
    vehicle.landing_gear       = SUAVE.Components.Landing_Gear.Landing_Gear()
    vehicle.control_systems    = SUAVE.Components.Physical_Component()
    vehicle.electrical_systems = SUAVE.Components.Physical_Component()
    vehicle.avionics           = SUAVE.Components.Energy.Peripherals.Avionics()
    vehicle.passenger_weights  = SUAVE.Components.Physical_Component()
    vehicle.furnishings        = SUAVE.Components.Physical_Component()
    vehicle.air_conditioner    = SUAVE.Components.Physical_Component()
    vehicle.fuel               = SUAVE.Components.Physical_Component()
    vehicle.apu                = SUAVE.Components.Physical_Component()
    vehicle.hydraulics         = SUAVE.Components.Physical_Component()
    vehicle.optionals          = SUAVE.Components.Physical_Component()

    vehicle.wings['vertical_stabilizer'].rudder = SUAVE.Components.Physical_Component()
    
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

    config.max_lift_coefficient_factor    = 1.

    configs.cruise = Data()
    configs.cruise = config
    # config.maximum_lift_coefficient = 1.2
    
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'

    config.wings['main_wing'].flaps.angle = 15.0 * Units.deg
    config.wings['main_wing'].slats.angle =  0.0 * Units.deg
    config.max_lift_coefficient_factor    = 1.08526
    config.V2_VS_ratio = 1.143

    configs.takeoff = Data()
    configs.takeoff = config

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].flaps.angle = 30.0 * Units.deg
    config.wings['main_wing'].slats.angle =  0.0 * Units.deg
    config.max_lift_coefficient_factor    = 1.12965
    
    config.Vref_VS_ratio = 1.23

    configs.landing = Data()
    configs.landing = config

    configs.landing.landing_constants = Data()
    configs.landing.landing_constants[0] = 250. * 0.25
    configs.landing.landing_constants[1] = 0.
    configs.landing.landing_constants[2] = 2.485 / 9.81

    return configs
