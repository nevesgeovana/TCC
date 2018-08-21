# Constant_Throttle_Constant_EAS.py
#
# Created:  
# Modified: Aug 2016, T. MacDonald

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import scipy.optimize

# SUAVE imports
from SUAVE.Analyses.Mission.Segments import Aerodynamic
from SUAVE.Analyses.Mission.Segments import Conditions

from SUAVE.Methods.Missions import Segments as Methods

from SUAVE.Analyses import Process

# Units
from SUAVE.Core import Units


# ----------------------------------------------------------------------
#  Segment
# ----------------------------------------------------------------------

class Constant_Throttle_Constant_EAS(Aerodynamic):
    
    def __defaults__(self):
        
        # --------------------------------------------------------------
        #   User inputs
        # --------------------------------------------------------------
        self.altitude_start       = None # Optional
        self.altitude_end         = 10. * Units.km
        self.throttle             = 0.5
        self.mach_number          = 0.7
        self.settings.root_finder = scipy.optimize.root
        
        # --------------------------------------------------------------
        #   State
        # --------------------------------------------------------------
        
        # conditions
        self.state.conditions.update( Conditions.Aerodynamics() )
        
        # initials and unknowns
        ones_row = self.state.ones_row
        self.state.unknowns.body_angle = ones_row(1) * -1. * Units.deg
        self.state.unknowns.wind_angle = ones_row(1) * 0.8 * Units.deg
        self.state.residuals.forces    = ones_row(2) * 0.0
        
        
        # --------------------------------------------------------------
        #   The Solving Process
        # --------------------------------------------------------------
        
        # --------------------------------------------------------------
        #   Initialize - before iteration
        # --------------------------------------------------------------
        initialize = self.process.initialize
        initialize.clear()
        
        initialize.expand_state            = Methods.expand_state
        initialize.differentials           = Methods.Common.Numerics.initialize_differentials_dimensionless
        initialize.conditions              = Methods.Descent.Constant_Throttle_Constant_EAS.initialize_conditions
        initialize.velocities              = Methods.Descent.Constant_Throttle_Constant_EAS.update_velocity_vector_from_wind_angle
        initialize.differentials_altitude  = Methods.Descent.Constant_Throttle_Constant_EAS.update_differentials_time      
        
        # --------------------------------------------------------------
        #   Converge - starts iteration
        # --------------------------------------------------------------
        converge = self.process.converge
        converge.clear()
        
        converge.converge_root             = Methods.converge_root        
        
        # --------------------------------------------------------------
        #   Iterate - this is iterated
        # --------------------------------------------------------------
        iterate = self.process.iterate
        iterate.clear()
                
        # Update Initials
        iterate.initials = Process()
        iterate.initials.time              = Methods.Common.Frames.initialize_time
        iterate.initials.weights           = Methods.Common.Weights.initialize_weights
        iterate.initials.inertial_position = Methods.Common.Frames.initialize_inertial_position
        iterate.initials.planet_position   = Methods.Common.Frames.initialize_planet_position
        
        # Unpack Unknowns
        iterate.unpack_unknowns            = Methods.Descent.Constant_Throttle_Constant_EAS.unpack_body_angle 
        
        # Update Conditions
        iterate.conditions = Process()
        iterate.conditions.velocities      = Methods.Descent.Constant_Throttle_Constant_EAS.update_velocity_vector_from_wind_angle
        iterate.conditions.differentials_a = Methods.Descent.Constant_Throttle_Constant_EAS.update_differentials_time
        iterate.conditions.acceleration    = Methods.Descent.Constant_Throttle_Constant_EAS.update_acceleration
        iterate.conditions.altitude        = Methods.Common.Aerodynamics.update_altitude
        iterate.conditions.atmosphere      = Methods.Common.Aerodynamics.update_atmosphere
        iterate.conditions.gravity         = Methods.Common.Weights.update_gravity
        iterate.conditions.freestream      = Methods.Common.Aerodynamics.update_freestream
        iterate.conditions.orientations    = Methods.Common.Frames.update_orientations
        iterate.conditions.aerodynamics    = Methods.Common.Aerodynamics.update_aerodynamics
        iterate.conditions.stability       = Methods.Common.Aerodynamics.update_stability
        iterate.conditions.propulsion      = Methods.Common.Energy.update_thrust
        iterate.conditions.weights         = Methods.Descent.Constant_Throttle_Constant_EAS.update_weights
        iterate.conditions.forces          = Methods.Descent.Constant_Throttle_Constant_EAS.update_forces
        iterate.conditions.planet_position = Methods.Common.Frames.update_planet_position
        
        # Solve Residuals
        iterate.residuals = Process()
        iterate.residuals.total_forces     = Methods.Descent.Constant_Throttle_Constant_EAS.residual_total_forces
        
        # --------------------------------------------------------------
        #   Finalize - after iteration
        # --------------------------------------------------------------
        finalize = self.process.finalize
        finalize.clear()
        
        # Post Processing
        finalize.post_process = Process()        
        finalize.post_process.inertial_position = Methods.Common.Frames.integrate_inertial_horizontal_position
        finalize.post_process.stability         = Methods.Common.Aerodynamics.update_stability
       
        return

