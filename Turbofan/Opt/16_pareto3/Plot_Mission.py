# Plot_Mission.py
# 
# Created:  May 2015, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Input_Output.Results import  print_parasite_drag,  \
     print_compress_drag, \
     print_engine_data,   \
     print_mission_breakdown, \
     print_weight_breakdown
import pylab as plt

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(nexus,id=0,line_style='bo-'):
    results   = nexus.results.base
    axis_font = {'fontname':'Arial', 'size':'14'}
    configs = nexus.vehicle_configurations
    analyses = Data()
    analyses.configs = Data()
    analyses.configs = nexus.analyses

    # print mission breakdown
    print_mission_breakdown(results, filename='mission_breakdown'+str(id)+'.dat')

    # print engine data into file
    print_engine_data(configs.base, filename='engine_data'+str(id)+'.dat')

    # print weight breakdown
    print_weight_breakdown(configs.base, filename='weight_breakdown'+str(id)+'.dat')

    # print compressibility drag data into file
    print_compress_drag(configs.cruise, analyses, filename='compress_drag'+str(id)+'.dat')

    # print parasite drag data into file - define reference condition for parasite drag
    ref_condition = Data()
    ref_condition.mach_number = 0.3
    ref_condition.reynolds_number = 12e6
    print_parasite_drag(ref_condition, configs.cruise, analyses, 'parasite_drag'+str(id)+'.dat')
    # ------------------------------------------------------------------
    #   Aerodynamics
    # ------------------------------------------------------------------

    fig = plt.figure('Mission Parameters' + str(id), figsize=(8, 12))
    for segment in results.segments.values():

        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        Thrust = segment.conditions.frames.body.thrust_force_vector[:, 0] / Units.lbf
        eta = segment.conditions.propulsion.throttle[:, 0]

        mdot = segment.conditions.weights.vehicle_mass_rate[:, 0] * 60 * 60
        mdot2 = segment.conditions.weights.vehicle_mass_rate[:, 0] / Units['lb/h']

        try:
            sfc = segment.conditions.propulsion.psfc
            pitch = segment.conditions.propulsion.pitch_command[:, 0]
            P = segment.conditions.propulsion.propeller_power[:, 0] * 1.34102 / 1000
            gas_turbine_power = segment.conditions.propulsion.gas_turbine_power[:, 0] * 1.34102 / 1000
            etap = segment.conditions.propulsion.etap[:, 0]
            mdot3 = segment.conditions.propulsion.vehicle_mass_rate
            sfc2 = mdot2 / P / 2.
        except:
            sfc = mdot2 / Thrust / 2.
            pitch = segment.conditions.propulsion.throttle[:, 0] * 0.
            P = segment.conditions.propulsion.throttle[:, 0] * 0.
            gas_turbine_power = segment.conditions.propulsion.throttle[:, 0] * 0.
            etap = segment.conditions.propulsion.throttle[:, 0] * 0.
            sfc2 = sfc
            mdot3 = mdot2

        axes = fig.add_subplot(5, 1, 1)
        axes.plot(time, Thrust, line_style)
        axes.set_ylabel('Thrust (lbf)', axis_font)
        axes.grid(True)

        axes = fig.add_subplot(5, 1, 2)
        axes.plot(time, P, 'ko-', label='Propeller')
        axes.plot(time, gas_turbine_power, 'ro-', label='Gas Turbine')
        axes.set_ylabel('Power (HP)', axis_font)
        axes.grid(True)

        axes = fig.add_subplot(5, 1, 3)
        axes.plot(time, eta, line_style)
        axes.set_xlabel('Time (min)', axis_font)
        axes.set_ylabel('Throttle', axis_font)
        axes.grid(True)

        axes = fig.add_subplot(5, 1, 4)
        axes.plot(time, pitch * 180 / 3.14159, line_style)
        axes.set_xlabel('Time (min)', axis_font)
        axes.set_ylabel('Pitch Command', axis_font)
        axes.grid(True)

        axes = fig.add_subplot(5, 1, 5)
        axes.plot(time, etap, line_style)
        axes.set_xlabel('Time (min)', axis_font)
        axes.set_ylabel('Propeller Efficiency', axis_font)
        axes.grid(True)

        plt.savefig("engine" + str(id) + ".png")

    # ------------------------------------------------------------------
    #   Aerodynamics 
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

    plt.savefig("aero" + str(id) + ".png")

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
    plt.savefig("drag" + str(id) + ".png")

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
    plt.savefig("mission" + str(id) + ".png")
    # plt.show(block=True)

    return

if __name__ == '__main__': 
    main()    
    # plt.show()