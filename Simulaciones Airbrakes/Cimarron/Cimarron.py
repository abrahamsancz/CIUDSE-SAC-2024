import rocketpy
from rocketpy import Environment, Rocket, GenericMotor, Flight
import datetime

env = Environment(
    latitude=32.94058043702919,
    longitude=-106.92174324252515,
    elevation=1400,
) 

#tomorrow = datetime.date.today() + datetime.timedelta(days=1)

env.set_date(
  (2024, 6, 22, 12), timezone="America/Denver"
) # Tomorrow's date in year, month, day, hour UTC format

env.set_atmospheric_model(type='Forecast', file='GFS')

# Motor
AeroTech_M2500T = GenericMotor(
    thrust_source="AeroTech_M2500T.eng",
    dry_mass=3.353,
    dry_inertia=(0.125, 0.125, 0.002),
    center_of_dry_mass_position=0.385,
    burn_time=3.9,
    chamber_radius=0.049,
    chamber_height=0.751,
    chamber_position=0.3755,
    propellant_initial_mass=4.711,
    nozzle_radius=0.0127,
    interpolation_method="linear",
    nozzle_position=0,
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

# Cohete
Cimarron = Rocket(
    radius=0.065,
    mass=24.029,  # without motor
    inertia=(6.321, 6.321, 0.034),
    power_off_drag="powerOffDragCurve.csv",
    power_on_drag="powerOnDragCurve.csv",
    center_of_mass_without_motor=1.89, #1.89
    coordinate_system_orientation="tail_to_nose",
)

# Rails
buttons = Cimarron.set_rail_buttons(
    upper_button_position=1.35,
    lower_button_position=0.029,
    angular_position=90,
)

Cimarron.add_motor(AeroTech_M2500T, position=-0.08)

# Ojiva
nose = Cimarron.add_nose(
    length=0.648, kind="vonKarman", position=3.535004
)

# Aletas
fins = Cimarron.add_trapezoidal_fins(
    n=4,
    root_chord=0.437007,
    tip_chord=0.2015998,
    span=0.1197356,
    sweep_length=0.238252,
    cant_angle=25.54,
    position=0.6458518, #-2.89
)

tail = Cimarron.add_tail(
    top_radius=0.0649986, bottom_radius=0.047498, length=0.094996, position=0 #-3.53
)

# Paracaidas
main = Cimarron.add_parachute(
    name="main",
    cd_s=1.5,
    trigger=381,  # ejection altitude in meters
    sampling_rate=100,
    lag=1.5,
    noise=(0, 8.3, 0.5),
)

drogue = Cimarron.add_parachute(
    name="drogue",
    cd_s=1.5,
    trigger="apogee",  # ejection at apogee
    sampling_rate=100,
    lag=1.5,
    noise=(0, 8.3, 0.5),
)

# Airbrakes
def controller_function(
    time, sampling_rate, state, state_history, observed_variables, air_brakes
):
    # state = [x, y, z, vx, vy, vz, e0, e1, e2, e3, wx, wy, wz]
    altitude_ASL = state[2]
    altitude_AGL = altitude_ASL - env.elevation
    vx, vy, vz = state[3], state[4], state[5]

    # Get winds in x and y directions
    wind_x, wind_y = env.wind_velocity_x(altitude_ASL), env.wind_velocity_y(altitude_ASL)

    # Calculate Mach number
    free_stream_speed = (
        (wind_x - vx) ** 2 + (wind_y - vy) ** 2 + (vz) ** 2
    ) ** 0.5
    mach_number = free_stream_speed / env.speed_of_sound(altitude_ASL)

    # Get previous state from state_history
    previous_state = state_history[-1]
    previous_vz = previous_state[5]

    # If we wanted to we could get the returned values from observed_variables:
    returned_time, deployment_level, drag_coefficient = observed_variables[-1]

    # Check if the rocket has reached burnout
    if time < AeroTech_M2500T.burn_out_time:
        return None

    # If below 65% of the total altitude (3048m), air_brakes are not deployed
    if altitude_AGL < 1981.2:
        air_brakes.deployment_level = 0

    # Else calculate the deployment level
    else:
        # Controller logic
        new_deployment_level = (
            air_brakes.deployment_level + 0.1 * vz + 0.01 * previous_vz**2
        )

        # Limiting the speed of the air_brakes to 0.2 per second
        # Since this function is called every 1/sampling_rate seconds
        # the max change in deployment level per call is 0.2/sampling_rate
        max_change = 0.5 / sampling_rate
        lower_bound = air_brakes.deployment_level - max_change
        upper_bound = air_brakes.deployment_level + max_change
        new_deployment_level = min(max(new_deployment_level, lower_bound), upper_bound)

        air_brakes.deployment_level = new_deployment_level

    # Return variables of interest to be saved in the observed_variables list
    return (
        time,
        air_brakes.deployment_level,
        air_brakes.drag_coefficient(air_brakes.deployment_level, mach_number),
    )

air_brakes = Cimarron.add_air_brakes(
    drag_coefficient_curve="air_brakes_cd.csv",
    controller_function=controller_function,
    sampling_rate=10,
    reference_area=0.03035441382, #0.03035441382
    clamp=True,
    initial_observed_variables=[0, 0, 0],
    override_rocket_drag=False,
    name="Air Brakes",
)

air_brakes.all_info()

Cimarron_flight = Flight(
    rocket=Cimarron,
    environment=env,
    rail_length=5.18,
    inclination=86,
    heading=0,
    time_overshoot=False,
    terminate_on_apogee=False,
)

# Informacion Airbrakes
import matplotlib.pyplot as plt

time_list, deployment_level_list, drag_coefficient_list = [], [], []

obs_vars = Cimarron_flight.get_controller_observed_variables()

for time, deployment_level, drag_coefficient in obs_vars:
    time_list.append(time)
    deployment_level_list.append(deployment_level)
    drag_coefficient_list.append(drag_coefficient)

# Plot deployment level by time
plt.plot(time_list, deployment_level_list)
plt.xlabel("Time (s)")
plt.ylabel("Deployment Level")
plt.title("Deployment Level by Time")
plt.grid()
plt.show()

# Plot drag coefficient by time
plt.plot(time_list, drag_coefficient_list)
plt.xlabel("Time (s)")
plt.ylabel("Drag Coefficient")
plt.title("Drag Coefficient by Time")
plt.grid()
plt.show()

Cimarron_flight.prints.burn_out_conditions()
Cimarron_flight.prints.apogee_conditions()
Cimarron_flight.altitude()
Cimarron_flight.vz()

# Informacion del cohete
Cimarron.draw()

# Informacion y graficas
Cimarron_flight.all_info()

# Archivo para google earth
Cimarron_flight.export_kml(file_name="Cimarron_flight.kml")