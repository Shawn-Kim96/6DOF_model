# simulator_6dof_model_and_sensors

This simulates sensor data collected from various sensors installed on a vehicle during driving.

It is based on a 6-DOF forced motion equation. The motion state calculated from the motion equation is considered the true value, and the sensor measurement equations and noise models are applied to extract the measured values based on this true value.


## Environment Setup
- Python version: 3.9.10
- Install the required libraries using pip install -r requirements.txt

## Simulation Workflow
`RigidBody6DOF` is the class that contains the motion equations and integration methods needed to run the simulation. `SimData` is the class used to store simulation data.

The simulation workflow first simulates the rigid body's motion using `RigidBody6DOF`, and then recalculates what values are measured by the sensor models. Due to this structure, it is not possible to implement simulations where sensor values with noise are filtered to estimate the motion state and control the system based on these estimates. However, it is possible to implement logic that controls forces and moments by feeding back the ideal motion state without noise, by customizing the `maneuver_func`.

The following code demonstrates how to run the simulation, and the parameters are explained in the next section.

```python
# init classes
sim_result = SimData()
sim = RigidBody6DOF(sim_result, params)

# run simulation
sim.run_sim(
    initial_state=np.hstack(
        (
            [0., 0., 0.],  # position
            [0., 0., 0.],  # velocity
            [0., 0., 0.],  # angular velocity
            euler2quat([0.1, 0.2, 0.3])  # quaternion (from euler angle )
        )
    ),
    timespan=[0, 10],
    maneuver_func=force_and_moment_example
)

# calc sensor measurement
acc = Accelerometer(sim_result, params)
gyro = Gyrometer(sim_result, params)
mag = Magnetometer(sim_result, params)
gps = GPS(sim_result, params)
baro = Barometer(sim_result, params)

# sim result abstract
print('sim_result contains')
for key in sim_result.__dict__:
    print(f"{key} : {sim_result.__dict__[key].shape}")
```


## 6-DOF Model
- Input: Forces, moments
- Output: Displacement, velocity, acceleration, orientation, angular velocity, angular acceleration

Model Parameters
```python
params['mass'] = 1.  # [kg]
params['inertia'] = np.eye(3)
params['gain for quaternion normalization'] = 1.
params['time interval'] = 0.001  # [s]
params['init_LLA'] = [37.4849885, 127.0358544, 5]  # [deg], [deg], [m]
params['init_time'] = '2020-12-01 01:23:45'  # '%Y-%m-%d %H:%M:%S'
params['gravity_constant'] = 9.81  # [m/s^2]
```

## Sensor Models
- Accelerometer, Gyroscope, Magnetometer, GPS, Barometer

### Accelerometer Model Parameters
```python
params['lever_arm'] = np.array([0, 1, 0])  # [m]
params['acc_scale_factor'] = 1
params['acc_cross_coupling'] = np.eye(3)
params['acc_bias'] = np.array([0, 0, 0])  # [m/s^2]
params['acc_noise_pow'] = 120  # [ug/sqrt(Hz)]
params['acc_saturaion_upperlimit'] = np.inf  # [m/s^2]
params['acc_saturaion_lowerlimit'] = -np.inf  # [m/s^2]
params['acc_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor
```

### Gyroscope Model Parameters
```python
params['gyro_scale_factor'] = 1
params['gyro_cross_coupling'] = np.eye(3)
params['gyro_bias'] = np.array([0, 0, 0])  # [rad/s]
params['gyro_acc_sensitive_bias'] = np.array([0, 0, 0])  # [rad/s]
params['gyro_noise_pow'] = 3.8  # [mdps/sqrt(Hz)]
params['gyro_saturaion_upperlimit'] = np.inf
params['gyro_saturaion_lowerlimit'] = -np.inf
params['gyro_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor
```

### Magnetometer Model Parameters
```python
params['mag_cross_axis_sensitivity'] = 0.2  # [%FS/gauss]
params['mag_linearity'] = 0.1  # [%FS/gauss]
params['mag_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor
```

### GPS Model Parameters
```python
params['GPS_horizontal_position_accuracy'] = 1.5  # [m], CEP50
params['GPS_vertical_position_accuracy'] = 3  # [m], CEP50
params['GPS_velocity_accuracy'] = 0.08  # [m/s], 1-sigma
params['GPS_heading_accuracy'] = 0.3  # [deg], 1-sigma
```

### Barometer Model Parameters
```python
params['gas_constant'] = 8.3145  # [J / mol K]
params['air_molar_mass'] = 28.97  # [g / mol]
params['local_ref_temperature'] = 25  # [Celsius]
params['local_ref_pressure'] = 1013.25  # [hPa]
params['baro_solder_drift'] = 30  # [Pa]
params['baro_relative_accuracy'] = 6  # [Pa]
params['baro_short_term_drift'] = 1.5  # [Pa]
params['baro_long_term_drift'] = 10  # [Pa]
params['baro_absolute_accuracy'] = 30  # [Pa]
params['baro_noise_stddev'] = 0.95  # [PaRMS]
```
