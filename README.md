# simulator_6dof_model_and_sensors

차량에 각종 센서를 장착하고 주행할 때 획득되는 센서데이터를 시뮬레이션한다.

6-DOF 강제 운동 방정식을 기반으로 하였다. 
운동방정식으로부터 계산되는 운동상태를 참값으로 하고, 
이 값을 기준으로 센서의 측정 방정식과 노이즈 모델에 적용하여 측정값을 추출하도록 구성하였다.


## 환경설정
- python version : 3.9.10
- 라이브러리는 `pip install -r requirements.txt` 로 설치

## 시뮬레이션 구동 방식
RigidBody6DOF는 시뮬레이션이 구동되는 데에 필요한 운동방정식, 적분 방법을 담고 있는 클래스이며, 
SimData는 시뮬레이션 데이터를 담기 위한 클래스이다.

RigidBody6DOF를 통하여 강체의 운동을 먼저 시뮬레이션 한 다음, 센서 모델들을 통하여 어떤 값이 측정되는지 재계산하는 순서로 구성이 된다.
이러한 구조로 인해 노이즈가 포함된 센서값을 필터링하여 운동상태값을 추정하고 이를 기반으로 제어하는 방식의 시뮬레이션은 구현할 수 없다. 
다만 노이즈 없는 이상적인 운동 상태를 피드백하여 힘과 모멘트를 제어하는 로직은 maneuver_func을 customize함으로써 구현이 가능하다.

시뮬레이션을 위한 예시 코드는 다음과 같으며, 파라미터에 대한 설명은 다음 절에서 상세히 설명한다.



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



## 6-DOF 모델
- 입력 : 힘, 모멘트
- 출력 : 이동 거리, 속도, 가속도, 자세, 각속도, 각가속도
- 모델 파라미터
```
    params['mass'] = 1.  # [kg]
    params['inertia'] = np.eye(3)
    params['gain for quaternion normalization'] = 1.
    params['time interval'] = 0.001  # [s]
    params['init_LLA'] = [37.4849885, 127.0358544, 5]  # [deg], [deg], [m]
    params['init_time'] = '2020-12-01 01:23:45'  # '%Y-%m-%d %H:%M:%S'
    params['gravity_constant'] = 9.81  # [m/s^2]
```

## 센서 모델
- 가속도 센서, 자이로 센서, 지자기 센서, GPS, 기압 센서


### 가속도 센서 모델 파라미터
    params['lever_arm'] = np.array([0, 1, 0])  # [m]
    params['acc_scale_factor'] = 1
    params['acc_cross_coupling'] = np.eye(3)
    params['acc_bias'] = np.array([0, 0, 0])  # [m/s^2]
    params['acc_noise_pow'] = 120  # [ug/sqrt(Hz)]
    params['acc_saturaion_upperlimit'] = np.inf  # [m/s^2]
    params['acc_saturaion_lowerlimit'] = -np.inf  # [m/s^2]
    params['acc_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

### 자이로 센서 모델 파라미터
    params['gyro_scale_factor'] = 1
    params['gyro_cross_coupling'] = np.eye(3)
    params['gyro_bias'] = np.array([0, 0, 0])  # [rad/s]
    params['gyro_acc_sensitive_bias'] = np.array([0, 0, 0])  # [rad/s]
    params['gyro_noise_pow'] = 3.8  # [mdps/sqrt(Hz)]
    params['gyro_saturaion_upperlimit'] = np.inf
    params['gyro_saturaion_lowerlimit'] = -np.inf
    params['gyro_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

### 지자기 센서 모델 파라미터
    params['mag_cross_axis_sensitivity'] = 0.2  # [%FS/gauss]
    params['mag_linearity'] = 0.1  # [%FS/gauss]
    params['mag_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

### GPS 모델 파라미터
    params['GPS_horizontal_position_accuracy'] = 1.5  # [m], CEP50
    params['GPS_vertical_position_accuracy'] = 3  # [m], CEP50
    params['GPS_velocity_accuracy'] = 0.08  # [m/s], 1-sigma
    params['GPS_heading_accuracy'] = 0.3  # [deg], 1-sigma

### 기압계 모델 파라미터
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
    

## 관련 위키 페이지

- [운동방정식 & 센서 모델 기반 시뮬레이터 구축](https://42dot.atlassian.net/wiki/spaces/UFII/pages/2458320935)
- [센서 모델링](https://42dot.atlassian.net/wiki/spaces/UFII/pages/2471198899)
- [기압 모델](https://42dot.atlassian.net/wiki/spaces/UFII/pages/2476376080/WIP)