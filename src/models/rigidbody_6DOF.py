import numpy as np
import navpy
from .geomag import igrf
from datetime import datetime, timedelta
from scipy.stats import norm

import copy


def euler2quat(euler):
    # input order = X, Y, Z
    # rotation sequence = Z, Y, X
    phi, the, psi = euler
    cphi = np.cos(phi / 2)
    sphi = np.sin(phi / 2)
    cthe = np.cos(the / 2)
    sthe = np.sin(the / 2)
    cpsi = np.cos(psi / 2)
    spsi = np.sin(psi / 2)
    return np.array([
        cphi * cthe * cpsi + sphi * sthe * spsi,
        sphi * cthe * cpsi - cphi * sthe * spsi,
        cphi * sthe * cpsi + sphi * cthe * spsi,
        cphi * cthe * spsi - sphi * sthe * cpsi
    ])


def quat2euler(q):
    q0, q1, q2, q3 = q
    return np.array([
        np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)),
        np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)),
        np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2)),
    ])


def quat2dcm(q):
    s = np.dot(q, q) * 2
    qr, qi, qj, qk = q
    return np.array([
        [1 - s * (qj ** 2 + qk ** 2), s * (qi * qj + qk * qr), s * (qi * qk - qj * qr)],
        [s * (qi * qj - qk * qr), 1 - s * (qi ** 2 + qk ** 2), s * (qj * qk + qi * qr)],
        [s * (qi * qk + qj * qr), s * (qj * qk - qi * qr), 1 - s * (qi ** 2 + qj ** 2)]
    ])


def force_and_moment_example(t, x):
    F = np.array([0., 0., 0.])
    M = np.array([0., 0., 0.])

    if 1.0 <= t < 2.0:
        F[1] = 1.
    else:
        F[1] = 0.

    if 7.0 <= t < 8.0:
        M[0] = 1.
    else:
        M[0] = 0.
    return F, M


class SimData:
    def to_numpy(self):
        for key in self.__dict__:
            if type(self.__dict__[key]) is not np.ndarray:
                self.__dict__[key] = np.array(self.__dict__[key])


class RigidBody6DOF:
    def __init__(self, sim_result: SimData, params: dict):
        """
        어떠한 강체에 대해 초기 위치값 (initial_state)과 시간, 그리고 시간에 따른 외부 힘(힘 + 모멘텀)을 입력하면, 강체가 어떻게 움직이는지 나타내주는
        시뮬레이터다.

        :param sim_result: 시뮬레이션 결과들을 저장하는 클래스
        :param params: 시뮬레이션을 돌릴 때 주는 초기 값들
        """
        self.g = 9.81  # [m/s^2]
        self.m = params['mass']
        self.I = params['inertia']
        self.Iinv = params['inverse of inertia']
        self.kappa = params['gain for quaternion normalization']
        self.h = params['time interval']
        self.force_and_moment = None

        self.rec_internal_var = False
        self.sim_result = sim_result
        self.sim_result.F = []
        self.sim_result.M = []
        self.sim_result.t = []
        self.sim_result.x = []
        self.sim_result.x_dot = []

    # t: time
    # x: state vector
    # F: force vector applied in the body-fixed frame
    # M: moment vector applied in the body-fixed frame
    def f(self, t, x):
        """
        시간과 강체 정보가 주어지면, 다음 시간에서의 강체 정보를 계산해서 반환한다.

        :param t: time
        :param x: 강체 정보, [x, y, z, v_x, v_y, v_z, w_x, w_y, w_z, q0, q1, q2, q3]
        :return: 주어진 강체 정보를 기반으로 강체가 받는 힘, 모멘텀, 속도를 계산해서 반환한다.
        """
        F, M = self.force_and_moment(t, x)

        # x = [x, y, z, v_x,
        # S = x[0:3]
        V = x[3:6]
        W = x[6:9]
        Q = x[9:13]

        DCM = quat2dcm(Q)

        Sdot = DCM.T @ V

        Vdot = F / self.m - np.cross(W, V)

        Wdot = self.Iinv @ (M - np.cross(W, self.I @ W))

        p, q, r = W
        eps = 1. - np.linalg.norm(Q)
        Qdot = 0.5 * np.array([[0., -p, -q, -r],
                               [p, 0., r, -q],
                               [q, -r, 0., p],
                               [r, q, -p, 0.]]) @ Q + self.kappa * eps * Q

        x_dot = np.hstack((Sdot, Vdot, Wdot, Qdot))
        if self.rec_internal_var:
            self.sim_result.t.append(t)
            self.sim_result.x.append(x)
            self.sim_result.x_dot.append(x_dot)
            self.sim_result.F.append(F)
            self.sim_result.M.append(M)

        return x_dot

    def rk4(self, t0, x0):
        """
        Runge-Kutta methods를 이용하여 timestep 이후 강체의 정보값 (t1, x1)을 반환한다.
        f 함수에서 강체가 순간 시간에 받는 힘을 계산하고, 그 힘으로 인해 바뀐 강체의 위치 및 정보를 반환한다.

        Runge-Kutta methods: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods

        :param t0: 초기 시간값
        :param x0: 초기 시간에서의 강체 정보
        :return: timestep 이후 강체의 정보값 (t1, x1)
        """
        self.rec_internal_var = True
        k1 = self.f(t0, x0)
        self.rec_internal_var = False
        k2 = self.f(t0 + self.h / 2., x0 + (k1 * self.h) / 2.)
        k3 = self.f(t0 + self.h / 2., x0 + (k2 * self.h) / 2.)
        k4 = self.f(t0 + self.h, x0 + (k3 * self.h))
        x1 = x0 + self.h / 6. * (k1 + (2 * k2) + (2 * k3) + k4)
        t1 = t0 + self.h
        return t1, x1

    def run_sim(self, initial_state, timespan=None, maneuver_func=None):
        """
        어떤 강체에 대해 초기값과 시간, 힘의 변화량이 주어졌을 때, 시간에 따른 강체의 움직임과 정보를 반환한다.

        :param initial_state: 강체의 초기 정보값
        :param timespan: 시뮬레이션을 할 시간값, [시작시간, 끝시간]
        :param maneuver_func: 시간당 주어진 힘, 모멘텀 값
        :return: 시간에 따른 강체의 정보값
        """
        if timespan is None:
            timespan = [0.0, 10.0]
        if maneuver_func is None:
            self.force_and_moment = force_and_moment_example
        else:
            self.force_and_moment = maneuver_func

        x = np.array(initial_state)
        t = timespan[0]
        t_end = timespan[1]  # terminal time

        # calc. states
        while t < t_end:
            t, x = self.rk4(t, x)
        self.sim_result.to_numpy()

        # calc. additional info
        self.sim_result.euler = list(map(quat2euler, self.sim_result.x[:, 9:13]))
        self.sim_result.to_numpy()


class Accelerometer:

    def __init__(self, sim_result, params):
        """
        가속도계 모사 장치. simulation 을 만든 시간에 따른 강체의 움직임 정보를 받으면, 가속도계는 어떤 값을 뱉는지 모사하는 함수이다.
        계산식에 대한 설명은 컨플루언스에 정리되어 있다. (https://42dot.atlassian.net/wiki/spaces/UFII/pages/2471198899)

        :param sim_result: 시뮬레이션 결과
        :param params: 가속도계 초기 세팅값
        """
        self.d = params['lever_arm']
        self.a_sf_cc = params['acc_scale_factor'] * params['acc_cross_coupling']
        self.a_bias = params['acc_bias']
        self.g_ned = np.array([0, 0, params['gravity_constant']])
        self.sat_lower = params['acc_saturaion_lowerlimit']
        self.sat_upper = params['acc_saturaion_upperlimit']
        self.acc_noise_stddev = params['acc_noise_pow'] * self.g_ned * 1e-6 / np.sqrt(params['time interval'])
        self.install_att = params['acc_installation_attitude']  # phi, the, psi[rad] Body -> Sensor

        self.sim_result = sim_result
        self.sim_result.g = []
        self.gen_data()

    def gen_data(self):
        sr = self.sim_result
        # calc. g
        Q = sr.x[:, 9:13]
        for q in Q:
            sr.g.append(quat2dcm(q) @ self.g_ned)

        # calc. A_meas
        sr.Ab_meas_ideal_no_g = []  # meas: measurement
        for Ab, w, wdot in zip(sr.x_dot[:, 3:6],
                               sr.x[:, 6:9],
                               sr.x_dot[:, 6:9]):  # TODO: (수현) wdot은 x_dot에서 추출해야 되지 않나요? x -> x_dot
            sr.Ab_meas_ideal_no_g.append(
                Ab + np.cross(w, np.cross(w, self.d)) + np.cross(wdot, self.d)
            )
        sr.to_numpy()

        R_B2S = quat2dcm(euler2quat(self.install_att))
        sr.Ab_meas_ideal_no_install_att = sr.Ab_meas_ideal_no_g + sr.g  # 가속도계 설치 각도를 고려하지 않았을 때
        sr.Ab_meas_ideal = (R_B2S @ sr.Ab_meas_ideal_no_install_att.T).T

        sr.acc_noise = np.random.normal(scale=self.acc_noise_stddev, size=sr.Ab_meas_ideal.shape)
        sr.Ab_meas_noisy = np.clip(
            sr.Ab_meas_ideal @ self.a_sf_cc + self.a_bias + sr.acc_noise,
            self.sat_lower,
            self.sat_upper
        )


class Gyrometer:
    def __init__(self, sim_result, params):
        """
        자이로센서 모사 장치. simulation 을 만든 시간에 따른 강체의 움직임 정보를 받으면, 자이로센서는 어떤 값을 뱉는지 모사하는 함수이다.
        계산식에 대한 설명은 컨플루언스에 정리되어 있다. (https://42dot.atlassian.net/wiki/spaces/UFII/pages/2471198899)

        :param sim_result: 시뮬레이션 결과
        :param params: 자이로센서 초기 세팅값
        """
        self.g_sen = params['gyro_acc_sensitive_bias']
        self.g_sf_cc = params['gyro_scale_factor'] * params['gyro_cross_coupling']
        self.g_bias = params['gyro_bias']
        self.gyro_noise_stddev = params['gyro_noise_pow'] * 1e-3 * np.pi / 180 / np.sqrt(params['time interval'])
        self.sat_lower = params['gyro_saturaion_lowerlimit']
        self.sat_upper = params['gyro_saturaion_upperlimit']
        self.install_att = params['gyro_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

        self.sim_result = sim_result
        self.gen_data()

    def gen_data(self):
        sr = self.sim_result

        R_B2S = quat2dcm(euler2quat(self.install_att))
        sr.W_meas_ideal_no_install_att = sr.x[:, 6:9]
        sr.W_meas_ideal = (R_B2S @ sr.W_meas_ideal_no_install_att.T).T

        sr.gyro_noise = np.random.normal(scale=self.gyro_noise_stddev, size=sr.W_meas_ideal.shape)

        sr.W_meas_noisy = np.clip(
            sr.W_meas_ideal @ self.g_sf_cc + self.g_bias + np.multiply(sr.Ab_meas_ideal, self.g_sen) + sr.gyro_noise,
            self.sat_lower,
            self.sat_upper
        )


class Magnetometer:
    """
    지자기 센서 모사 장치. simulation 을 만든 시간에 따른 강체의 움직임 정보를 받으면, 지자기 센서는 어떤 값을 뱉는지 모사하는 함수이다.
    계산식에 대한 설명은 컨플루언스에 정리되어 있다. (https://42dot.atlassian.net/wiki/spaces/UFII/pages/2471198899)

    :param sim_result: 시뮬레이션 결과
    :param params: 지자기 센서 초기 세팅값
    """
    def __init__(self, sim_result, params):
        self.mag_cass = params['mag_cross_axis_sensitivity'] * 0.01
        self.mag_lin = params['mag_linearity'] * 0.01
        self.init_lla = params['init_LLA']
        self.init_time = datetime.strptime(params['init_time'], '%Y-%m-%d %H:%M:%S')
        self.igrf = igrf.GeoMag('WMM2020.COF')
        self.t_datetime = None
        self.sim_result = sim_result
        self.install_att = params['mag_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

        self.gen_data()

    def gen_data(self):
        sr = self.sim_result
        # calc. LLA
        sr.lla = np.array(
            navpy.ned2lla(
                sr.x[:, 0:3],
                self.init_lla[0], self.init_lla[1], self.init_lla[2]
            )
        ).T

        # calc. magnet. vector
        t_datetime = [self.init_time + t * timedelta(seconds=1) for t in sr.t]

        R_B2S = quat2dcm(euler2quat(self.install_att))
        sr.mag_meas_ideal_no_install_att = self.igrf.geomag_xyz_iter(sr.lla, t_datetime)
        sr.mag_meas_ideal = (R_B2S @ sr.mag_meas_ideal_no_install_att.T).T

        sr.mag_cass_noise = np.random.uniform(-self.mag_cass, self.mag_cass, [len(sr.mag_meas_ideal), 6])

        sr.mag_lin_noise = 1 + np.random.uniform(-self.mag_lin, self.mag_lin, sr.mag_meas_ideal.shape)

        sr.mag_noisy = copy.deepcopy(sr.mag_meas_ideal)

        sr.mag_noisy[:, 0] += np.multiply(sr.mag_cass_noise[:, 0], sr.mag_meas_ideal[:, 1])
        sr.mag_noisy[:, 0] += np.multiply(sr.mag_cass_noise[:, 1], sr.mag_meas_ideal[:, 2])

        sr.mag_noisy[:, 1] += np.multiply(sr.mag_cass_noise[:, 2], sr.mag_meas_ideal[:, 0])
        sr.mag_noisy[:, 1] += np.multiply(sr.mag_cass_noise[:, 3], sr.mag_meas_ideal[:, 2])

        sr.mag_noisy[:, 2] += np.multiply(sr.mag_cass_noise[:, 4], sr.mag_meas_ideal[:, 0])
        sr.mag_noisy[:, 2] += np.multiply(sr.mag_cass_noise[:, 5], sr.mag_meas_ideal[:, 1])

        sr.mag_noisy = np.multiply(sr.mag_noisy, sr.mag_lin_noise)


def coef_cep_n(percent):
    return norm.ppf(0.5 + percent / 200)  # ppf: percent point function


class GPS:
    def __init__(self, sim_result, params):
        """
        GPS 센서 모사 장치. simulation 을 만든 시간에 따른 강체의 움직임 정보를 받으면, GPS는 어떤 값을 뱉는지 모사하는 함수이다.
        계산식에 대한 설명은 컨플루언스에 정리되어 있다. (https://42dot.atlassian.net/wiki/spaces/UFII/pages/2471198899)

        :param sim_result: 시뮬레이션 결과
        :param params: GPS 초기 세팅값
        """
        self.sim_result = sim_result
        self.coef_cep50 = coef_cep_n(50)
        self.horizontal_error_stddev = params['GPS_horizontal_position_accuracy'] / self.coef_cep50
        self.vertical_stddev = params['GPS_vertical_position_accuracy'] / self.coef_cep50
        self.vel_accuracy = params['GPS_velocity_accuracy']
        self.head_accuracy = params['GPS_heading_accuracy']
        self.gen_data()

    def gen_data(self):
        sr = self.sim_result

        horizontal_noise = np.random.normal(scale=self.horizontal_error_stddev, size=[len(sr.t), 2])
        vertical_noise = np.random.normal(scale=self.vertical_stddev, size=[len(sr.t), 1])

        sr.gps_pos_ideal = sr.x[:, 0:3]
        sr.gps_pos_noisy = copy.deepcopy(sr.gps_pos_ideal)
        sr.gps_pos_noisy[:, 0:2] += horizontal_noise
        sr.gps_pos_noisy[:, 2:3] += vertical_noise

        sr.gps_vel_ideal = np.linalg.norm(sr.x[:, 3:6], axis=1)
        sr.gps_vel_noisy = sr.gps_vel_ideal + np.random.normal(scale=self.vel_accuracy,
                                                               size=sr.gps_vel_ideal.shape)

        sr.gps_head_ideal = sr.euler[:, 2:3]
        sr.gps_head_meas = sr.gps_head_ideal * 180 / np.pi + np.random.normal(scale=self.head_accuracy,
                                                                              size=sr.gps_head_ideal.shape)
        sr.gps_head_meas = np.mod(sr.gps_head_meas, 360)


class Barometer:
    def __init__(self, sim_result, params):
        """
        기압센서 센서 모사 장치. simulation 을 만든 시간에 따른 강체의 움직임 정보를 받으면, 기압센서는 어떤 값을 뱉는지 모사하는 함수이다.
        계산식에 대한 설명은 컨플루언스에 정리되어 있다. (https://42dot.atlassian.net/wiki/spaces/UFII/pages/2471198899)

        :param sim_result: 시뮬레이션 결과
        :param params: 기압센서 초기 세팅값
        """
        self.sim_result = sim_result
        self.R = params['gas_constant']
        self.M = params['air_molar_mass']
        self.T0 = params['local_ref_temperature'] + 273.15
        self.p0 = params['local_ref_pressure']
        self.g = params['gravity_constant']
        self.c_sold = params['baro_solder_drift'] / 100
        self.c_rel_acc = params['baro_relative_accuracy'] / 100
        self.c_stab_short = params['baro_short_term_drift'] / 100
        self.c_stab_long = params['baro_long_term_drift'] / 100
        self.b_abs_acc = params['baro_absolute_accuracy'] / 100
        self.sig_noise = params['baro_noise_stddev'] / 100

        self.n_offset = np.random.uniform(-self.c_sold, self.c_sold)
        self.n_drift_coef = np.random.uniform(-self.c_stab_short, self.c_stab_short) / (3600 * 24) \
                            + np.random.uniform(-self.c_stab_long, self.c_stab_long) / (3600 * 24 * 365)

        self.gen_data()

    def gen_data(self):
        sr = self.sim_result
        exp_const = - (self.g * self.M) / (self.R * self.T0)
        sr.baro_p_ideal = self.p0 * np.exp(exp_const * sr.lla[:, 2] / 1000)
        sat_noise = self.n_offset \
                    + self.n_drift_coef * sr.t \
                    + np.random.uniform(-self.c_rel_acc, self.c_rel_acc, sr.baro_p_ideal.shape)
        sat_noise = np.clip(sat_noise, -self.b_abs_acc, self.b_abs_acc)
        sr.baro_p_noisy = sr.baro_p_ideal + sat_noise + np.random.normal(scale=self.sig_noise,
                                                                         size=sr.baro_p_ideal.shape)


if __name__ == "__main__":
    # parameters for 6dof
    params = {}
    params['mass'] = 1.  # [kg]
    params['inertia'] = np.eye(3)
    params['inverse of inertia'] = np.linalg.inv(params['inertia'])
    params['gain for quaternion normalization'] = 1.
    params['time interval'] = 0.001  # [s]
    params['init_LLA'] = [37.4849885, 127.0358544, 5]  # [deg], [deg], [m]
    params['init_time'] = '2020-12-01 01:23:45'  # '%Y-%m-%d %H:%M:%S'
    params['gravity_constant'] = 9.81  # [m/s^2]

    # parameters for acc. sensor
    params['lever_arm'] = np.array([0, 1, 0])  # [m]
    params['acc_scale_factor'] = 1
    params['acc_cross_coupling'] = np.eye(3)
    params['acc_bias'] = np.array([0, 0, 0])  # [m/s^2]
    params['acc_noise_pow'] = 120  # [ug/sqrt(Hz)]
    params['acc_saturaion_upperlimit'] = np.inf  # [m/s^2]
    params['acc_saturaion_lowerlimit'] = -np.inf  # [m/s^2]
    params['acc_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

    # parameters for gyro. sensor
    params['gyro_scale_factor'] = 1
    params['gyro_cross_coupling'] = np.eye(3)
    params['gyro_bias'] = np.array([0, 0, 0])  # [rad/s]
    params['gyro_acc_sensitive_bias'] = np.array([0, 0, 0])  # [rad/s]
    params['gyro_noise_pow'] = 3.8  # [mdps/sqrt(Hz)]
    params['gyro_saturaion_upperlimit'] = np.inf
    params['gyro_saturaion_lowerlimit'] = -np.inf
    params['gyro_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

    # parameters for mag. sensor
    params['mag_cross_axis_sensitivity'] = 0.2  # [%FS/gauss]
    params['mag_linearity'] = 0.1  # [%FS/gauss]
    params['mag_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

    # parameters for GPS
    params['GPS_horizontal_position_accuracy'] = 1.5  # [m], CEP50
    params['GPS_vertical_position_accuracy'] = 3  # [m], CEP50
    params['GPS_velocity_accuracy'] = 0.08  # [m/s], 1-sigma
    params['GPS_heading_accuracy'] = 0.3  # [deg], 1-sigma

    # parameters for barometer
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

    sim_result = SimData()
    sim = RigidBody6DOF(sim_result, params)

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

    acc = Accelerometer(sim_result, params)
    gyro = Gyrometer(sim_result, params)
    mag = Magnetometer(sim_result, params)
    gps = GPS(sim_result, params)
    baro = Barometer(sim_result, params)

    # sim result abstract
    print('sim_result contains')
    for key in sim_result.__dict__:
        print(f"{key} : {sim_result.__dict__[key].shape}")
