"""Offline source/config unit checks for the original 250 Hz Iris IMU.

The native preintegrator uses Q = V diag(N**2, W**2) V.T, with
Vmeasurement = .5*dt for each midpoint endpoint and Vbias = dt.  Therefore
N is a per-sample standard deviation and W is a per-step bias-rate standard
deviation.  The Gazebo plugin's input parameters are continuous densities.
These tests do not claim that independent midpoint endpoints capture the
cross-step reuse of the actual sensor samples or that flight quality passed.
"""
import math
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def source_parameters():
    model=ET.parse(ROOT/'vio/models/iris_vio/iris_vio.sdf').getroot()
    plugin=next(p for p in model.iter('plugin') if p.get('filename')=='libgazebo_imu_plugin.so')
    world=ET.parse(ROOT/'scene/worlds/obstacle_yard.world').getroot()
    dt=float(next(world.iter('max_step_size')).text)
    return dt,{child.tag:float(child.text) for child in plugin
               if child.tag.endswith(('NoiseDensity','RandomWalk','BiasCorrelationTime','TurnOnBiasSigma'))}


def configuration(path=None):
    raw=(path or ROOT/'vio/config/stereo_imu.yaml').read_text()
    return {name:float(re.search(r'^'+name+r':\s*([^\s#]+)',raw,re.M).group(1))
            for name in ('acc_n','gyr_n','acc_w','gyr_w','acc_bias_prior_sigma','gyr_bias_prior_sigma')}


def source_bias_variance(dt,random_walk,tau):
    return random_walk**2 * tau/2 * -math.expm1(-2*dt/tau)


class VinsImuNoiseModelTests(unittest.TestCase):
    def assertRelative(self,observed,expected):
        self.assertAlmostEqual(observed/expected,1.,places=6)

    def test_nominal_source_sample_interval_is_four_sim_milliseconds(self):
        dt,_=source_parameters();self.assertEqual(dt,.004)

    def test_config_white_noise_matches_actual_source_sample_variance(self):
        dt,source=source_parameters();config=configuration()
        for name,prefix in (('acc_n','accelerometer'),('gyr_n','gyroscope')):
            with self.subTest(name=name):
                self.assertRelative(config[name]**2,source[prefix+'NoiseDensity']**2/dt)

    def test_config_native_bias_step_variance_matches_exact_gazebo_ou_increment(self):
        dt,source=source_parameters();config=configuration()
        for name,prefix in (('acc_w','accelerometer'),('gyr_w','gyroscope')):
            with self.subTest(name=name):
                expected=source_bias_variance(dt,source[prefix+'RandomWalk'],source[prefix+'BiasCorrelationTime'])
                self.assertRelative((config[name]*dt)**2,expected)

    def test_initial_bias_prior_matches_source_turn_on_distribution(self):
        _,source=source_parameters();config=configuration()
        self.assertEqual(config['acc_bias_prior_sigma'],source['accelerometerTurnOnBiasSigma'])
        self.assertEqual(config['gyr_bias_prior_sigma'],source['gyroscopeTurnOnBiasSigma'])

    def test_asset_generator_preserves_the_source_calibration_when_regenerating(self):
        dt,source=source_parameters();config=configuration(ROOT/'vio/generate_assets.py')
        for prefix,measurement,bias in (('accelerometer','acc_n','acc_w'),('gyroscope','gyr_n','gyr_w')):
            with self.subTest(prefix=prefix):
                self.assertRelative(config[measurement]**2,source[prefix+'NoiseDensity']**2/dt)
                expected=source_bias_variance(dt,source[prefix+'RandomWalk'],source[prefix+'BiasCorrelationTime'])
                self.assertRelative((config[bias]*dt)**2,expected)

    def test_prior_density_directly_used_as_native_bias_rate_understates_variance_250_fold(self):
        dt,source=source_parameters()
        for prefix in ('accelerometer','gyroscope'):
            q=source[prefix+'RandomWalk']
            ratio=source_bias_variance(dt,q,source[prefix+'BiasCorrelationTime'])/(q*dt)**2
            self.assertGreater(ratio,249.99);self.assertLess(ratio,250.)

    def test_midpoint_endpoint_reuse_remains_an_explicit_multistep_covariance_limit(self):
        # Trapezoidal integration of n+1 independent raw samples reuses every
        # internal sample twice. Native diagonal-Q step summation omits that
        # correlation even when its individual endpoint variance is calibrated.
        n=25;dt,_=source_parameters();sample_variance=.029409182239565932**2
        actual=sample_variance*dt**2*(n-.5)
        native_independent_endpoints=sample_variance*dt**2*n*.5
        self.assertAlmostEqual(actual/native_independent_endpoints,1.96)
        self.assertGreater(actual,native_independent_endpoints)


if __name__=='__main__':unittest.main()
