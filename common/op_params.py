#!/usr/bin/env python3
import os
import json
import time
from common.colors import opParams_error as error
from common.colors import opParams_warning as warning
from common.hardware import PC
try:
  from common.realtime import sec_since_boot
except ImportError:
  sec_since_boot = time.time
  warning("Using python time.time() instead of faster sec_since_boot")

travis = True if PC else False  # replace with travis_checker if you use travis or GitHub Actions


class ValueTypes:
  number = [float, int]
  none_or_number = [type(None), float, int]

class Param:
  def __init__(self, default, allowed_types, description=None, live=False, hidden=False, depends_on=None):
    self.default = default
    if not isinstance(allowed_types, list):
      allowed_types = [allowed_types]
    self.allowed_types = allowed_types
    self.description = description
    self.hidden = hidden
    self.live = live
    self.depends_on = depends_on
    self.children = []
    self._create_attrs()

  def is_valid(self, value):
    if not self.has_allowed_types:
      return True
    if self.is_list and isinstance(value, list):
      for v in value:
        if type(v) not in self.allowed_types:
          return False
      return True
    else:
      return type(value) in self.allowed_types or value in self.allowed_types

  def _create_attrs(self):  # Create attributes and check Param is valid
    self.has_allowed_types = isinstance(self.allowed_types, list) and len(self.allowed_types) > 0
    self.has_description = self.description is not None
    self.is_list = list in self.allowed_types
    self.is_bool = bool in self.allowed_types
    if self.has_allowed_types:
      assert type(self.default) in self.allowed_types or self.default in self.allowed_types, 'Default value type must be in specified allowed_types!'

      if self.is_list:
        for v in self.default:
          assert type(v) in self.allowed_types, 'Default value type must be in specified allowed_types!'


class opParams:
  def __init__(self):
    """
      To add your own parameter to opParams in your fork, simply add a new entry in self.fork_params, instancing a new Param class with at minimum a default value.
      The allowed_types and description args are not required but highly recommended to help users edit their parameters with opEdit safely.
        - The description value will be shown to users when they use opEdit to change the value of the parameter.
        - The allowed_types arg is used to restrict what kinds of values can be entered with opEdit so that users can't crash openpilot with unintended behavior.
              (setting a param intended to be a number with a boolean, or viceversa for example)
          Limiting the range of floats or integers is still recommended when `.get`ting the parameter.
          When a None value is allowed, use `type(None)` instead of None, as opEdit checks the type against the values in the arg with `isinstance()`.
        - Finally, the live arg tells both opParams and opEdit that it's a live parameter that will change. Therefore, you must place the `op_params.get()` call in the update function so that it can update.

      Here's an example of a good fork_param entry:
      self.fork_params = {'camera_offset': Param(default=0.06, allowed_types=VT.number)}  # VT.number allows both floats and ints
    """

    VT = ValueTypes()
    self.fork_params = {CAM_OFFSET: Param(0.06, VT.number, 'Your camera offset to use in lane_planner.py', live=True),
                        'indi_inner_gain': Param(9.0, VT.number, live=True, depends_on=SHOW_INDI_PARAMS),
                        'indi_outer_gain': Param(8.9, VT.number, live=True, depends_on=SHOW_INDI_PARAMS),
                        'indi_time_constant': Param(5.5, VT.number, live=True, depends_on=SHOW_INDI_PARAMS),
                        'indi_actuator_effectiveness': Param(9.0, VT.number, live=True, depends_on=SHOW_INDI_PARAMS),
                        SHOW_ACTUATOR_DELAY_PARAMS: Param(False, bool, live=True, depends_on=ENABLE_LAT_PARAMS),
                        STEER_ACTUATOR_DELAY: Param(0.60, VT.number, live=True, depends_on=SHOW_ACTUATOR_DELAY_PARAMS),
                        ENABLE_ACTUATOR_DELAY_BPS: Param(False, bool, live=True, depends_on=SHOW_ACTUATOR_DELAY_PARAMS),
                        STEER_ACTUATOR_DELAY_BP: Param([0.], [list, float, int], live=True, depends_on=ENABLE_ACTUATOR_DELAY_BPS),
                        STEER_ACTUATOR_DELAY_V: Param([0.6], [list, float, int], live=True, depends_on=ENABLE_ACTUATOR_DELAY_BPS),
                        'alca_nudge_required': Param(False, bool, 'Whether to wait for applied torque to the wheel (nudge) before making lane changes. '
                                                                 'If False, lane change will occur IMMEDIATELY after signaling'),
                        'alca_min_speed': Param(20.0, VT.number, 'The minimum speed allowed for an automatic lane change (in MPH)'),
                        ENABLE_COASTING: Param(False, bool, 'When true the car will try to coast down hills instead of braking.', live=True, depends_on=SHOW_EXPERIMENTAL_OPTS),
                        COAST_SPEED: Param(10.0, VT.number, 'The amount of speed to coast by before applying the brakes. Unit: MPH',
                                          live=True, depends_on=ENABLE_COASTING),
                        SETPOINT_OFFSET: Param(0, int, 'The difference between the car\'s set cruise speed and OP\'s. Unit: MPH', live=True),
                        DOWNHILL_INCLINE: Param(-1, VT.number, 'If the angle between the current road and the future predicted road is less than this value, '
                                                              'the car will try to coast downhill. Unit: degrees', live=True, depends_on=ENABLE_COASTING),
                        'corolla_use_indi': Param(False, bool),
                        'accel_hyst_gap': Param(0.02, VT.number, live=True),
                        ALWAYS_EVAL_COAST: Param(False, bool, live=True, depends_on=ENABLE_COASTING),
                        EVAL_COAST_LONG: Param(False, bool, live=True, depends_on=ENABLE_COASTING),
                        ENABLE_LONG_PARAMS: Param(False, bool, live=True, description='When true the long controller will used the params in opParam '
                                                  'instead of the car\' params'),
                        ENABLE_GAS_PARAMS: Param(True, bool, live=True, depends_on=ENABLE_LONG_PARAMS),
                        GAS_MAX_BP: Param([0., 20, 33], [list, float, int], live=True, depends_on=ENABLE_GAS_PARAMS),
                        GAS_MAX_V: Param([0.3, 0.2, 0.075], [list, float], live=True, depends_on=ENABLE_GAS_PARAMS),
                        ENABLE_BRAKE_PARAMS: Param(False, bool, live=True, depends_on=ENABLE_LONG_PARAMS),
                        BRAKE_MAX_BP: Param([0., 20, 33], [list, float, int], live=True, depends_on=ENABLE_BRAKE_PARAMS),
                        BRAKE_MAX_V: Param([0.5, 0.5, 0.5], [list, float], live=True, depends_on=ENABLE_BRAKE_PARAMS),
                        ENABLE_LONG_PID_PARAMS: Param(False, bool, live=True, depends_on=ENABLE_LONG_PARAMS),
                        LONG_PID_KP_BP: Param([0., 5., 35.], [list, float, int], live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_KP_V: Param([3.6, 2.4, 1.5], [list, float, int], live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_KI_BP: Param([0., 35.], [list, float, int], live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_KI_V: Param([0.54, 0.36], [list, float, int], live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_KF: Param(1., VT.number, live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_SAT_LIMIT: Param(0.8, VT.number, live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        ENABLE_LONG_DEADZONE_PARAMS: Param(False, bool, live=True, depends_on=ENABLE_LONG_PARAMS),
                        LONG_DEADZONE_BP: Param([0., 9.], [list, float, int], live=True, depends_on=ENABLE_LONG_DEADZONE_PARAMS),
                        LONG_DEADZONE_V: Param([0., .15], [list, float, int], live=True, depends_on=ENABLE_LONG_DEADZONE_PARAMS),
                        INDI_SHOW_BREAKPOINTS: Param(False, bool, live=True, depends_on=SHOW_INDI_PARAMS),
                        'indi_use_vego_breakpoints': Param(False, bool, live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        'indi_use_steer_angle_breakpoints': Param(False, bool, live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        'indi_inner_gain_bp': Param([0, 255, 255], [list, float, int], live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        'indi_inner_gain_v': Param([6.0, 6.0, 6.0], [list, float, int], live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        'indi_outer_gain_bp': Param([0, 255, 255], [list, float, int], live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        'indi_outer_gain_v': Param([15, 15, 15], [list, float, int], live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        'indi_time_constant_bp': Param([0, 255, 255], [list, float, int], live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        'indi_time_constant_v': Param([5.5, 5.5, 5.5], [list, float, int], live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        'indi_actuator_effectiveness_bp': Param([0, 255, 255], [list, float, int], live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        'indi_actuator_effectiveness_v': Param([6, 6, 6], [list, float, int], live=True, depends_on=INDI_SHOW_BREAKPOINTS),
                        SHOW_A_CRUISE: Param(False, bool, live=True),
                        'a_cruise_min_bp': Param([0., 5.,  10., 20.,  40.], [list, float], live=True, depends_on=SHOW_A_CRUISE),
                        'a_cruise_min_v': Param([-1.0, -.8, -.67, -.5, -.30], [list, float], live=True, depends_on=SHOW_A_CRUISE),
                        'a_cruise_min_v_following': Param([-1.0, -.8, -.67, -.5, -.30], [list, float], live=True, depends_on=SHOW_A_CRUISE),
                        'a_cruise_max_bp': Param([0.,  6.4, 22.5, 40.], [list, float], live=True, depends_on=SHOW_A_CRUISE),
                        'a_cruise_max_v': Param([1.2, 1.2, 0.65, .4], [list, float], live=True, depends_on=SHOW_A_CRUISE),
                        'a_cruise_max_v_following': Param([1.6, 1.6, 0.65, .4], [list, float], live=True, depends_on=SHOW_A_CRUISE),
                        ENABLE_UNSAFE_STEERING_RATE: Param(False, bool, depends_on=SHOW_UNSAFE_OPTS, description='Toyota only.\nThis is HIGHLY unsafe, '
                                                          'at best, you have less time to react, at worst, you\'ll have steering faults.\nDo NOT use.'),
                        ENABLE_LAT_PARAMS: Param(False, bool, live=True, description="When true, the lat params set in op_edit."),
                        WHICH_LAT_CTRL: Param('indi', ['pid', 'indi', 'lqr'], live=True, depends_on= ENABLE_LAT_PARAMS, description='Which lat controller to use, '
                                              'options are pid, indi, or lqr.'),
                        SHOW_LQR_PARAMS: Param(False, [bool], live=True, depends_on=ENABLE_LAT_PARAMS),
                        LQR_SCALE: Param(1500.0, VT.number, live=True, depends_on=SHOW_LQR_PARAMS),
                        LQR_KI: Param(0.05, VT.number, live=True, depends_on=SHOW_LQR_PARAMS),
                        LQR_A: Param([0., 1., -0.22619643, 1.21822268], [list, float, int], live=True, depends_on=SHOW_LQR_PARAMS),
                        LQR_B: Param([-1.92006585e-04, 3.95603032e-05], [list, float, int], live=True, depends_on=SHOW_LQR_PARAMS),
                        LQR_C: Param([1., 0.], [list, float, int], live=True, depends_on=SHOW_LQR_PARAMS),
                        LQR_K: Param([-110.73572306, 451.22718255], [list, float, int], live=True, depends_on=SHOW_LQR_PARAMS),
                        LQR_L: Param([0.3233671, 0.3185757], [list, float, int], live=True, depends_on=SHOW_LQR_PARAMS),
                        LQR_DC_GAIN: Param(0.002237852961363602, VT.number, live=True, depends_on=SHOW_LQR_PARAMS),
                        STEER_LIMIT_TIMER: Param(0.4, VT.number, live=True, depends_on=ENABLE_LAT_PARAMS),
                        SHOW_LAT_PID_PARAMS: Param(False, [bool], live=True, depends_on=ENABLE_LAT_PARAMS),
                        LAT_PID_KP_BP: Param([0., 5., 35.], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KP_V: Param([3.6, 2.4, 1.5], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KI_BP: Param([0., 35.], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KI_V: Param([0.54, 0.36], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KF: Param(1., VT.number, live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        SHOW_INDI_PARAMS: Param(False, [bool], live=True, depends_on=ENABLE_LAT_PARAMS),
                        SHOW_UNSAFE_OPTS: Param(False, [bool], live=True, description='Shows options for unsafe / dangerous features. '
                                                'If any of these are enabled, prepare for the worst: no steering, no gas / brake, etc.'),
                        SHOW_EXPERIMENTAL_OPTS: Param(False, [bool], live=True, description='Shows options for experimental, unfinished, features. '
                                                      'Generally you should never use these.')}

    self._params_file = '/data/op_params.json'
    self._backup_file = '/data/op_params_corrupt.json'
    self._last_read_time = sec_since_boot()
    self.read_frequency = 2.5  # max frequency to read with self.get(...) (sec)
    self._to_delete = ['lane_hug_direction', 'lane_hug_angle_offset', 'prius_use_lqr']  # a list of params you want to delete (unused)
    self._last_mod_time = 0.0
    self._run_init()  # restores, reads, and updates params

  def _run_init(self):  # does first time initializing of default params
    # Two required parameters for opEdit
    self.fork_params['username'] = Param(None, [type(None), str, bool], 'Your identifier provided with any crash logs sent to Sentry.\nHelps the developer reach out to you if anything goes wrong')
    self.fork_params['op_edit_live_mode'] = Param(False, bool, 'This parameter controls which mode opEdit starts in', hidden=True)
    self.params = self._get_all_params(default=True)  # in case file is corrupted

    for k, p in self.fork_params.items():
      d = p.depends_on
      while d:
        fp = self.fork_params[d]
        fp.children.append(k)
        d = fp.depends_on

    if travis:
      return

    to_write = False
    if os.path.isfile(self._params_file):
      if self._read():
        to_write = self._add_default_params()  # if new default data has been added
        to_write |= self._delete_old()  # or if old params have been deleted
      else:  # backup and re-create params file
        error("Can't read op_params.json file, backing up to /data/op_params_corrupt.json and re-creating file!")
        to_write = True
        if os.path.isfile(self._backup_file):
          os.remove(self._backup_file)
        os.rename(self._params_file, self._backup_file)
    else:
      to_write = True  # user's first time running a fork with op_params, write default params

    if to_write:
      if self._write():
        os.chmod(self._params_file, 0o764)

  def get(self, key=None, force_live=False):  # any params you try to get MUST be in fork_params
    param_info = self.param_info(key)
    self._update_params(param_info, force_live)

    if key is None:
      return self._get_all_params()

    self._check_key_exists(key, 'get')
    value = self.params[key]
    if param_info.is_valid(value):  # always valid if no allowed types, otherwise checks to make sure
      return value  # all good, returning user's value

    warning('User\'s value type is not valid! Returning default')  # somehow... it should always be valid
    return param_info.default  # return default value because user's value of key is not in allowed_types to avoid crashing openpilot

  def put(self, key, value):
    self._check_key_exists(key, 'put')
    if not self.param_info(key).is_valid(value):
      raise Exception('opParams: Tried to put a value of invalid type!')
    self.params.update({key: value})
    self._write()

  def delete(self, key):  # todo: might be obsolete. remove?
    if key in self.params:
      del self.params[key]
      self._write()

  def param_info(self, key):
    if key in self.fork_params:
      return self.fork_params[key]
    return Param(None, type(None))

  def _check_key_exists(self, key, met):
    if key not in self.fork_params or key not in self.params:
      raise Exception('opParams: Tried to {} an unknown parameter! Key not in fork_params: {}'.format(met, key))

  def _add_default_params(self):
    added = False
    for key, param in self.fork_params.items():
      if key not in self.params:
        self.params[key] = param.default
        added = True
      elif not param.is_valid(self.params[key]):
        warning('Value type of user\'s {} param not in allowed types, replacing with default!'.format(key))
        self.params[key] = param.default
        added = True
    return added

  def _delete_old(self):
    deleted = False
    for param in self._to_delete:
      if param in self.params:
        del self.params[param]
        deleted = True
    return deleted

  def _get_all_params(self, default=False, return_hidden=False):
    if default:
      return {k: p.default for k, p in self.fork_params.items()}
    return {k: self.params[k] for k, p in self.fork_params.items() if k in self.params and (not p.hidden or return_hidden)}

  def _update_params(self, param_info, force_live):
    if force_live or param_info.live:  # if is a live param, we want to get updates while openpilot is running
      if not travis and sec_since_boot() - self._last_read_time >= self.read_frequency:  # make sure we aren't reading file too often
        if self._read():
          self._last_read_time = sec_since_boot()

  def _read(self):
    if os.path.isfile(self._params_file):
      try:
        mod_time = os.path.getmtime(self._params_file)
        if mod_time > self._last_mod_time:
          with open(self._params_file, "r") as f:
            self.params = json.loads(f.read())
          self._last_mod_time = mod_time
          return True
        else:
          return False
      except Exception as e:
        print("Unable to read file: " + str(e))
        return False
    else:
      return False

  def _write(self):
    if not travis or os.path.isdir("/data/"):
      try:
        with open(self._params_file, "w") as f:
          f.write(json.dumps(self.params, indent=2))  # can further speed it up by remove indentation but makes file hard to read
        return True
      except Exception as e:
        print("Unable to write file: " + str(e))
        return False

CAM_OFFSET = 'camera_offset'

ENABLE_UNSAFE_STEERING_RATE = "enable_unsafe_steering_rate"

ENABLE_COASTING = "enable_coasting"
COAST_SPEED = "coast_speed"
SETPOINT_OFFSET = "setpoint_offset"
DOWNHILL_INCLINE = "downhill_incline"
ALWAYS_EVAL_COAST = "always_eval_coast_plan"
EVAL_COAST_LONG = "eval_coast_long_controller"

SHOW_INDI_PARAMS = 'show_indi_params'
INDI_SHOW_BREAKPOINTS = 'indi_show_breakpoint_opts'

SHOW_A_CRUISE = 'a_cruise_show_opts'

ENABLE_LONG_PARAMS = 'enable_long_params'
ENABLE_GAS_PARAMS = 'enable_gas_params'
GAS_MAX_BP = 'gas_max_bp'
GAS_MAX_V = 'gas_max_v'
ENABLE_BRAKE_PARAMS = 'enable_brake_params'
BRAKE_MAX_BP = 'brake_max_bp'
BRAKE_MAX_V = 'brake_max_v'
ENABLE_LONG_PID_PARAMS = 'enable_long_pid_params'
LONG_PID_KP_BP = 'long_pid_kp_bp'
LONG_PID_KP_V = 'long_pid_kp_v'
LONG_PID_KI_BP = 'long_pid_ki_bp'
LONG_PID_KI_V = 'long_pid_ki_v'
LONG_PID_KF = 'long_pid_kf'
LONG_PID_SAT_LIMIT = 'long_pid_sat_limit'
ENABLE_LONG_DEADZONE_PARAMS = 'enable_long_deadzone_params'
LONG_DEADZONE_BP = 'long_deadzone_bp'
LONG_DEADZONE_V = 'long_deadzone_v'

ENABLE_LAT_PARAMS = 'enable_lat_params'
WHICH_LAT_CTRL = 'which_lat_controller'

SHOW_LQR_PARAMS = 'show_lqr_params'
LQR_SCALE = 'lqr_scale'
LQR_KI = 'lqr_ki'
LQR_A = 'lqr_a'
LQR_B = 'lqr_b'
LQR_C = 'lqr_c'
LQR_K = 'lqr_k'
LQR_L = 'lqr_l'
LQR_DC_GAIN = 'lqr_dc_gain'
STEER_LIMIT_TIMER = 'steer_limit_timer'

SHOW_ACTUATOR_DELAY_PARAMS = "show_actuator_delay_params"
STEER_ACTUATOR_DELAY = 'steer_actuator_delay'
ENABLE_ACTUATOR_DELAY_BPS = 'enable_actuator_delay_breakpoints'
STEER_ACTUATOR_DELAY_BP = 'steer_actuator_delay_bp'
STEER_ACTUATOR_DELAY_V = 'steer_actuator_delay_v'

SHOW_LAT_PID_PARAMS = 'show_lat_pid_params'
LAT_PID_KP_BP = 'lat_pid_kp_bp'
LAT_PID_KP_V = 'lat_pid_kp_v'
LAT_PID_KI_BP = 'lat_pid_ki_bp'
LAT_PID_KI_V = 'lat_pid_ki_v'
LAT_PID_KF = 'lat_pid_kf'

SHOW_UNSAFE_OPTS = 'show_unsafe_options'
SHOW_EXPERIMENTAL_OPTS = 'show_experimental_options'