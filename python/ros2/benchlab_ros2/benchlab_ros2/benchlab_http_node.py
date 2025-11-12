#!/usr/bin/env python3
"""BenchLab HTTP Bridge ROS2 Node

Production-grade ROS2 node that bridges HTTP service to ROS2 topics/services.
Uses existing benchlabd service for device access.

Features:
- Structured ROS2 messages (type-safe, introspectable)
- Service servers for all protocol commands via HTTP API
- Diagnostics publishing
- Lifecycle node management
- Configurable QoS policies
- Multi-device support via namespaces
- API key authentication support
"""

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from benchlab_msgs.msg import (
    BenchLabTelemetry, PowerSensor, FanSensor, DeviceInfo,
    FanProfile, RGBConfig, CalibrationData
)
from benchlab_msgs.srv import (
    SetDeviceName, SetFanProfile, SetRGB, SetCalibration,
    LoadCalibration, StoreCalibration, ExecuteAction
)

import requests
import json
import traceback


class BenchLabHttpNode(LifecycleNode):
    """HTTP bridge ROS2 node for BenchLab devices.

    Connects to benchlabd HTTP service and publishes to ROS2.
    """

    def __init__(self):
        super().__init__('benchlab_http_node')

        # Declare parameters
        self.declare_parameter('base_url', 'http://127.0.0.1:8080')
        self.declare_parameter('api_key', '')
        self.declare_parameter('device_id', 'benchlab0')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('request_timeout', 2.0)  # seconds
        self.declare_parameter('device_namespace', '')

        # HTTP session
        self.session: requests.Session = None

        # Publishers
        self.telemetry_pub = None
        self.device_info_pub = None
        self.diagnostics_pub = None

        # Timer
        self.timer = None

        # Device info
        self.device_name = ""
        self.device_uid = ""
        self.fw_version = 0

        # Diagnostics state
        self.read_count = 0
        self.error_count = 0
        self.last_error = ""

        self.get_logger().info('BenchLab HTTP Node created')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure node - create HTTP session and publishers."""
        self.get_logger().info('Configuring...')

        try:
            # Get parameters
            base_url = self.get_parameter('base_url').value
            api_key = self.get_parameter('api_key').value
            device_id = self.get_parameter('device_id').value
            timeout = self.get_parameter('request_timeout').value
            namespace = self.get_parameter('device_namespace').value

            # Create HTTP session
            self.session = requests.Session()
            if api_key:
                self.session.headers.update({'Authorization': f'Bearer {api_key}'})
            self.session.headers.update({'Content-Type': 'application/json'})

            self.base_url = base_url
            self.device_id = device_id
            self.timeout = timeout

            # Test connection and get device info
            response = self.session.get(f'{base_url}/devices/{device_id}/info', timeout=timeout)
            response.raise_for_status()
            info = response.json()

            self.device_name = info.get('name', '')
            self.fw_version = info.get('firmwareVersion', 0)

            # Get UID
            response = self.session.get(f'{base_url}/devices/{device_id}/uid', timeout=timeout)
            response.raise_for_status()
            uid_data = response.json()
            self.device_uid = uid_data.get('uid', '')

            self.get_logger().info(f'Connected to: {self.device_name} via {base_url}')

            # Create QoS profiles
            telemetry_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            status_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )

            # Create publishers
            topic_prefix = f'{namespace}/' if namespace else ''

            self.telemetry_pub = self.create_publisher(
                BenchLabTelemetry,
                f'{topic_prefix}benchlab/telemetry',
                telemetry_qos
            )

            self.device_info_pub = self.create_publisher(
                DeviceInfo,
                f'{topic_prefix}benchlab/device_info',
                status_qos
            )

            self.diagnostics_pub = self.create_publisher(
                DiagnosticArray,
                '/diagnostics',
                10
            )

            # Create service servers
            self.create_service(
                SetDeviceName,
                f'{topic_prefix}benchlab/set_name',
                self._handle_set_name
            )

            self.create_service(
                SetFanProfile,
                f'{topic_prefix}benchlab/set_fan_profile',
                self._handle_set_fan_profile
            )

            self.create_service(
                SetRGB,
                f'{topic_prefix}benchlab/set_rgb',
                self._handle_set_rgb
            )

            self.create_service(
                SetCalibration,
                f'{topic_prefix}benchlab/set_calibration',
                self._handle_set_calibration
            )

            self.create_service(
                LoadCalibration,
                f'{topic_prefix}benchlab/load_calibration',
                self._handle_load_calibration
            )

            self.create_service(
                StoreCalibration,
                f'{topic_prefix}benchlab/store_calibration',
                self._handle_store_calibration
            )

            self.create_service(
                ExecuteAction,
                f'{topic_prefix}benchlab/execute_action',
                self._handle_execute_action
            )

            # Publish device info
            self._publish_device_info()

            self.get_logger().info('Configuration complete')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Configuration failed: {e}')
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate node - start streaming."""
        self.get_logger().info('Activating...')

        try:
            rate = self.get_parameter('publish_rate').value
            period = 1.0 / rate
            self.timer = self.create_timer(period, self._telemetry_callback)

            self.get_logger().info(f'Streaming telemetry at {rate} Hz')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Activation failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate node - stop streaming."""
        self.get_logger().info('Deactivating...')

        if self.timer:
            self.timer.cancel()
            self.timer = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup node - close HTTP session."""
        self.get_logger().info('Cleaning up...')

        if self.session:
            self.session.close()
            self.session = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown node."""
        self.get_logger().info('Shutting down...')

        if self.session:
            self.session.close()

        return TransitionCallbackReturn.SUCCESS

    def _telemetry_callback(self):
        """Fetch and publish telemetry via HTTP."""
        try:
            # Fetch sensors from HTTP API
            response = self.session.get(
                f'{self.base_url}/devices/{self.device_id}/sensors',
                timeout=self.timeout
            )
            response.raise_for_status()
            data = response.json()

            # Create ROS2 message
            msg = BenchLabTelemetry()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.device_name

            # Populate from JSON
            msg.voltages_mv = data.get('voltages', [0] * 13)
            msg.vdd_mv = data.get('vdd', 0)
            msg.vref_mv = data.get('vref', 0)

            msg.chip_temp_c = data.get('chipTemp', 0.0)
            temps = data.get('temperatures', [0.0] * 6)
            msg.temp1_c = temps[1] if len(temps) > 1 else 0.0
            msg.temp2_c = temps[2] if len(temps) > 2 else 0.0
            msg.temp3_c = temps[3] if len(temps) > 3 else 0.0
            msg.temp4_c = temps[4] if len(temps) > 4 else 0.0
            msg.ambient_temp_c = data.get('ambientTemp', 0.0)

            msg.humidity_percent = data.get('humidity', 0.0)

            msg.fan_switch_status = data.get('fanSwitchStatus', 0)
            msg.rgb_switch_status = data.get('rgbSwitchStatus', 0)
            msg.rgb_ext_status = data.get('rgbExtStatus', 0)
            msg.fan_ext_duty = data.get('fanExtDuty', 0)

            # Power sensors
            msg.power = []
            for ps_data in data.get('power', []):
                ps = PowerSensor()
                ps.voltage_v = ps_data.get('v', 0.0)
                ps.current_a = ps_data.get('a', 0.0)
                ps.power_w = ps_data.get('w', 0.0)
                msg.power.append(ps)

            # Fan sensors
            msg.fans = []
            for fs_data in data.get('fans', []):
                fs = FanSensor()
                fs.enabled = fs_data.get('enabled', False)
                fs.duty_cycle = fs_data.get('duty', 0)
                fs.rpm = fs_data.get('rpm', 0)
                fs.duty_percent = fs_data.get('dutyPercent', 0.0)
                msg.fans.append(fs)

            # Publish
            self.telemetry_pub.publish(msg)
            self.read_count += 1

            if self.read_count % 10 == 0:
                self._publish_diagnostics(DiagnosticStatus.OK, 'Streaming telemetry')

        except Exception as e:
            self.error_count += 1
            self.last_error = str(e)
            self.get_logger().error(f'Telemetry fetch error: {e}')
            self._publish_diagnostics(DiagnosticStatus.ERROR, f'Fetch failed: {e}')

    def _publish_device_info(self):
        """Publish device information."""
        msg = DeviceInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.device_path = f'/dev/{self.device_id}'
        msg.device_name = self.device_name
        msg.uid = self.device_uid
        msg.vendor_id = 0xEE
        msg.product_id = 0x10
        msg.firmware_version = self.fw_version

        self.device_info_pub.publish(msg)

    def _publish_diagnostics(self, level: int, message: str):
        """Publish diagnostics."""
        diag_array = DiagnosticArray()
        diag_array.header = Header()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.level = level
        status.name = f'BenchLab HTTP: {self.device_name}'
        status.message = message
        status.hardware_id = self.device_uid

        status.values = [
            KeyValue(key='base_url', value=self.base_url),
            KeyValue(key='device_id', value=self.device_id),
            KeyValue(key='read_count', value=str(self.read_count)),
            KeyValue(key='error_count', value=str(self.error_count)),
            KeyValue(key='last_error', value=self.last_error)
        ]

        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)

    # Service Handlers (HTTP API calls)

    def _handle_set_name(self, request, response):
        """Handle set device name via HTTP API."""
        try:
            resp = self.session.put(
                f'{self.base_url}/devices/{self.device_id}/name',
                json={'name': request.name},
                timeout=self.timeout
            )
            resp.raise_for_status()
            response.success = True
            response.message = 'OK'
            self.device_name = request.name
            self._publish_device_info()
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_set_fan_profile(self, request, response):
        """Handle set fan profile via HTTP API."""
        try:
            mode_str = 'manual' if request.profile.mode == 0 else 'auto'
            payload = {
                'mode': mode_str,
                'manualDuty': request.profile.manual_duty,
                'tempThreshold': request.profile.temp_threshold_c,
                'minDuty': request.profile.min_duty,
                'maxDuty': request.profile.max_duty,
                'sensorIndex': request.profile.sensor_index
            }

            resp = self.session.put(
                f'{self.base_url}/devices/{self.device_id}/fans/{request.fan_index}',
                json=payload,
                timeout=self.timeout
            )
            resp.raise_for_status()
            data = resp.json()
            response.success = True
            response.message = data.get('status', 'OK')
            response.status_code = data.get('statusCode', 0)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.status_code = 255
        return response

    def _handle_set_rgb(self, request, response):
        """Handle set RGB via HTTP API."""
        try:
            mode_map = {0: 'off', 1: 'solid', 2: 'breathing', 3: 'cycle', 4: 'temperature'}
            payload = {
                'mode': mode_map.get(request.config.mode, 'off'),
                'red': request.config.red,
                'green': request.config.green,
                'blue': request.config.blue,
                'brightness': request.config.brightness,
                'speed': request.config.speed
            }

            resp = self.session.put(
                f'{self.base_url}/devices/{self.device_id}/rgb',
                json=payload,
                timeout=self.timeout
            )
            resp.raise_for_status()
            data = resp.json()
            response.success = True
            response.message = data.get('status', 'OK')
            response.status_code = data.get('statusCode', 0)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.status_code = 255
        return response

    def _handle_set_calibration(self, request, response):
        """Handle set calibration via HTTP API."""
        try:
            payload = {
                'voltageOffsets': list(request.calibration.voltage_offsets_mv),
                'voltageScales': list(request.calibration.voltage_scales),
                'tempOffset': request.calibration.temp_offset,
                'tempScale': request.calibration.temp_scale,
                'currentOffsets': list(request.calibration.current_offsets_ma),
                'currentScales': list(request.calibration.current_scales),
                'flags': request.calibration.flags
            }

            resp = self.session.put(
                f'{self.base_url}/devices/{self.device_id}/calibration',
                json=payload,
                timeout=self.timeout
            )
            resp.raise_for_status()
            data = resp.json()
            response.success = True
            response.message = data.get('status', 'OK')
            response.status_code = data.get('statusCode', 0)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.status_code = 255
        return response

    def _handle_load_calibration(self, request, response):
        """Handle load calibration via HTTP API."""
        try:
            resp = self.session.post(
                f'{self.base_url}/devices/{self.device_id}/calibration/load',
                timeout=self.timeout
            )
            resp.raise_for_status()
            data = resp.json()
            response.success = True
            response.message = data.get('status', 'OK')
            response.status_code = data.get('statusCode', 0)

            # Get calibration data
            if response.status_code == 0:
                cal_resp = self.session.get(
                    f'{self.base_url}/devices/{self.device_id}/calibration',
                    timeout=self.timeout
                )
                cal_resp.raise_for_status()
                cal_data = cal_resp.json()

                response.calibration = CalibrationData()
                response.calibration.voltage_offsets_mv = cal_data.get('voltageOffsets', [0] * 13)
                response.calibration.voltage_scales = cal_data.get('voltageScales', [1000] * 13)
                response.calibration.temp_offset = cal_data.get('tempOffset', 0)
                response.calibration.temp_scale = cal_data.get('tempScale', 1000)
                response.calibration.current_offsets_ma = cal_data.get('currentOffsets', [0] * 11)
                response.calibration.current_scales = cal_data.get('currentScales', [1000] * 11)
                response.calibration.flags = cal_data.get('flags', 0)

        except Exception as e:
            response.success = False
            response.message = str(e)
            response.status_code = 255
        return response

    def _handle_store_calibration(self, request, response):
        """Handle store calibration via HTTP API."""
        try:
            resp = self.session.post(
                f'{self.base_url}/devices/{self.device_id}/calibration/store',
                timeout=self.timeout
            )
            resp.raise_for_status()
            data = resp.json()
            response.success = True
            response.message = data.get('status', 'OK')
            response.status_code = data.get('statusCode', 0)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.status_code = 255
        return response

    def _handle_execute_action(self, request, response):
        """Handle execute action via HTTP API."""
        try:
            resp = self.session.post(
                f'{self.base_url}/devices/{self.device_id}/action',
                json={'actionId': request.action_id},
                timeout=self.timeout
            )
            resp.raise_for_status()
            data = resp.json()
            response.success = True
            response.message = data.get('status', 'OK')
            response.status_code = data.get('statusCode', 0)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.status_code = 255
        return response


def main(args=None):
    rclpy.init(args=args)

    node = BenchLabHttpNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
