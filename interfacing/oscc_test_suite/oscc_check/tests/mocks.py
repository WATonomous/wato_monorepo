from oscccan.canbus import Report


class CanBus(object):
    """
    Mocked CanBus class for tests that don't require harware in loop.
    """

    def report_success(self, increase_from=None, decrease_from=None):
        value = 0x01

        if increase_from is not None:
            value += increase_from
        elif decrease_from is not None:
            value -= decrease_from

        return Report(success=True, value=value)

    def __init__(
            self,
            vehicle,
            bustype='socketcan_native',
            channel='can0',
            bitrate=500000):
        pass

    def reading_sleep(self, duration=1.0):
        pass

    def bus_send_msg(self, arbitration_id, data=None, timeout=1.0):
        pass

    def enable_module(self, module, timeout=None):
        pass

    def disable_module(self, module, timeout=None):
        pass

    def check_module_enabled_status(
            self,
            module,
            timeout=1.0,
            expect=False):
        return True

    def send_command(
            self,
            module,
            value,
            timeout=None):
        pass

    def recv_report(
            self,
            module=None,
            can_ids=None,
            timeout=1.0):
        pass

    def check_brake_pressure(
            self,
            increase_from=None,
            decrease_from=None,
            timeout=2.0,):
        return self.report_success(increase_from=increase_from,
                                   decrease_from=decrease_from)

    def check_steering_wheel_angle(
            self,
            increase_from=None,
            decrease_from=None,
            timeout=2.0):
        return self.report_success(increase_from=increase_from,
                                   decrease_from=decrease_from)

    def get_wheel_speed(self, data, offset):
        pass

    def check_wheel_speed(
            self,
            timeout=2.0):
        return self.report_success()


class CallableNoOp:
    def __call__(self, *args):
        pass
