import time
import csv
import socket

try:
    import ConfigParser
except ImportError:
    import configparser as ConfigParser

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusIOException


"""
dps_modbus_tcp.py

DROP-IN replacement for:
  dps_modbus.py (minimalmodbus / serial)

but using:
  pymodbus Modbus TCP

Compatibility goals:
- Keep same class names: Import_limits, Serial_modbus, Dps5005
- Keep same public method signatures
- Allow host string "ip" or "ip:port"
- Auto reconnect if connection breaks
"""


# ------------------------------------------------------------
# Import system limit thresholds
# ------------------------------------------------------------
class Import_limits:
    def __init__(self, filename):
        Config = ConfigParser.ConfigParser()
        Config.read(filename)

        b = Config.options("SectionOne")  # safety limits
        for x in range(len(b)):
            c = b[x]
            exec("self.%s = %s" % (c, Config.get("SectionOne", c)))

        b = Config.options("SectionTwo")  # decimal places
        for x in range(len(b)):
            c = b[x]
            exec("self.%s = %s" % (c, Config.get("SectionTwo", c)))

        b = Config.options("SectionThree")  # plot colours
        for x in range(len(b)):
            c = b[x]
            exec("self.%s = %s" % (c, Config.get("SectionThree", c)))


# ------------------------------------------------------------
# TCP Modbus wrapper (keeps original class name Serial_modbus!)
# ------------------------------------------------------------
class Serial_modbus:
    """
    Drop-in replacement for original Serial_modbus, but TCP.

    Old signature:
        Serial_modbus(port1, addr, baud_rate, byte_size)

    We keep that signature but reinterpret parameters:

    - port1: "IP" or "IP:PORT"
    - addr: unit_id (slave id)
    - baud_rate: ignored if port1 contains port, otherwise = TCP port (default 502)
    - byte_size: timeout seconds (default 3.0)

    Examples:
        Serial_modbus("192.168.11.53", 1, 502, 3)
        Serial_modbus("192.168.11.53:502", 1, 0, 3)
        Serial_modbus("192.168.11.53:1502", 1, 0, 1.5)
    """

    def __init__(self, port1, addr, baud_rate, byte_size):
        self.host, self.port = self._parse_host_port(port1, baud_rate)
        self.unit_id = int(addr)
        self.timeout = float(byte_size) if byte_size else 3.0

        # reconnect management
        self._client = None
        self._last_connect_try = 0.0
        self._reconnect_min_interval = 1.0  # seconds

        # Match the attribute name used elsewhere in GUI
        self.instrument = self

        # initial connect
        self._connect()

    def _parse_host_port(self, port1, baud_rate):
        """
        Accept:
          "ip"
          "ip:port"
        """
        s = str(port1).strip()

        if ":" in s:
            host, port_str = s.rsplit(":", 1)
            host = host.strip()
            port = int(port_str.strip())
            return host, port

        # fallback: baud_rate used as port
        port = int(baud_rate) if baud_rate else 502
        return s, port

    def _connect(self):
        try:
            if self._client:
                try:
                    self._client.close()
                except Exception:
                    pass
            self._client = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout)
            return bool(self._client.connect())
        except Exception:
            self._client = None
            return False

    def _ensure(self):
        """
        Ensure client is connected, do reconnect with rate limit.
        Raise IOError if not possible.
        """
        now = time.time()

        if self._client is None:
            if now - self._last_connect_try < self._reconnect_min_interval:
                raise IOError("Not connected")
            self._last_connect_try = now
            if not self._connect():
                raise IOError("Cannot connect to Modbus TCP server")
            return

        # if client exists but socket dead => reconnect
        try:
            # pymodbus: "connected" isn't always reliable, do a lightweight connect()
            if not self._client.connect():
                if now - self._last_connect_try < self._reconnect_min_interval:
                    raise IOError("Reconnect throttled")
                self._last_connect_try = now
                if not self._connect():
                    raise IOError("Reconnect failed")
        except Exception:
            if now - self._last_connect_try < self._reconnect_min_interval:
                raise IOError("Reconnect throttled")
            self._last_connect_try = now
            if not self._connect():
                raise IOError("Reconnect failed")

    # --------------------------------
    # minimalmodbus compatible methods
    # --------------------------------
    def read(self, reg_addr, decimal_places):
        """
        minimalmodbus.read_register(reg, decimals) returns scaled float.
        emulate that:
        raw / (10**decimal_places)
        """
        self._ensure()
        rr = self._client.read_holding_registers(int(reg_addr), count=1, device_id=self.unit_id)
        if rr is None or isinstance(rr, ModbusIOException) or getattr(rr, "registers", None) is None:
            raise IOError("Failed to read from instrument")

        raw = rr.registers[0]
        return raw / float(10 ** int(decimal_places))

    def read_block(self, reg_addr, size_of_block):
        self._ensure()
        rr = self._client.read_holding_registers(int(reg_addr), count=int(size_of_block), device_id=self.unit_id)
        if rr is None or isinstance(rr, ModbusIOException) or getattr(rr, "registers", None) is None:
            raise IOError("Failed to read block from instrument")
        return rr.registers

    def write(self, reg_addr, value, decimal_places):
        """
        minimalmodbus.write_register(reg, value, decimals)
        -> writes scaled integer
        """
        self._ensure()
        raw = int(round(float(value) * float(10 ** int(decimal_places))))
        wr = self._client.write_register(int(reg_addr), raw, device_id=self.unit_id)
        if wr is None:
            raise IOError("Failed to write to instrument")

    def write_block(self, reg_addr, value):
        self._ensure()
        wr = self._client.write_registers(int(reg_addr), list(map(int, value)), device_id=self.unit_id)
        if wr is None:
            raise IOError("Failed to write block to instrument")


# ------------------------------------------------------------
# DPS logic (kept identical)
# ------------------------------------------------------------
class Dps5005:
    def __init__(self, ser, limits):
        self.serial_data = ser
        self.limits = limits

    def voltage_set(self, RWaction="r", value=0.0):  # R/W
        return self.function(
            0x00,
            self.limits.decimals_vset,
            RWaction,
            value,
            self.limits.voltage_set_max,
            self.limits.voltage_set_min,
        )

    def current_set(self, RWaction="r", value=0.0):  # R/W
        return self.function(
            0x01,
            self.limits.decimals_iset,
            RWaction,
            value,
            self.limits.current_set_max,
            self.limits.current_set_min,
        )

    def voltage(self):  # R
        return self.function(0x02, self.limits.decimals_v)

    def current(self):  # R
        return self.function(0x03, self.limits.decimals_i)

    def power(self):  # R
        return self.function(0x04, self.limits.decimals_power)

    def voltage_in(self):  # R
        return self.function(0x05, self.limits.decimals_vin)

    def lock(self, RWaction="r", value=0):  # R/W
        return self.function(15, 0, RWaction, value, self.limits.lock_set_max, self.limits.lock_set_min)

    def protect(self):  # R
        return self.function(0x16, 0)

    def cv_cc(self):  # R
        return self.function(17, 0)

    def onoff(self, RWaction="r", value=0):  # R/W
        return self.function(18, 0, RWaction, value, self.limits.onoff_set_max, self.limits.onoff_set_min)

    def b_led(self, RWaction="r", value=0):  # R/W
        return self.function(0x0A, 0, RWaction, value, self.limits.b_led_set_max, self.limits.b_led_set_min)

    def model(self):  # R
        return self.function(22, 0)

    def version(self):  # R
        return self.function(23, self.limits.decimals_version)

    def read_all(self, RWaction="r", value=0.0):
        data = self.functions(0x00, 30, RWaction, value)

        # adjust to float
        data[0] = data[0] / float(10 ** self.limits.decimals_vset)
        data[1] = data[1] / float(10 ** self.limits.decimals_iset)
        data[2] = data[2] / float(10 ** self.limits.decimals_v)
        data[3] = data[3] / float(10 ** self.limits.decimals_i)
        data[4] = data[4] / float(10 ** self.limits.decimals_power)
        data[5] = data[5] / float(10 ** self.limits.decimals_vin)
        data[23] = data[23] / float(10 ** self.limits.decimals_version)
        data[13] = data[13] / float(10)
        data[8] = data[8] / float(1000)

        return data

    def write_voltage_current(self, RWaction="r", value=0):
        reg_addr = 0x00

        if value[0] > self.limits.voltage_set_max or value[0] < self.limits.voltage_set_min:
            value[0] = 0
        value[0] = int(value[0] * float(10 ** self.limits.decimals_v))

        if value[1] > self.limits.current_set_max or value[1] < self.limits.current_set_min:
            value[1] = 0
        value[1] = int(value[1] * float(10 ** self.limits.decimals_i))

        self.functions(reg_addr, 0, "w", value)

    def write_all(self, reg_addr=0, value=0):
        self.functions(reg_addr, 0, "w", value)

    def function(self, reg_addr=0, decimal_places=0, RWaction="r", value=0.0, max_value=0, min_value=0):
        a = False
        if value > max_value or value < min_value:
            value = 0.0
        if RWaction != "w":
            try:
                a = self.serial_data.read(reg_addr, decimal_places)
            except IOError:
                print("Failed to read from instrument")
        else:
            try:
                self.serial_data.write(reg_addr, value, decimal_places)
            except IOError:
                print("Failed to write to instrument")
        return a

    def functions(self, reg_addr=0, num_of_addr=0, RWaction="r", value=0):
        a = False
        if RWaction != "w":
            try:
                a = self.serial_data.read_block(reg_addr, num_of_addr)
            except IOError:
                print("Failed to read block from instrument")
        else:
            try:
                self.serial_data.write_block(reg_addr, value)
            except IOError:
                print("Failed to write block to instrument")
        return a

    def delay(self, value):
        global time_old
        value = float(value)
        if value == 0.0:
            time_old = time.time()

        while True:
            time_interval = time.time() - time_old
            if time_interval >= value:
                time_old = time.time()
                break
            time.sleep(0.01)

    def action_csv_file(self, filename="sample.csv", value=0):
        try:
            with open(filename, "r") as f:
                csvReader = csv.reader(f)
                next(csvReader, None)
                data_list = list(csvReader)

            # initialise dps state
            self.voltage_set("w", float(0.0))
            self.current_set("w", float(0.0))
            self.onoff("w", 1)
            total_time = 0

            for row in data_list:
                total_time += float(row[0])
            print("Test Time: %5.1fseconds" % (total_time))

            for row in data_list:
                value0 = float(row[0])
                value1 = float(row[1])
                value2 = float(row[2])
                print(" Step_time: %5.1fs, Voltage: %5.2fV, Current: %5.3fA" % (value0, value1, value2))
                self.voltage_set("w", value1)
                self.current_set("w", value2)
                self.delay(value0)

            self.onoff("w", 0)
            print("Complete!")
            return

        except Exception:
            print("Failed to load file.")


# ------------------------------------------------------------
# Command line test
# ------------------------------------------------------------
if __name__ == "__main__":
    # Drop-in instantiation:
    # Serial_modbus(host_or_hostport, unit_id, port_if_needed, timeout_s)
    ser = Serial_modbus("192.168.11.53:502", 1, 502, 3)

    limits = Import_limits("dps5005_limits.ini")
    dps = Dps5005(ser, limits)

    try:
        while True:
            route = input("Enter command: ")
            if route == "q":
                quit()
            elif route == "read":
                start = time.time()
                print(dps.read_all())
                print(time.time() - start)
            elif route == "write":
                value = [23.47, 1.23]
                dps.write_voltage_current("w", value)
            elif route == "on":
                dps.onoff("w", 1)
            elif route == "off":
                dps.onoff("w", 0)
            else:
                pass

    except KeyboardInterrupt:
        print("close")
    finally:
        try:
            dps.onoff("w", 0)
        except Exception as e:
            print("Failed to write to instrument:", repr(e))
            pass
        quit()
