import rclpy
from rclpy.node import Node
from datetime import datetime
import serial
import re
from nr5g_sierra_at.msg import AtSierraNr5g  # Custom message
from std_msgs.msg import String

gNodeB_ID_Length = 28

class NR5GSierraPublisher(Node):
    def __init__(self):
        super().__init__('nr5g_sierra_publisher')

        # Serial port configuration
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=1
        )

        # Publisher setup
        self.publisher_ = self.create_publisher(AtSierraNr5g, 'sierra_nr5g', 10)

        # Timer to call the publish method at the desired interval
        self.refresh = 0.25
        self.timer = self.create_timer(self.refresh, self.publish_status)

        self.get_logger().info('Node has been initialized')

    def publish_status(self):
        measurements = self.parse_nr5g()

        if not measurements:
            self.get_logger().warning('Failed to parse measurements.')
            return

        nr5g_params = measurements["net_param"]
        msg = AtSierraNr5g()

        # Assign values to the custom message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.net_type = nr5g_params.get("net_type", "N/A")
        msg.rsrp = self.safe_float_conversion(nr5g_params.get("rsrp"))
        msg.rsrq = self.safe_float_conversion(nr5g_params.get("rsrq"))
        msg.sinr = self.safe_float_conversion(nr5g_params.get("sinr"))
        msg.band = nr5g_params.get("band", "N/A")
        msg.bw_dl = self.safe_int_conversion(nr5g_params.get("bandwidth_dl"))
        msg.bw_ul = self.safe_int_conversion(nr5g_params.get("bandwidth_ul"))
        msg.cell_id = self.safe_int_conversion(nr5g_params.get("cell_id"))
        msg.gNB = self.safe_int_conversion(nr5g_params.get("gNB"))
        msg.sector_ID = self.safe_int_conversion(nr5g_params.get("sector_ID"))
        msg.mimo_dl = self.safe_int_conversion(nr5g_params.get("mimo_dl"))
        msg.mimo_ul = self.safe_int_conversion(nr5g_params.get("mimo_ul"))
        msg.tx_power = self.safe_float_conversion(nr5g_params.get("tx_power"))
        msg.rssi_0 = self.safe_float_conversion(nr5g_params.get("rssi_0"))
        msg.rssi_1 = self.safe_float_conversion(nr5g_params.get("rssi_1"))
        msg.rssi_2 = self.safe_float_conversion(nr5g_params.get("rssi_2"))
        msg.rssi_3 = self.safe_float_conversion(nr5g_params.get("rssi_3"))

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info('Published 5G network status')

    def parse_nr5g(self):
        # Initialize variables with default values
        nr5g_rssi = [None] * 4
        nr5g_rsrp, nr5g_rsrq, nr5g_sinr, nr5g_band, nr5g_bw_dl, nr5g_bw_ul, nr5g_cell_id, gNodeB_ID, sector_ID = [None] * 9
        nr5g_mimo_dl, nr5g_mimo_ul, nr5g_tx_power = [None] * 3

        # Send command to the device
        self.ser.write(b'AT!GSTATUS?\r\n')

        while True: 
            line = self.ser.readline().replace(b'\r\n', b'').decode('utf-8')

            # Only process non-empty lines
            if line:
                try:
                    # Parse NR5G Cell ID
                    if 'NR5G Cell ID' in line:
                        try:
                            nr5g_cell_id = line.split(':')[2].strip().split()[1].replace(')','').replace('(','')
                            gNodeB_ID = int(nr5g_cell_id)//(2 ** (36 - gNodeB_ID_Length))
                            sector_ID = int(nr5g_cell_id) - (gNodeB_ID * (2 ** (36 - gNodeB_ID_Length)))
                        except (IndexError, ValueError) as e:
                            print(f"Parsing Error in 'NR5G Cell ID': {e}")

                    # Parse NR5G Band
                    elif 'NR5G band' in line:
                        try:
                            nr5g_band = line.split(':')[1].strip().split()[0]
                        except IndexError as e:
                            print(f"Parsing Error in 'NR5G band': {e}")

                    # Parse NR5G Download and Upload Bandwidth
                    elif 'NR5G dl bw' in line:
                        try:
                            parts = line.split()
                            nr5g_bw_dl = parts[3]
                            nr5g_bw_ul = parts[8]
                        except IndexError as e:
                            print(f"Parsing Error in 'NR5G dl bw': {e}")

                    # Parse RSSI values
                    elif 'NR5G(sub6) Rx' in line:
                        try:
                            # Use regex to find all RSSI values in the line
                            rssi_matches = re.findall(r'NR5G\(sub6\) Rx(\d) RSSI \(dBm\):\s*(-?\d+\.\d+)', line)
                            for match in rssi_matches:
                                index = int(match[0])
                                nr5g_rssi[index] = float(match[1])
                        except ValueError as e:
                            print(f"Parsing Error in 'NR5G(sub6) Rx': {e}")

                    elif 'NR5G RSRP' in line:
                        nr5g_rsrp = line.split(':')[1].strip().split()[0]
                        nr5g_rsrq = line.split(':')[2].strip().split()[0]
                    elif 'NR5G SINR' in line:
                        nr5g_sinr = line.split(':')[1].strip().split()[0]
                    elif 'NR5G dl MIMO' in line:
                        nr5g_mimo_dl = line.split(':')[1].strip().split()[0]
                        nr5g_mimo_ul = line.split(':')[2].strip().split()[0]
                    elif 'NR5G Tx Power' in line:
                        nr5g_tx_power = line.split(':')[1].strip().split()[0]
                    elif line.startswith('OK'):
                        break
                
                except Exception as e:
                    # General exception catch for any unexpected error in parsing
                    print(f"Unexpected error: {e}")

            # Return the parsed dictionary
            return {
                "timestamp": str(datetime.now()),
                "net_param": {
                    "net_type": "5G",
                    "rsrp": nr5g_rsrp,
                    "rsrq": nr5g_rsrq,
                    "sinr": nr5g_sinr,
                    "band": nr5g_band,
                    "bandwidth_dl": nr5g_bw_dl,
                    "bandwidth_ul": nr5g_bw_ul,
                    "rssi_0": nr5g_rssi[0],
                    "rssi_1": nr5g_rssi[1],
                    "rssi_2": nr5g_rssi[2],
                    "rssi_3": nr5g_rssi[3],
                    "cell_id": nr5g_cell_id,
                    "gNB": gNodeB_ID,
                    "sector_ID": sector_ID,
                    "mimo_dl": nr5g_mimo_dl,
                    "mimo_ul": nr5g_mimo_ul,
                    "tx_power": nr5g_tx_power
                }
            }

    @staticmethod
    def safe_float_conversion(value):
        return float(value) if value not in [None, '---'] else float('nan')

    @staticmethod
    def safe_int_conversion(value):
        return int(value) if value not in [None, '---'] else 0

def main(args=None):
    rclpy.init(args=args)
    node = NR5GSierraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
