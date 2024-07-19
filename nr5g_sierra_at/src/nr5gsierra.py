import rospy
import serial
import time
from datetime import datetime
import re
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from nr5g_sierra_at.msg import at_sierra_nr5g

# Read interval
refresh = 0.2

gNodeB_ID_Length = 28

# Configuration of the serial port comm
ser = serial.Serial(timeout=1)
ser.baudrate = 115200
ser.port = '/dev/ttyUSB0'
ser.open()

def parse_nr5g():
    # Initialize variables
    nr5g_rssi = [None] * 4
    nr5g_rsrp, nr5g_rsrq, nr5g_sinr, nr5g_band, nr5g_bw_dl, nr5g_bw_ul, nr5g_cell_id = [None] * 7
    nr5g_mimo_dl, nr5g_mimo_ul, nr5g_tx_power = [None] * 3

    ser.write(b'AT!GSTATUS?\r\n')

    while True:
        line = ser.readline().replace(b'\r\n', b'').decode('utf-8')
        # print(f"DEBUG: {line}")  # Debug print to trace each line

        if line:
            if 'NR5G Cell ID' in line:
                nr5g_cell_id = line.split(':')[2].strip().split()[1].replace(')','').replace('(','')
                gNodeB_ID = int(nr5g_cell_id)//(2 ** (36 - gNodeB_ID_Length))
                sector_ID = int(nr5g_cell_id) - (gNodeB_ID * (2 ** (36 - gNodeB_ID_Length)))
            elif 'NR5G band' in line:
                nr5g_band = line.split(':')[1].strip().split()[0]
            elif 'NR5G dl bw' in line:
                parts = line.split()
                nr5g_bw_dl = parts[3]
                nr5g_bw_ul = parts[8]
            elif 'NR5G(sub6) Rx' in line:
                # Use regex to find all RSSI values in the line
                rssi_matches = re.findall(r'NR5G\(sub6\) Rx(\d) RSSI \(dBm\):\s*(-?\d+\.\d+)', line)
                for match in rssi_matches:
                    index = int(match[0])
                    nr5g_rssi[index] = float(match[1])
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
    # return {       
    #     "timestamp": str(datetime.now()),
    #     "net_param": {
    #         "net_type": "5G",
    #         "rsrp": 1,
    #         "rsrq": 2,
    #         "sinr": 3,
    #         "band": 'N78',
    #         "bandwidth_dl": 100,
    #         "bandwidth_ul": 100,
    #         "rssi_0": -1,
    #         "rssi_1": -2,
    #         "rssi_2": -3,
    #         "rssi_3": -4,
    #         "cell_id": 12,
    #         "gNB": 15,
    #         "sector_ID": 16,
    #         "mimo_dl": 4,
    #         "mimo_ul": 1,
    #         "tx_power": 20
    #     }
    #}

def scan():
    nr5g_data = parse_nr5g()
    return {"nr5g": nr5g_data}

def publish_5g_status():
    rospy.init_node('nr5g_sierra_at', anonymous=True)
    pub = rospy.Publisher('sierra_nr5g', at_sierra_nr5g, queue_size=10)
    rate = rospy.Rate(1/refresh)  # Frequency based on refresh rate

    while not rospy.is_shutdown():
        measurements = scan()
        nr5g_params = measurements["nr5g"]["net_param"]
        msg = at_sierra_nr5g()
        msg.header.stamp = rospy.Time.now()
        msg.net_type = nr5g_params["net_type"]
        msg.rsrp = float(nr5g_params["rsrp"]) if nr5g_params["rsrp"] is not None else float('nan')
        msg.rsrq = float(nr5g_params["rsrq"]) if nr5g_params["rsrq"] is not None else float('nan')
        msg.sinr = float(nr5g_params["sinr"]) if nr5g_params["sinr"] is not None else float('nan')
        msg.band = nr5g_params["band"]
        msg.bw_dl = nr5g_params["bandwidth_dl"]
        msg.bw_ul = nr5g_params["bandwidth_ul"]
        msg.cell_id = nr5g_params["cell_id"]
        msg.gNB = nr5g_params["gNB"]
        msg.sector_ID = nr5g_params["sector_ID"]
        msg.mimo_dl = int(nr5g_params["mimo_dl"]) if nr5g_params["mimo_dl"] is not None else 0
        msg.mimo_ul = int(nr5g_params["mimo_ul"]) if nr5g_params["mimo_ul"] is not None else 0
        msg.tx_power = float(nr5g_params["tx_power"]) if nr5g_params["tx_power"] is not None else float('nan')
        msg.rssi_0 = float(nr5g_params["rssi_0"]) if nr5g_params["rssi_0"] is not None else float('nan')
        msg.rssi_1 = float(nr5g_params["rssi_1"]) if nr5g_params["rssi_1"] is not None else float('nan')
        msg.rssi_2 = float(nr5g_params["rssi_2"]) if nr5g_params["rssi_2"] is not None else float('nan')
        msg.rssi_3 = float(nr5g_params["rssi_3"]) if nr5g_params["rssi_3"] is not None else float('nan')

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_5g_status()
    except rospy.ROSInterruptException:
        pass
