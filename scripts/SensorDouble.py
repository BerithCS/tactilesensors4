#!/usr/bin/env python3
"""
SensorDouble.py — ROS 2 version
"""

import rclpy
from rclpy.node import Node
import csv
import time
import threading
import numpy as np

# ── Change this path to your dataset location ──────────────────────────────
PATH_DOC = '/home/labcoro/Documents/Berith/Test_01.csv'
# ───────────────────────────────────────────────────────────────────────────

from tactilesensors4.msg import StaticData, Dynamic


class TactileSensorSubscriber(Node):

    def __init__(self):
        super().__init__('tactile_sensor_subscriber')

        self.StacticData   = [0] * 28
        self.StacticDataS2 = [0] * 28
        self.DynamicData   = 0
        self.DynamicDataS2 = 0

        self.create_subscription(
            StaticData,
            'TactileSensor4/Sensor1/StaticData',
            self._cb_static_s1,
            10
        )
        self.create_subscription(
            StaticData,
            'TactileSensor4/Sensor2/StaticData',
            self._cb_static_s2,
            10
        )
        self.create_subscription(
            Dynamic,
            'TactileSensor4/Sensor1/Dynamic',
            self._cb_dynamic_s1,
            10
        )
        self.create_subscription(
            Dynamic,
            'TactileSensor4/Sensor2/Dynamic',
            self._cb_dynamic_s2,
            10
        )

        self.get_logger().info('TactileSensorSubscriber ready, waiting for data...')

    def _cb_static_s1(self, msg):
        self.StacticData = list(msg.value[:28])

    def _cb_static_s2(self, msg):
        self.StacticDataS2 = list(msg.value[:28])

    def _cb_dynamic_s1(self, msg):
        self.DynamicData = int(msg.value)

    def _cb_dynamic_s2(self, msg):
        self.DynamicDataS2 = int(msg.value)


# =============================================================================
# Recording logic
# =============================================================================

recording = False
ts = None

TITLES = [
    'tax1',  'tax2',  'tax3',  'tax4',  'tax5',  'tax6',  'tax7',
    'tax8',  'tax9',  'tax10', 'tax11', 'tax12', 'tax13', 'tax14',
    'tax15', 'tax16', 'tax17', 'tax18', 'tax19', 'tax20', 'tax21',
    'tax22', 'tax23', 'tax24', 'tax25', 'tax26', 'tax27', 'tax28',
    'S2tax1',  'S2tax2',  'S2tax3',  'S2tax4',  'S2tax5',  'S2tax6',
    'S2tax7',  'S2tax8',  'S2tax9',  'S2tax10', 'S2tax11', 'S2tax12',
    'S2tax13', 'S2tax14', 'S2tax15', 'S2tax16', 'S2tax17', 'S2tax18',
    'S2tax19', 'S2tax20', 'S2tax21', 'S2tax22', 'S2tax23', 'S2tax24',
    'S2tax25', 'S2tax26', 'S2tax27', 'S2tax28',
    'S1Dina', 'S2Dina'
]


def logData():
    global recording, ts

    with open(PATH_DOC, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, quoting=csv.QUOTE_ALL)
        writer.writerow(TITLES)

        BiasS1 = np.zeros((50, 28))
        BiasS2 = np.zeros((50, 28))
        k = 0

        while recording:
            time.sleep(0.05)  # 20 Hz

            lineS1 = np.asarray(ts.StacticData)
            lineS2 = np.asarray(ts.StacticDataS2)

            if k == 50:
                BiasS1 = np.mean(BiasS1, axis=0)
                BiasS2 = np.mean(BiasS2, axis=0)
                print('Bias calculated — start moving the sensor now.')

            totalData = np.concatenate((
                lineS1,
                lineS2,
                [np.int64(ts.DynamicData)],
                [np.int64(ts.DynamicDataS2)]
            ))
            writer.writerow(totalData)

            if k < 50:
                BiasS1[k, :] = lineS1[:28]
                BiasS2[k, :] = lineS2[:28]

            k += 1


# =============================================================================
# Entry point
# =============================================================================
def main():
    global recording, ts

    rclpy.init()
    ts = TactileSensorSubscriber()

    spin_thread = threading.Thread(target=rclpy.spin, args=(ts,), daemon=True)
    spin_thread.start()

    # Wait a moment to let subscriptions receive first data
    print('Waiting for sensor data...')
    time.sleep(2.0)
    print(f'Sensor2 check — first 4 values: {ts.StacticDataS2[:4]}')

    print('=' * 50)
    print('Tactile Sensor Recorder — ROS 2')
    print(f'Saving to: {PATH_DOC}')
    print('=' * 50)

    try:
        while True:
            recording = True
            log_thread = threading.Thread(target=logData, daemon=True)
            log_thread.start()

            input('\n[RECORDING]  Press Enter to stop...\n')

            recording = False
            log_thread.join()
            print('[STOPPED]  Data saved.')

    except KeyboardInterrupt:
        recording = False
        print('\nShutting down.')
    finally:
        ts.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()