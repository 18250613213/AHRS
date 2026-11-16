import unittest
import importlib.util

SAMPLE = b'$BDFPDL,2445,120970.000,269.99044,-0.38092,0.63102,30.1234567,120.1234567,50.664,0.0165,0.0158,-0.0002,0.029,0.007,0.003,-0.110,-0.071,9.805,11,11,16,0,48,0*3E\r\n'


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('t02'), 'T02 parser is required')
        from t02 import T02Parser
        self.parser = T02Parser

    def test_real_packet_fields_and_units(self):
        data = self.parser.parse_frame(SAMPLE)
        self.assertNotIn('error', data)
        for key, value in {'yaw': 269.99044, 'pitch': -0.38092, 'roll': 0.63102,
                           'latitude': 30.1234567, 'longitude': 120.1234567,
                           'velocity_east': .0165, 'velocity_north': .0158,
                           'velocity_up': -.0002, 'gyro_x': .029,
                           'body_accel_z': 9.805, 'satellite_num_2': 11,
                           'system_state': 48, 'flight_state': 0}.items():
            self.assertEqual(data[key], value)

    def test_every_serial_split_and_noise_recovery(self):
        for split in range(len(SAMPLE)):
            first, remaining = self.parser.find_frames(b'noise\xff' + SAMPLE[:split])
            second, remaining = self.parser.find_frames(remaining + SAMPLE[split:] + SAMPLE)
            self.assertEqual(len(first + second), 2)
            self.assertEqual(remaining, b'')
        frames, _ = self.parser.find_frames(b'$BDFPDL,broken' + SAMPLE)
        self.assertEqual(len(frames), 1)
        self.assertNotIn('error', self.parser.parse_frame(frames[0]))

    def test_corruption_and_missing_checksum_rejected(self):
        self.assertIn('error', self.parser.parse_frame(SAMPLE.replace(b'269.99044', b'169.99044')))
        self.assertIn('error', self.parser.parse_frame(SAMPLE.split(b'*')[0]))

    def test_nonfinite_and_wrong_field_count_rejected(self):
        for body in [SAMPLE[1:].split(b'*')[0].replace(b'269.99044', b'nan'), b'BDFPDL,2445']:
            checksum = 0
            for byte in body:
                checksum ^= byte
            packet = b'$' + body + f'*{checksum:02X}'.encode()
            self.assertIn('error', self.parser.parse_frame(packet))


if __name__ == '__main__':
    unittest.main()
