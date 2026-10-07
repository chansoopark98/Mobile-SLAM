import hashlib,json,struct,subprocess,tempfile,unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
class InputOracleTest(unittest.TestCase):
    def test_exact_gray_and_float64_packet_sha(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);dataset=root/'dataset';config=root/'config.yaml';config.write_text('test: 1\n')
            for name in ('cam0','imu0','mocap0'):
                folder=dataset/'mav0'/name;folder.mkdir(parents=True);(folder/'data.csv').write_text('#header\n')
            gray=b'\x00\x40\x80\xff';packet=struct.pack('<7d',1520216380.2800002,1,2,3,4,5,6)
            raw=root/'input.bin';raw.write_bytes(struct.pack('<I',4)+gray+struct.pack('<I',1)+packet)
            output=root/'manifest.json'
            subprocess.run(['python3',str(ROOT/'scripts/dev/hash-inputs.py'),'--raw',str(raw),'--dataset',str(dataset),'--config',str(config),'--output',str(output)],check=True,capture_output=True)
            data=json.loads(output.read_text());self.assertEqual(data['frame_count'],1)
            frame=data['frames'][0];self.assertEqual(frame['imageSha256'],hashlib.sha256(gray).hexdigest());self.assertEqual(frame['imuSha256'],hashlib.sha256(packet).hexdigest());self.assertEqual(frame['imuCount'],1);self.assertEqual(frame['imuFirstTimestamp'],1520216380.2800002)
    def test_truncated_input_is_not_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);dataset=root/'dataset';config=root/'config.yaml';config.write_text('test: 1\n')
            for name in ('cam0','imu0','mocap0'):
                folder=dataset/'mav0'/name;folder.mkdir(parents=True);(folder/'data.csv').write_text('#header\n')
            raw=root/'input.bin';raw.write_bytes(b'\x01\x00');output=root/'manifest.json'
            result=subprocess.run(['python3',str(ROOT/'scripts/dev/hash-inputs.py'),'--raw',str(raw),'--dataset',str(dataset),'--config',str(config),'--output',str(output)],capture_output=True)
            self.assertNotEqual(result.returncode,0);self.assertFalse(output.exists())
if __name__=='__main__':unittest.main()
