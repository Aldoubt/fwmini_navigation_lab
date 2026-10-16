"""Standard-library unit/integration tests; never touch a real CAN socket."""
import json
from pathlib import Path
from unittest.mock import patch
import subprocess
from tempfile import TemporaryDirectory
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from server import Records, Handler, ThreadingHTTPServer, TASKS, probe


def get(url):
    with urlopen(url, timeout=3) as resp:
        return resp.read(), resp.headers


class StorageTests(unittest.TestCase):
    def test_persistent_status_events_and_independent_lanes(self):
        with TemporaryDirectory() as td:
            db = Records(td)
            baseline = db.active
            db.set_status('p01_can_interface', 'BLOCKED', 'CAN 网卡未连接', 'can0.txt')
            db.set_status('p06_femto_usb', 'PASS', '相机已验证', 'camera.log')
            rec = Records(td).state()
            self.assertEqual(Records(td).active, baseline)
            self.assertEqual(rec['steps']['p01_can_interface']['status'], 'BLOCKED')
            self.assertEqual(rec['steps']['p06_femto_usb']['status'], 'PASS')
            self.assertIn('BLOCKED', db.report('md').decode())
            self.assertIn('camera.log', db.report('csv').decode('utf-8-sig'))
            events = (db.folder() / 'events.jsonl').read_text().splitlines()
            self.assertEqual(len(events), 3)
            db.new_session('下一次测试','实验室')
            self.assertNotIn('p01_can_interface', db.state()['steps'])
            self.assertTrue((Path(td) / baseline / 'session.json').is_file())

    def test_no_probe_auto_pass(self):
        with TemporaryDirectory() as td:
            db=Records(td)
            filename=db.store_probe('system','read only check')
            self.assertTrue((db.folder()/filename).is_file())
            self.assertEqual(db.state()['steps'], {})
            with self.assertRaises(ValueError):
                db.set_status('invalid', 'PASS')
            with self.assertRaises(ValueError):
                db.set_status('p00_environment', 'APPROVED')
            with self.assertRaises(ValueError):
                db.store_probe('../../bad','no')

    def test_only_predefined_probe(self):
        with self.assertRaises(ValueError):
            probe('bash -c echo runaway')
        self.assertIn('READ ONLY',probe('storage'))

    def test_no_camera_no_rgbd_bag_even_when_tf_exists(self):
        with TemporaryDirectory() as td:
            db=Records(td)
            fake=subprocess.CompletedProcess(['ros2','topic','list'],0,'/tf\n/clock\n','')
            with patch('server.shutil.which',return_value='/usr/bin/ros2'), \
                 patch('server.subprocess.run',return_value=fake), \
                 patch('server.subprocess.Popen') as spawn:
                for profile in ('rgbd','core','slam'):
                    with self.assertRaisesRegex(ValueError,'缺少核心话题'):
                        db.start_bag(profile)
                spawn.assert_not_called()
            self.assertEqual(db.state()['bag_runs'], [])

    def test_bag_reject_unknown_profile(self):
        with TemporaryDirectory() as td:
            db=Records(td)
            with self.assertRaises(ValueError):
                db.start_bag('../../tmp')
            self.assertEqual(db.state()['bag_runs'], [])


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory()
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.server.db=Records(self.tmp.name)
        self.server.token='dummy_for_test'
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()
        self.base=f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.tmp.cleanup()

    def post(self,path,data,token=True):
        req=Request(self.base+path,json.dumps(data).encode(),
                    {'Content-Type':'application/json',
                     'X-Workbench-Token':'dummy_for_test' if token else 'bad'},method='POST')
        with urlopen(req,timeout=4) as resp:
            return json.loads(resp.read())

    def test_ui_api_and_csv_export(self):
        html,_=get(self.base+'/')
        self.assertNotIn(b'__CSRF_TOKEN__', html)
        self.assertIn(b'dummy_for_test',html)
        state=json.loads(get(self.base+'/api/state')[0])
        self.assertEqual(len(state['tasks']),len(TASKS))
        self.assertEqual(len(state['session']['steps']),0)
        self.post('/api/status',dict(step='p01_can_interface',status='BLOCKED',note='网卡未连接',evidence='log.txt'))
        self.post('/api/status',dict(step='p06_femto_usb',status='SKIPPED',note='先搞RTAB',evidence=''))
        report,_=get(self.base+'/api/report.csv')
        self.assertIn('BLOCKED',report.decode('utf-8-sig'))
        self.assertIn('log.txt',report.decode('utf-8-sig'))
        self.assertIn('网卡未连接',get(self.base+'/api/report.md')[0].decode())
        state=json.loads(get(self.base+'/api/state')[0])
        self.assertEqual(len(state['session']['steps']),2)
        self.post('/api/new',dict(title='外场第二轮',place='农科院'))
        self.assertEqual(len(json.loads(get(self.base+'/api/state')[0])['session']['steps']),0)
        self.assertEqual(len(json.loads(get(self.base+'/api/state')[0])['sessions']),2)
        original=state['session']['id']
        self.post('/api/open',dict(id=original))
        self.assertEqual(len(json.loads(get(self.base+'/api/state')[0])['session']['steps']),2)

    def test_write_reject_without_token_and_unknown_probe(self):
        with self.assertRaises(HTTPError) as ctx:
            self.post('/api/status',dict(step='p00_environment',status='PASS'),token=False)
        self.assertEqual(ctx.exception.code,403)
        with self.assertRaises(HTTPError) as ctx:
            self.post('/api/probe',dict(probe='dangerous_shell_input'))
        self.assertEqual(ctx.exception.code,400)
        with self.assertRaises(HTTPError) as ctx:
            self.post('/api/open',dict(id='../../etc/passwd'))
        self.assertEqual(ctx.exception.code,400)
        self.assertEqual(self.server.db.state()['steps'],{})

if __name__=='__main__': unittest.main()
