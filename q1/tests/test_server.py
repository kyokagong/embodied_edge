import http.client
import json
import threading
import unittest
try:
    from q1_edge.server import create_server
except ImportError:
    create_server = None

class ControllerBoundary:
    def __init__(self):self.commands=[]
    def state(self):return {'backend':'mock','enabled':False}
    def action(self,name,client_id=None):
        if name not in ['stop','enable']:raise ValueError('Unknown action')
        self.commands.append(name);return self.state()
    def input(self,payload):self.commands.append('input');return self.state()
    def pico_frame(self,payload,source='simulator'):self.commands.append(source);return self.state()

class ServerTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(create_server,'Local API is not implemented')
        self.controller=ControllerBoundary();self.server=create_server(self.controller,port=0)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.port=self.server.server_address[1]
        self.base=f'http://127.0.0.1:{self.port}'
    def tearDown(self):
        if hasattr(self,'server'):self.server.shutdown();self.server.server_close();self.thread.join()
    def request(self,path='/api/action',body=None,headers=None,method='POST'):
        data=json.dumps(body or {'action':'stop'}).encode()
        hdr={'Content-Type':'application/json','X-Q1-Token':self.server.api_token,'Origin':self.base}
        hdr.update(headers or {})
        c=http.client.HTTPConnection('127.0.0.1',self.port,timeout=3)
        c.request(method,path,body=None if method=='GET' else data,headers=hdr)
        r=c.getresponse();result=r.status,r.read();c.close();return result
    def test_state_is_json_and_no_motion(self):
        status,payload=self.request('/api/state',method='GET')
        self.assertEqual(status,200);self.assertFalse(json.loads(payload)['enabled']);self.assertEqual(self.controller.commands,[])
    def test_command_requires_token(self):
        status,_=self.request(headers={'X-Q1-Token':'bad'})
        self.assertEqual(status,403);self.assertEqual(self.controller.commands,[])
    def test_foreign_origin_cannot_command(self):
        status,_=self.request(headers={'Origin':'https://malicious.invalid'})
        self.assertEqual(status,403);self.assertEqual(self.controller.commands,[])
    def test_plain_form_cannot_command(self):
        status,_=self.request(headers={'Content-Type':'text/plain'})
        self.assertEqual(status,415);self.assertEqual(self.controller.commands,[])
    def test_local_command_returns_result(self):
        status,payload=self.request(body={'action':'stop','client_id':'a'})
        self.assertEqual(status,200);self.assertTrue(json.loads(payload)['ok']);self.assertEqual(self.controller.commands,['stop'])
    def test_network_cannot_claim_native_pico_source(self):
        status,_=self.request('/api/pico',body={'source':'xrobotoolkit'})
        self.assertEqual(status,400);self.assertEqual(self.controller.commands,[])
    def test_oversize_request_rejected(self):
        status,_=self.request(body={'action':'stop','padding':'a'*40000})
        self.assertEqual(status,413);self.assertEqual(self.controller.commands,[])
    def test_dns_rebinding_host_rejected(self):
        status,_=self.request(headers={'Host':'malicious.invalid'})
        self.assertEqual(status,403);self.assertEqual(self.controller.commands,[])
    def test_action_cannot_omit_or_empty_client_identity(self):
        for client in (None,'',34):
            status,_=self.request(body={'action':'enable','client_id':client})
            self.assertEqual(status,400)
        self.assertEqual(self.controller.commands,[])

if __name__=='__main__':unittest.main()
