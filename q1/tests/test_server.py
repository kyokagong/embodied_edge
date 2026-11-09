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
    def input(self,payload,compact=False):
        self.commands.append('input')
        return {'sequence':payload.get('sequence')} if compact else self.state()
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
    def test_malformed_url_is_logged_and_returns_bad_request(self):
        with self.assertLogs('q1_edge.server', level='WARNING') as logs:
            status,_=self.request('http://[', headers={'Host':f'127.0.0.1:{self.port}'})
        self.assertEqual(status,400)
        self.assertIn('api_rejected', '\n'.join(logs.output))
        self.assertEqual(self.controller.commands,[])

    def test_compact_input_ack_and_reusable_http_connection(self):
        c=http.client.HTTPConnection('127.0.0.1',self.port,timeout=3)
        headers={'Host':f'127.0.0.1:{self.port}','Content-Type':'application/json',
                 'X-Q1-Token':self.server.api_token,'Origin':self.base,'X-Q1-Compact':'1'}
        for seq in (7,8):
            c.request('POST','/api/input',body=json.dumps({'sequence':seq}),headers=headers)
            response=c.getresponse();payload=json.loads(response.read())
            self.assertEqual(response.status,200)
            self.assertEqual(response.version,11)
            self.assertNotIn('state',payload)
            self.assertEqual(payload['ack']['sequence'],seq)
            self.assertIsNotNone(c.sock)
            if seq==7: first_socket=c.sock
            else:self.assertIs(c.sock,first_socket)
        c.close()
        self.assertEqual(self.controller.commands,['input','input'])

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
