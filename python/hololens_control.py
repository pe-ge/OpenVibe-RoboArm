from __future__ import print_function
import requests
from threading import Thread
from Queue import Queue
from time import sleep

class MyOVBox(OVBox):
	def __init__(self):
		OVBox.__init__(self)
	  
		self.url = 'HIDDEN'
		self.url_header = {
			'Authorization': 'HIDDEN'
		}
		
		self.hololens_thread = Thread(target=self._hololens_controller, name='HoloLens Controller')
		self.hololens_cmds = Queue()
		
		self.hololens_thread.start()
		
	def _hololens_controller(self):
		print('HoloLens: starting controller')
		while True:
			print('WAITING FOR CMD')
			cmd = self.hololens_cmds.get()
			print('CMD RECVD: ', cmd)
			if cmd == 'TRIGGER':
				print('HoloLens: received trigger')
				
				print('HoloLens: sending movement UP')
				success = self.move_up()
				
				if success:
					print('HoloLens: sleeping for 2s')
					sleep(2)
					
					print('HoloLens: sending movement DOWN')
					self.move_down()
				
			elif cmd == 'EXIT':
				print('HoloLens: stopping controller')
				break
			else:
				print('HoloLens: ERROR unknown cmd')

	def process(self):
		for chunkIndex in range(len(self.input[0])):
		    chunk = self.input[0].pop()
		    if(type(chunk) == OVStimulationSet):
				for stimIdx in range(len(chunk)):
					stim=chunk.pop();
					if stim.identifier == OpenViBE_stimulation['OVTK_StimulationId_SegmentStart']:
						self.hololens_cmds.put('TRIGGER')
				
	def move_up(self):
		return self._send_cmd(self.url % 'h')
		
	def move_down(self):
		return self._send_cmd(self.url % 'l')
		
	def _send_cmd(self, full_url):
		try:
			response = requests.request("POST", full_url, headers=self.url_header, data={}, timeout=2)
			print(response.text.encode('utf8'))
			return True
		except requests.Timeout:
			print('ERROR: HoloLens is not responding')
			return False
		
	def uninitialize(self):
		self.hololens_cmds.put('EXIT')
		

box = MyOVBox()