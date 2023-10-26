# ==========================================================
# Main script For Week 8 Task
# Managing the communication between RPi and STM, RPi and Android
# ===============================================================

from multiprocessing import Process, Queue   # Manage multi-thread programming
import time
import signal

from Android import Android

import base64
import os
import glob
import socket


def readMsg(queue, interface):
	while True:
		try:
			msg = interface.read()
			if msg is None:
				continue
			queue.put(msg)
		except Exception as e:
			print(f"[READ ERROR]: {str(e)}")

# Take photo


class TimeoutException(Exception):   # Custom exception class
	pass


def break_after(seconds=2):
	def timeout_handler(signum, frame):   # Custom signal handler
		raise TimeoutException

	def function(function):
		def wrapper(*args, **kwargs):
			signal.signal(signal.SIGALRM, timeout_handler)
			signal.alarm(seconds)
			try:
				res = function(*args, **kwargs)
				signal.alarm(0)      # Clear alarm
				return res
			except TimeoutException:
				print(f'Oops, timeout: %s sec reached.' %
								seconds, function.__name__, args, kwargs)
				return
		return wrapper
	return function


@break_after(3)
def readSTM(command):
	data = ""
	while True:
		data = interfaces[STM].read()
		print(f"DATA FROM STM: {data}")
		if data:
			return


# ===============================================================================================

if __name__ == '__main__':
	print("[MAIN] Initialising Multiprocessing Communication ...")

	# List of Interfaces - STM32F board, Android
	interfaces = []
	interfaces.append(Android())

	# Index of the interfaces in the list
	ANDROID = 0

	# Set up a queue to support manage messages received by the interfaces
	queue = Queue()

	# Create Process objects
	# stm = Process(target=readMsg, args=(queue, interfaces[STM]))
	android = Process(target=readMsg, args=(queue, interfaces[ANDROID]))

	# Establish connections between RPi and all other interfaces
	for interface in interfaces:
		interface.connect()  # connect STM first, then Android
	android.start()  # Starts to receive message from Android
	print("[MAIN] Multiprocess communication started.")
	while True:
		msg = queue.get()
		if(msg != ''):
			print(f"am i getting any msg: {msg}")
	print("all connection success!")