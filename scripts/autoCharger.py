#!/usr/bin/env python
from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
import zbar
import getpass
import requests
from requests_ntlm import HttpNtlmAuth
import untangle
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from icharger_usb.srv import SetCurrent
from recordtype import recordtype
import signal
import sys
import threading
import xml.etree.ElementTree as ET

ChannelStruct = recordtype("ChannelStruct", "charger channel pack_voltage cell_voltages charge status dv num_cells")
channels = []

# Capture Ctrl-C (seems to be necessary to escape from ROS)
def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	sys.exit(0)

# Capture channel status
def statusCallback(data, channel):
	channels[channel].status = data.data;

# Capture pack voltage
def packVoltageCallback(data, channel):
	channels[channel].pack_voltage = data.data;

# Capture cell voltages
def cellVoltagesCallback(data, channel):
	channels[channel].cell_voltages = data.data;
	minCell = float('inf');
	maxCell = float('-inf');
	channels[channel].num_cells = 0
	# Count channels, sanity check voltages and capture min/max
	for cell in channels[channel].cell_voltages:
		if cell > 2.5:
			channels[channel].num_cells+=1
		if cell < 1.0:
			break
		if cell > maxCell:
			maxCell = cell
		if cell < minCell:
			minCell = cell
	# Calculate dV
	if channels[channel].num_cells > 0:
		channels[channel].dv = maxCell - minCell
	
# Capture channel charge
def chargeCallback(data, channel):
	channels[channel].charge = data.data;

# Once charing has been initiated, this function is called in a thread to monitor the charging status
def monitorChannel(sharepointSession, logEtag, logID, ch):
	# Wait a while for charge to initialise
	sleep(10)
	comments = ""
	
	# Loop forever unless charge completes or is interrupted
	# TODO: Figure out if charger has any error status codes and check for these too
	while (True):
		if channels[ch].status == 40:
			print("Charger " + str(channels[ch].charger) + " channel " + str(channels[ch].channel) + " is done")
			break
		if channels[ch].status == 0 or channels[ch].status == 1:
			print("Charger " + str(channels[ch].charger) + " channel " + str(channels[ch].channel) + " interrupted")
			comments = "Incomplete, interrupted"
			break
		sleep(1)
	
	# Update charepoint record with post-charge data
	r = sharepointSession.post('https://teams.ljmu.ac.uk/7/Drones/Operations/_api/contextinfo')
	rootXML = ET.fromstring(r.text)
	digest = rootXML[1].text
	payload = "{{ '__metadata': {{ 'type': 'SP.Data.Battery_x0020_logsListItem'}},'Post_x002d_V': {postv},'Post_x002d_dV': {postdv}, 'Charge_x0020__x0028_Ah_x0029_': {charge}, 'Comments': '{comments}'}}".format(postv=channels[ch].pack_voltage,postdv=channels[ch].dv,charge=float(channels[ch].charge)/1000.0,comments=comments)
	r = sharepointSession.patch("https://teams.ljmu.ac.uk/7/Drones/Operations/_api/Web/Lists/GetByTitle('Battery logs')/getItemById('{logID}')".format(logID=logID), timeout=30, data=payload, headers={"X-RequestDigest":digest,"content-type": "application/json;odata=verbose", "X-Http-Method": "PATCH", "If-Match": logEtag})

# Main function
def autoCharger():
	# Initialise some things
	signal.signal(signal.SIGINT, signal_handler)
	camera = PiCamera(resolution = (480,320), framerate=30)
	rospy.init_node('autoCharger', anonymous=True)
	topics = sorted(rospy.get_published_topics())
	
	# Identify chargers available and subscribe to their topics
	# NOTE: This only runs once, because running repeatedly could break things
	#		All charger nodes need to be running before we start this node
	# TODO: Handle hotplugging of chargers gracefully
	channel = 0
	for topic in topics:
		if 'status' in topic[0]:
			topicFields = topic[0].split('/')
			chargerNumber = topicFields[1].split('_')[1]
			channelNumber = topicFields[2].split('_')[1]
			channels.append(ChannelStruct(chargerNumber, channelNumber, 0.0, [], 0, 0, 0.0, 0))
			rospy.Subscriber(topic[0], Int32, callback=statusCallback, callback_args = channel)
			rospy.Subscriber(topic[0].replace('status','pack_voltage'), Float32, callback=packVoltageCallback, callback_args = channel)
			rospy.Subscriber(topic[0].replace('status','cell_voltages'), Float32MultiArray, callback=cellVoltagesCallback, callback_args = channel)
			rospy.Subscriber(topic[0].replace('status','charge'), Int32, callback=chargeCallback, callback_args = channel)
			channel+=1
	
	# Loop forever
	while True:
		# Start a session for data logging
		sharepointSession = requests.Session()
		
		# Loop forever until a successful login is made
		while True:
			username = raw_input("Username: ")
			if username == "":
				continue
			password = getpass.getpass("Password: ")
			
			# Authenticate session then delete password immediately
			sharepointSession.auth = HttpNtlmAuth('USERS\\' + username,password)
			del(password)
			
			# Poll the battery table and make sure we're an authorised user
			r = sharepointSession.get("https://teams.ljmu.ac.uk/7/Drones/Operations/_api/Web/Lists/GetByTitle('Battery register')", timeout=30)
			if r.status_code == 200:
				print("Login successful")
				break;
			else:	# Otherwise try again
				print("Unable to log in, code " + str(r.status_code))
		
		# Prompt user to scan barcode
		print("Please scan barcode")
		sleep(3)
		
		# Display camera view
		camera.start_preview()
		
		# create a reader
		scanner = zbar.ImageScanner()

		# configure the reader
		scanner.parse_config('enable')

		while True:
			# Capture frame from camera
			stream = BytesIO()
			camera.capture(stream, format='jpeg', use_video_port=True)
			
			# "Rewind" the stream to the beginning so we can read its content
			stream.seek(0)
			
			# Pass image to zbars
			pil = Image.open(stream).convert('L')
			width, height = pil.size
			raw = pil.tobytes()
			image = zbar.Image(width, height, 'Y800', raw)
			scanner.scan(image)
			
			# Extract results
			batteryID = ""
			for symbol in image:
				batteryID = symbol.data
				print("Detected battery ID: " + batteryID)
				break
			if batteryID != "":
				break
			sleep(0.01)
		# Tidy up scannign resources
		del(image)
		del(stream)
		camera.stop_preview()
		
		# Look up battery data on sharepoint
		print("Looking up data")
		try: 
			r = sharepointSession.get("https://teams.ljmu.ac.uk/7/Drones/Operations/_api/Web/Lists/GetByTitle('Battery register')/items?$select=ID,Title,Brand,Model,Cells,Capacity_x0020__x0028_Ah_x0029_&$filter=Title eq '" + symbol.data + "'", timeout=30)
		except requests.Timeout, e:
			print("Error looking up data, please try again")
			continue
		batteryData = untangle.parse(r.text)
		spID = batteryData.feed.entry.content.m_properties.d_ID.cdata;
		brand = batteryData.feed.entry.content.m_properties.d_Brand.cdata;
		model = batteryData.feed.entry.content.m_properties.d_Model.cdata;
		cells = batteryData.feed.entry.content.m_properties.d_Cells.cdata;
		capacity = batteryData.feed.entry.content.m_properties.d_Capacity_x0020__x0028_Ah_x0029_.cdata;
		print(batteryID + ": " + brand + " " + model + " " + cells + "S " + capacity + "Ah")
		query = raw_input("Is this correct (y/N)?: ")
		
		# If information is correct, proceed
		if query.upper() == "Y":
			
			# Find an empty charging channel
			chIdx = 0;
			chargerNum = -1;
			channelNum = -1;
			for channel in channels:
				if channel.status == 0:
					chargerNum = channel.charger
					channelNum = channel.channel
					break
				chIdx += 1
			if chargerNum == -1 and channelNum == -1:
				print("No charger channels available, try agian later...")
				return
			print("Please connect to charger " + str(chargerNum) + " channel " + str(channelNum))
			
			# Wait until battery is connected (check both pack voltage and number of cells)
			while abs(sum(channels[chIdx].cell_voltages) - channels[chIdx].pack_voltage) > 0.2 and channels[chIdx].num_cells != float(cells):
				sleep(1)
			
			# Charge or storage
			option = raw_input("(C)harge or (S)torage?: ")
			
			# Sanity check pack health
			#if channels[chIdx].dv > 0.05:
			#	print("Pack has a dV of >50mV, do not charge and inform DFO as soon as possible")
			#	continue
			if channels[chIdx].pack_voltage < 3*float(cells) or channels[chIdx].pack_voltage > 4.2*float(cells):
				print("Pack voltage is out of range, do not charge and inform DFO as soon as possible")
				continue
			
			# Take the relevent actions
			action = ""
			if option.upper() == "C":
				action = "Charge"
			elif option.upper() == "S":
				action = "Storage"
			else:
				print("Unrecognised option, try again...")
				continue
			
			# Create new sharepoint record for this activity 
			r = sharepointSession.post('https://teams.ljmu.ac.uk/7/Drones/Operations/_api/contextinfo')
			rootXML = ET.fromstring(r.text)
			digest = rootXML[1].text
			payload = "{{ '__metadata': {{ 'type': 'SP.Data.Battery_x0020_logsListItem'}},'BatteryId': {battery},'Action': '{action}','Pre_x002d_V': {prev},'Pre_x002d_dV': {predv}}}".format(battery=spID,action=action,prev=channels[chIdx].pack_voltage,predv=channels[chIdx].dv)
			while True:
				try:
					r = sharepointSession.post("https://teams.ljmu.ac.uk/7/Drones/Operations/_api/Web/Lists/GetByTitle('Battery logs')/items", timeout=10, data=payload, headers={"X-RequestDigest":digest,"content-type": "application/json;odata=verbose"})
					break;
				except:
					sleep(1)
			
			# Returned record contains log ID and ETag, both necessary for updating record
			# Extract these and pass to monitoring thread
			if option.upper() == "C":
				setCurrent = rospy.ServiceProxy("/charger_" + str(chargerNum) +"/charge_current", SetCurrent)
				setCurrent(float(capacity))
				startCharge = rospy.ServiceProxy("/charger_" + str(chargerNum) + "/channel_" + str(channelNum) + "/start_charge", Trigger)
				startCharge()
				print("Charge started")
			elif option.upper() == "S":
				setCurrent = rospy.ServiceProxy("/charger_" + str(chargerNum) +"/charge_current", SetCurrent)
				setCurrent(float(capacity))
				setCurrent = rospy.ServiceProxy("/charger_" + str(chargerNum) +"/discharge_current", SetCurrent)
				setCurrent(float(capacity))
				startStorage = rospy.ServiceProxy("/charger_" + str(chargerNum) + "/channel_" + str(channelNum) + "/start_storage", Trigger)
				startStorage()
				print("Storage started")
			else:
				print("Unrecognised option, try again...")
				continue
			batteryLogData = untangle.parse(r.text)
			logEtag = batteryLogData.entry['m:etag']
			logID = batteryLogData.entry.content.m_properties.d_ID.cdata;
			t = threading.Thread(target=monitorChannel, args=(sharepointSession,logEtag,logID,chIdx,))
			t.daemon = True
			t.start()
		else:
			print("Restarting...")
			continue

# ROS main
if __name__ == '__main__':
	try:
		autoCharger()
	except rospy.ROSInterruptException:
		pass