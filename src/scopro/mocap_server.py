import time
import sys
from MOCAP.getrigidbody import NatNetClient
import datetime
import csv
import copy
from threading import Thread, Lock

import socket

body = {}
MocapIndex = [14,15,]
optionsDict = {}
optionsDict["clientAddress"] = "10.191.76.182"
optionsDict["serverAddress"] = "10.191.76.211"
optionsDict["use_multicast"] = False

loopsize = 10

class Mocap(Thread):
    def __init__(self,clientAddress,serverAddress):
        Thread.__init__(self)
        self.clientAddress = clientAddress
        self.serverAddress = serverAddress
        self.running = True
        # self.mocapval = []
        self.body = {}
        self.inrec = False
        self.lock = Lock()
        self.fifoloop = {}
        self.fiforec = {}
        for a in MocapIndex:
            self.fifoloop[a] = [[0.0] * 9] * loopsize
            self.fiforec[a] = 0

    def receive_new_frame(self,data_dict):
        dump_args = False
        if dump_args == True:
            out_string = "    "
            for key in data_dict:
                out_string += key + "="
                if key in data_dict :
                    out_string += data_dict[key] + " "
                out_string+="/"

    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receive_rigid_body_frame( self,new_id, position, rotation):
        self.lock.acquire()
        self.body[new_id] = [position, rotation]
        # print(new_id)
        if (new_id in MocapIndex) and self.inrec:
            self.mocap = [time.time(), new_id]
            self.mocap.extend(position)
            self.mocap.extend(rotation)
            self.fifoloop[new_id][self.fiforec[new_id]] = self.mocap
            self.fiforec[new_id] = (self.fiforec[new_id] + 1) % loopsize
            self.writer.writerow(self.mocap)
        self.lock.release()
            
    def getmocapval(self, mocapid,  temps = 0):
        value = []
        self.lock.acquire()
        # value = copy.copy(self.mocapval)
        if temps == 0:
            value = copy.copy(self.fifoloop[mocapid][self.fiforec[mocapid]])
        else:
            tempscomp = [x[0] > temps for x in self.fifoloop[mocapid]]
            try:
                findtrue = tempscomp.index(True)
            except ValueError:
                value = copy.copy(self.fifoloop[mocapid][self.fiforec[mocapid]])
                self.lock.release()
                return value

            if findtrue == 0:
                try:
                    fintrue = tempscomp.index(False)
                except ValueError:
                    value = copy.copy(self.fifoloop[mocapid][self.fiforec[mocapid]])
                    self.lock.release()
                    return value
                try:
                    findtrue = tempscomp[fintrue:].index(True)
                except ValueError:
                    value = copy.copy(self.fifoloop[mocapid][loopsize - 1])
                    self.lock.release()
                    return value
                value = copy.copy(self.fifoloop[mocapid][findtrue+fintrue-1])
            else:
                value = copy.copy(self.fifoloop[mocapid][findtrue - 1])

        self.lock.release()
        return value


    def rec_req(self, message):
        print ("rec request for Mocap " , message)
        if self.inrec and message == False:
            if self.file:
                self.file.close()
            self.inrec = False
        elif not self.inrec and message == True:
            self.file = open('rec_mocap_' + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f") + ".csv", 'w', newline='')
            self.writer = csv.writer(self.file)
            self.inrec = True
            self.writer.writerow(["time","MocapIndex","mocapX","mocapY","mocapZ","rotX","rotY","rotZ","rotW"])

    def add_lists(self,totals, totals_tmp):
        totals[0]+=totals_tmp[0]
        totals[1]+=totals_tmp[1]
        totals[2]+=totals_tmp[2]
        return totals
    
    def run(self):
        while self.running:
            # try:
            #     currentbody = self.body[MocapIndex]
            # except KeyError:
            #     time.sleep(0.2)
            #     continue
            # temps = time.time()
            # self.lock.acquire()
            # self.mocapval=[temps]
            # self.mocapval.extend(currentbody[0])
            # self.mocapval.extend(currentbody[1])
            # self.lock.release()
            # print(self.mocapval)
            # self.body_with_header.pose.position.x = body[MocapIndex][0][0]
            # self.body_with_header.pose.position.y = body[MocapIndex][0][1]
            # self.body_with_header.pose.position.z = body[MocapIndex][0][2]
            # self.body_with_header.pose.orientation.x = body[MocapIndex][1][0]
            # self.body_with_header.pose.orientation.y = body[MocapIndex][1][1]
            # self.body_with_header.pose.orientation.z = body[MocapIndex][1][2]
            # self.body_with_header.pose.orientation.w = body[MocapIndex][1][3]
            time.sleep(0.003)
    
    def terminate(self):
        """clean mocap stop"""
        print("killed")
        self.running = False

if __name__ == "__main__":
    
    # This will create a new NatNet client

    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    mymocap = Mocap(optionsDict["clientAddress"],optionsDict["serverAddress"])
    streaming_client.new_frame_listener = mymocap.receive_new_frame
    streaming_client.rigid_body_listener = mymocap.receive_rigid_body_frame
    streaming_client.set_print_level(0)

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        try:
            sys.exit(2)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    streaming_client.get_infos()
    time.sleep(1)

    mymocap.start()
    mymocap.rec_req(True)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Associer le socket à l'adresse et au port
        s.bind(('localhost', 11311))
        # Écouter les connexions entrantes
        s.listen()
        # Accepter une connexion
        conn, addr = s.accept()
        with conn:
            print('Connecté par', addr)
            try:
                while True:
                    time.sleep(0.1)
                    for index in MocapIndex:
                        print(mymocap.getmocapval(index))
                        conn.sendall((str(mymocap.getmocapval(index))).encode())
            except (KeyboardInterrupt, SystemExit):
                streaming_client.shutdown()
                mymocap.terminate()
            # streaming_client.shutdown()
    print("killed")
