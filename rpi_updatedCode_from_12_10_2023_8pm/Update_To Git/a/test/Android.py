import time
import bluetooth

class Android(object):
    def __init__(self):
        self.isConnected = False
        self.client = None
        self.server = None
        RFCOMM_channel = 1
        uuid = '49930a2c-04f6-4fe6-beb7-688360fc5995' #Need to find out UUID of tablet

        self.server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.server.bind(("",RFCOMM_channel))
        self.server.listen(RFCOMM_channel)
        self.port = self.server.getsockname()[1]

        bluetooth.advertise_service(self.server,"MDP-Group12-RPi",service_id=uuid,service_classes=[uuid,bluetooth.SERIAL_PORT_CLASS],profiles=[bluetooth.SERIAL_PORT_PROFILE],protocols=[bluetooth.OBEX_UUID])

    def getisConnected(self):
        return self.isConnected
    
    def connect(self):
        print(f"Waiting for connection with Android Tablet on RFCOMM channel {self.port} ...")
        while self.isConnected == False:
            try:
                if self.client is None:
                    # Accepts the connection
                    self.client, address = self.server.accept()
                    print(f"[SUCCESSFUL CONNECTION]: Successfully established connection with Android Tablet from {address}.")
                    self.isConnected = True
                    
            except Exception as e:
                print(f"[ERROR] Unable to establish connection with Android: {str(e)}")
                self.client.close()
                self.client = None
                # Retry to connect
                print("Retrying to connect with Android Tablet ...")
                time.sleep(1)
                self.connect()


    def disconnect(self):
        try:
            print("Android: Shutting down Bluetooth Server ...")
            self.server.close()
            self.server = None
            print("Android: Shutting down Bluetooth Client ...")
            self.client.close()
            self.client = None
            self.isConnected = False
        except Exception as e:
            print("f[ERROR]: Unable to disconnect from Android: {str(e)}")

    def read(self):
        try:
            while True:
                msg = self.client.recv(1024).strip().decode('utf-8')
                if len(msg) == 0:
                    continue
                print(f"[FROM ANDROID] {msg}")
                return msg

        except Exception as e:
            print(f"[ERROR] Android read error: {str(e)}")
            # Retry connection if Android gets disconnected
            try:
                self.socket.getpeername()
            except:
                self.client.close()
                self.isConnected = False
                self.client = None
                print("Retrying to connect with Android Tablet ...")
                self.connect()
                print("Trying to read message from Android again...")
                msg = self.read()
        return msg

    def write(self, message):
        try:
            if self.isConnected:
                self.client.send(message)
                print(f"[SENT TO ANDROID]: {message}")
            else:
                print("[Error]  Connection with Android Tablet is not established")

        except Exception as e:
            print(f"[ERROR] Android write error: {str(e)}")
            # Retry connection if Android gets disconnected
            try:
                self.socket.getpeername()
            except:
                self.client.close()
                self.isConnected = False
                self.client = None
                print("Retrying to connect with Android Tablet...")
                self.connect()
                print("Trying to send the message to Android again...")
                self.write(self,message) # try writing again