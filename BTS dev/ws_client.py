#B_T_S Final Code#
#January 26th 2022#

import websocket
import _thread
import time
import json
from types import SimpleNamespace
import threading
#Shared data is common object for multiple threads. Thus, it chenges affects all processes.
class wsConnectionClass:
    def __init__(self, shared_data):
        self.shared_data = shared_data

    def wsConnection(self, connectionString = "ws://172.19.153.139:2300/"):
        self.shared_data["wsThreadUp"] = True
        ws_thread_stop = False
        websocket.enableTrace(False)
        print("Connecting to ",connectionString)
        ws = websocket.WebSocketApp(connectionString,
                                  on_open=self.on_open,
                                  on_message=self.on_message,
                                  on_error=self.on_error,
                                  on_close=self.on_close)
        ws.run_forever()

    def on_message(self,ws, message):
        try:
            #print(message)
            x = json.loads(message, object_hook=lambda d: SimpleNamespace(**d))
            #print(x.signal)
            #Print can be renived easly. Shared data is checked in other code parts to perform smthing.
            if(x.signal == "start"):
                print(x.targets)
                print(x.baskets)
                for i in range(len(x.targets)):
                    if(x.targets[i] == self.shared_data['name']):
                        #print("selecting basket ", x.baskets[0])
                        self.shared_data['targetColor'] = x.baskets[i]
                        self.shared_data['gameState'] = 0
            elif(x.signal == "stop"):
                for i in range(len(x.targets)):
                    if(x.targets[i] == self.shared_data['name']):
                        self.shared_data['gameState'] = -1
                        print("Time to stop")
        except Exception as e: print(e)

    def on_error(self,ws, error):
        self.shared_data["wsThreadUp"] = False
        ws.close()
        print(error)

    def on_close(self,ws, close_status_code, close_msg):
        self.shared_data["wsThreadUp"] = False
        ws.close()
        #print(self.shared_data["wsThreadUp"])
        print("### closed ###")

    #Fancy exit on connection loss.
    def threadEnd(self,ws):
        while(True):
            time.sleep(0.1)
            if(self.shared_data['wsStop'] == True):
                self.shared_data['wsStop'] = False
                self.shared_data["wsThreadUp"] = False
                ws.close()
                break

    def on_open(self,ws):
        stopThread = threading.Thread(target = self.threadEnd, args = [ws])
        stopThread.start()
