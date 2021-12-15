import websocket
import _thread
import time
import json
from types import SimpleNamespace
import threading
def wsConnection(sharedData, connectionString = "ws://172.19.154.87:2300/"):
    global shared_data
    shared_data = sharedData
    ws_thread_stop = False
    def on_message(ws, message):
        try:
            global shared_data
            #print(message)
            x = json.loads(message, object_hook=lambda d: SimpleNamespace(**d))
            #print(x.signal)
            if(x.signal == "start"):
                print(x.targets)
                print(x.baskets)
                if(x.targets[0] == "BTS"):
                    #print("selecting basket ", x.baskets[0])
                    shared_data['targetColor'] = x.baskets[0]
                    shared_data['gameState'] = 0
                elif(x.targets[1] == "BTS"):
                    #print("selecting basket ", x.baskets[1])
                    sharedData['targetColor'] = x.baskets[1]
                    shared_data['gameState'] = 0                   
                else:
                    print("Not for BTS")
            elif(x.signal == "stop"):
                #print(x.targets)
                if(x.targets[0] == "BTS" or x.targets[1] == "BTS"):
                    shared_data['gameState'] = -1
                    print("Time to stop")
                else:
                    print("Not for BTS")
        except Exception as e: print(e)

    def on_error(ws, error):
        print(error)

    def on_close(ws, close_status_code, close_msg):
        print("### closed ###")
        
    def threadEnd(sharedData,ws):
        global shared_data
        while(True):
            time.sleep(0.1)
            if(shared_data['wsStop'] == True):
                shared_data['wsStop'] = False
                ws.close()
                break
                
    def on_open(ws):
        global shared_data
        stopThread = threading.Thread(target = threadEnd, args = (shared_data,ws))
        stopThread.start()
        
    websocket.enableTrace(False)
    print("Connecting to ",connectionString)
    ws = websocket.WebSocketApp(connectionString,
                              on_open=on_open,
                              on_message=on_message)
    ws.run_forever()
   
