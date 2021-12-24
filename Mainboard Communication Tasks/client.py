import websocket
import _thread
import time
import json
from types import SimpleNamespace

def on_message(ws, message):
    #print(message)
    x = json.loads(message, object_hook=lambda d: SimpleNamespace(**d))
    print(x.signal)
    if(x.signal == "start"):
        print(x.targets)
        print(x.baskets)
        if(x.targets[0] == "BTS"):
            print("selecting basket ", x.baskets[0])
        elif(x.targets[1] == "BTS"):
            print("selecting basket ", x.baskets[1])
        else:
            print("Not for BTS")
    elif(x.signal == "stop"):
        print(x.targets)
        if(x.targets[0] == "BTS" or x.targets[0] == "BTS"):
             print("Time to stop")
        else:
            print("Not for BTS")

def on_error(ws, error):
    print(error)

def on_close(ws, close_status_code, close_msg):
    print("### closed ###")

def on_open(ws):
    ''''def run(*args):
        for i in range(3):
            time.sleep(1)
            ws.send("Hello %d" % i)
        time.sleep(1)
        ws.close()
        print("thread terminating...")
    _thread.start_new_thread(run, ())'''

if __name__ == "__main__":
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp("ws://127.0.0.1:2300/",
                              on_open=on_open,
                              on_message=on_message,
                              on_error=on_error,
                              on_close=on_close)

    ws.run_forever()