import asyncio
import time
import websockets

# create handler for each connection

async def handler(websocket, path):
    while(1==1):
        #data = await websocket.recv()

        #reply = f"Data recieved as:  {data}!"
        #print(reply)
        gameStart = """{
              "signal": "start",
              "targets":  ["BTS", "001TRT"],
              "baskets": ["magenta", "blue"]
            }"""
        await websocket.send(gameStart)
        time.sleep(1)
        gameEnd = """{
              "signal": "stop",
              "targets":  ["BTS", "001TRT"]
            }"""
        await websocket.send(gameEnd)
        time.sleep(1)
 

start_server = websockets.serve(handler, "127.0.0.1", 2300)

asyncio.get_event_loop().run_until_complete(start_server)

asyncio.get_event_loop().run_forever()