import zmq
from flask import Flask
from misc.socketconfig import HOST, PORT

socket = zmq.Context().socket(zmq.PUB)
socket.bind(f'tcp://{HOST}:{PORT}')

app = Flask(__name__)

@app.route("/ping")
def ping():
    return "Pong!"


'''
try:
    string = cSocket.recv(flags=zmq.NOBLOCK)
    // safely received, nonblocking
except zme.Again as e:
    # No message waiting to be processed
'''

PLAYING = True

'''
Starts up the emotion game in the waiting client script.
Returns 200 if the game is successfully started
Returns 409 error if the game was already running or a connection available

STRETCH GOAL: If that script has crashed or isn't running, start it running now.
'''
@app.route("/start", methods=['PATCH'])
def start():
    if(PLAYING) return ('Emotion game already running!', 409)
    PLAYING = True
    socket.send('start_game')
    return ('Successfully started emotion game!', 200)

'''
Stops the current instance of the emotion game and waits for the game stats in the socket response.
Submits game stats to the database when received.

Returns 200 if the game was successfully stopped and the stats were able to be submitted to the database.

Returns 409 error if the game wasn't running
Returns 504 if the game doesn't stop and respond in the allocated time
Returns 503 if the database isn't available to submit
'''
@app.route("/stop", methods=['PATCH'])
def stop():
    if(!PLAYING) return ('Emotion game not running!', 409)
    PLAYING = False
    '''
    TODO
    '''
    return ('Successfully stopped emotion game and submitted play stats!", 200)

def submitToDatabase():
    '''
    TODO
    '''
    return