from flask import Flask, render_template, Response, request, jsonify, send_from_directory, redirect, url_for
import cv2
import threading
import time
import os
import signal
import json
from subprocess import check_output
import logging
import socket
import base64
import numpy as np

# from OpenSSL import SSL

DEBUG = False

actual_app_state = 'Preview'
log_started = threading.Event()
log_started.clear()
log_proc = 0
anim_counter = 0
_, stop_image = cv2.imencode('.jpg', cv2.imread('templates/stop.jpg'))
wait_frame = cv2.imread('templates/wait.jpg')
wait_frame = cv2.resize(wait_frame, (640, 480))
wait_frame = cv2.flip(wait_frame, -1)
wait_frame = cv2.flip(wait_frame, 1)
_, jpg_wait_frame = cv2.imencode('.jpg', wait_frame)

HOST, PORT = "localhost", 7000
RCV_BUFFER_LEN = 10000000


# context = SSL.Context(SSL.TLSv1_METHOD)
# context.use_privatekey_file('server.key')
# context.use_certificate_file('server.crt')


def get_pid(name):
    return int(check_output(["pidof", name]))


class SLAMReader(object):
    def __init__(self):
        self.stop_now = False
        if DEBUG:
            self.fifo = None
            self.vid = None
        else:
            self.slam_socket = None
        self.package = None
        self.grabbed = False
        self.proc = threading.Thread(target=self.update, args=())
        self.proc.start()
        self.frame = stop_image
        self.stop_image = stop_image.tobytes()
        self.wait_image = jpg_wait_frame.tobytes()

    def __del__(self):
        if not DEBUG:
            self.slam_socket.close()

    def get_frame(self):
        if self.stop_now:
            image = self.stop_image
        else:
            image = self.frame
        self.grabbed = False
        if self.stop_now:
            self.grabbed = True
        return image  # .tobytes()

    def get_data(self):
        if self.stop_now:
            data = None
        else:
            data = {i: self.package[i] for i in self.package if i != 'frame'}
        return data

    def stop(self):
        if not DEBUG:
            self.slam_socket.close()
        else:
            self.vid.release()
        self.stop_now = True
        self.grabbed = True
        self.proc.join()

    def update(self):
        if DEBUG:
            self.vid = cv2.VideoCapture('video_slam.avi')
            saver_frame_counter = 0
        while True:
            if log_started.is_set():
                if not DEBUG:
                    self.slam_socket.send(b'next')
                    pack = self.slam_socket.recv(24)
                    pack_len = int(pack.decode("utf-8").lstrip().split(' ')[0])
                    pack = []
                    cnt = 0
                    while cnt < pack_len:
                        dat = self.slam_socket.recv(RCV_BUFFER_LEN)
                        cnt = cnt + len(dat)
                        pack.append(dat)
                    pack = b''.join(pack)
                    self.package = json.loads(pack)
                    buf_decode = base64.b64decode(self.package['frame'])
                    jpg = np.fromstring(buf_decode, np.uint8).tobytes()
                    # img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
                    # _, jpg = cv2.imencode('.jpg', img)
                    self.frame = jpg
                    self.grabbed = True
                else:
                    ret, img = self.vid.read()
                    saver_frame_counter += 1
                    if saver_frame_counter == self.vid.get(cv2.CAP_PROP_FRAME_COUNT):
                        saver_frame_counter = 0
                        self.vid.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    if ret:
                        _, jpg = cv2.imencode('.jpg', img).tobytes()
                        self.package = {'yaw': 1.0, 'pitch': 2.0, 'roll': 3.0, 'adc': 100.0}
                        self.frame = jpg
                        self.grabbed = True
            else:
                time.sleep(0.1)
                self.grabbed = True
                self.package = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'adc': 0.0}
                self.frame = self.wait_image
        if not DEBUG:
            pass
        else:
            self.vid.release()


slam_reader = SLAMReader()
app = Flask(__name__, static_url_path='', static_folder='static', template_folder='templates')
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


def gen_frames():
    while True:
        frame = slam_reader.get_frame()  # read the camera frame
        if slam_reader.grabbed:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result


def start_logging():
    global slam_reader
    global actual_app_state
    global log_started
    os.system('/home/step305/testServer/testServer &')
    time.sleep(10)
    log_started.set()
    slam_reader.slam_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    slam_reader.slam_socket.connect((HOST, PORT))
    actual_app_state = 'Logger::running...'


def stop_logging():
    global log_proc
    global log_started
    if log_started.is_set():
        log_started.clear()
        time.sleep(1)
        log_proc.join()
        slam_reader.slam_socket.close()
        pid = get_pid('testServer')
        os.kill(pid, signal.SIGINT)


@app.route("/utils.js")
def send_js():
    return send_from_directory('static', 'utils.js')


@app.route("/main.css")
def send_css():
    return send_from_directory('static', 'main.css')


@app.route("/command", methods=['POST'])
def parse_command():
    global actual_app_state
    global log_proc
    source = str(request.referrer).split('/')[-1]
    if source == 'logger':
        if request.form.get('action') == 'start':
            log_proc = threading.Thread(target=start_logging)
            log_proc.start()
            actual_app_state = "Logger::starting...Wait..."
        elif request.form.get('action') == 'stop':
            stop_logging()
            actual_app_state = "Logger::Preview"
    elif source == 'caruseling':
        if request.form.get('action') == 'start':
            actual_app_state = "Caruseling:: starting...Wait..."
        elif request.form.get('action') == 'stop':
            actual_app_state = "Caruseling::Preview"
    elif source == 'maytagging':
        if request.form.get('action') == 'start':
            actual_app_state = "Maytagging:: starting...Wait..."
        elif request.form.get('action') == 'stop':
            actual_app_state = "Maytagging::Preview"
    elif source == 'compassing':
        if request.form.get('action') == 'start':
            actual_app_state = "Compassing:: starting...Wait..."
        elif request.form.get('action') == 'stop':
            actual_app_state = "Compassing::Preview"

    return redirect(request.referrer)


@app.route("/", methods=['GET'])
def index():
    return redirect(url_for('logger'))


@app.route("/logger", methods=['GET'])
def logger():
    global actual_app_state
    global log_proc
    source = str(request.referrer).split('/')[-1]
    if source != 'logger':
        stop_logging()
        actual_app_state = "Logger::Preview"
    return render_template("base.html", actual_app_state=actual_app_state, path='logger')


@app.route("/caruseling", methods=['GET'])
def caruseling():
    global actual_app_state
    global log_proc
    source = str(request.referrer).split('/')[-1]
    if source != 'caruseling':
        stop_logging()
        actual_app_state = "Caruseling::Preview"
    return render_template("base.html", actual_app_state=actual_app_state, path='caruseling')


@app.route("/maytagging", methods=['GET'])
def maytagging():
    global actual_app_state
    global log_proc
    source = str(request.referrer).split('/')[-1]
    if source != 'maytagging':
        stop_logging()
        actual_app_state = "Maytagging::Preview"
    return render_template("base.html", actual_app_state=actual_app_state, path='maytagging')


@app.route("/compassing", methods=['GET'])
def compassing():
    global actual_app_state
    global log_proc
    source = str(request.referrer).split('/')[-1]
    if source != 'compassing':
        stop_logging()
        actual_app_state = "Compassing::Preview"
    return render_template("base.html", actual_app_state=actual_app_state, path='compassing')


@app.route('/timer', methods=['GET'])
def stuff():
    global actual_app_state
    current_time = time.strftime("%H:%M:%S\n")
    return jsonify(timer_value=current_time, actual_app_state=actual_app_state)


@app.route('/test')
def test():
    return render_template('test.html')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream_data')
def stream_data():
    global anim_counter
    anim_frames = ['---', '\\/\\', '|||', '/\\/']
    anim_counter = anim_counter + 1
    if anim_counter == len(anim_frames):
        anim_counter = 0

    yaw = anim_frames[anim_counter]
    pitch = anim_frames[anim_counter]
    roll = anim_frames[anim_counter]
    adc_value = anim_frames[anim_counter]
    if log_started.is_set():
        data = slam_reader.get_data()
        if data is not None:
            yaw = '{:.2f}deg'.format(data['yaw'])
            pitch = '{:.2f}deg'.format(data['pitch'])
            roll = '{:.2f}deg'.format(data['roll'])
            adc_value = '{:.2f}deg/hr'.format((data['adc']))
    return jsonify(yaw_value=yaw, pitch_value=pitch, roll_value=roll, adc_value=adc_value)


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=8000, threaded=True)  # , ssl_context=context)
    slam_reader.stop()
