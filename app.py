from flask import Flask, render_template, Response, request, jsonify, send_from_directory
import cv2
import threading
import time
import os
import signal
import json
from subprocess import check_output
# from OpenSSL import SSL

DEBUG = True

actual_app_state = 'Preview'
log_proc = 0
_, stop_image = cv2.imencode('.jpg', cv2.imread('templates/stop.jpg'))


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
            self.fifo = open('/home/sergey/SLAM_FIFO.tmp')
        self.package = None
        self.grabbed = True
        self.proc = threading.Thread(target=self.update, args=())
        self.proc.start()

    def __del__(self):
        if not DEBUG:
            self.fifo.close()

    def get_frame(self):
        if self.stop_now:
            image = stop_image
        else:
            image = self.package["frame"]
        self.grabbed = False
        if self.stop_now:
            self.grabbed = True
        return image.tobytes()

    def get_data(self):
        if self.stop_now:
            data = None
        else:
            data = {i:self.package[i] for i in self.package if i != 'frame'}
        return data

    def stop(self):
        if not DEBUG:
            self.fifo.close()
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
            try:
                if not DEBUG:
                    for line in self.fifo:
                        self.package = json.loads(line)
                        self.grabbed = True
                else:
                    ret, img = self.vid.read()
                    saver_frame_counter += 1
                    if saver_frame_counter == self.vid.get(cv2.CAP_PROP_FRAME_COUNT):
                        saver_frame_counter = 0
                        self.vid.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    if ret:
                        _, jpg = cv2.imencode('.jpg', img)
                        self.package = {'frame': jpg, 'yaw': 1.0, 'pitch': 2.0, 'roll': 3.0}
                        self.grabbed = True

            except Exception as e:
                print('Done!')
                break
        if not DEBUG:
            self.fifo.close()
        else:
            self.vid.release()


slam_reader = SLAMReader()
app = Flask(__name__, static_url_path='', static_folder='static', template_folder='templates')


def gen_frames():
    while True:
        frame = slam_reader.get_frame()  # read the camera frame
        if slam_reader.grabbed:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result


def start_logging():
    global slam_reader
    global actual_app_state
    time.sleep(0.5)
    os.system('./SLAM_logger_start.sh &')
    time.sleep(10)
    slam_reader = SLAMReader()
    actual_app_state = 'Logging is started...'


def stop_logging():
    global log_proc
    log_proc.join()
    pid = get_pid('SLAM_Logger')
    os.kill(pid, signal.SIGINT)


@app.route("/utils.js")
def send_js():
    return send_from_directory('static', 'utils.js')


@app.route("/main.css")
def send_css():
    return send_from_directory('static', 'main.css')


@app.route("/", methods=['GET', 'POST'])
def index():
    global actual_app_state
    global log_proc
    global slam_reader
    if request.method == 'POST':
        if request.form.get('Start Logging') == 'Start Logging':
            slam_reader.stop()
            log_proc = threading.Thread(target=start_logging)
            log_proc.start()
            actual_app_state = "Logging starting...Wait..."
        elif request.form.get('Stop Logging') == 'Stop Logging':
            stop_logging()
            actual_app_state = "Preview"
        else:
            actual_app_state = "Unknown"
    elif request.method == 'GET':
        actual_app_state = "Preview"
    return render_template("main.html", actual_app_state=actual_app_state)


@app.route('/stuff', methods=['GET'])
def stuff():
    global actual_app_state
    current_time = time.strftime("%H:%M:%S\n")
    return jsonify(timer_value=current_time, actual_app_state=actual_app_state)


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream_data')
def stream_data():
    data = slam_reader.get_data()
    if data is None:
        yaw = ''
        pitch = ''
        roll = ''
    else:
        yaw = '{:.2}deg'.format(data['yaw'])
        pitch = '{:.2}deg'.format(data['pitch'])
        roll = '{:.2}deg'.format(data['roll'])
    return jsonify(yaw_value=yaw, pitch_value=pitch, roll_value=roll)


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=8000, threaded=True)  #, ssl_context=context)
    slam_reader.stop()
