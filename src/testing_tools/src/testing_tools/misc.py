import subprocess
import re
import os
import logging
import time
import rospy
import rospkg
import rosnode
from rospy.names import canonicalize_name
import cv2
from cv_bridge import CvBridge
import rosbag
import socket
from rostopic import ROSTopicIOException
import rosgraph
from threading import Thread
import tempfile
import signal
import shutil
from Queue import Queue

logger = logging.getLogger('testing_tools')

CWD = os.path.abspath(os.path.dirname(__file__))

__all__ = [
    'run_shell_cmd', 'get_catkin_path', 'wait_for', 'wait_for_master',
    'wait_for_master_gone', 'capture_webcam_video', 'video2rosbag',
    'play_rosbag', 'rosbag_msg_generator', 'wait_for_message',
    'has_subscriber', 'wait_for_subscriber', 'wait_for_messages',
    'ThreadWorker', 'create_msg_listener', 'capture_screen', 'capture_camera',
    'startxvfb', 'stopxvfb', 'get_rosbag_file', 'get_data_path',
    'add_text_to_video', 'concatenate_videos', 'MessageQueue',
    'PololuSerialReader'
    ]

def run_shell_cmd(cmd, first=False):
    """Execute `cmd` in shell and get the outputs.
    If `first` is True, then get only the first line of outputs"""

    stdout, stderr = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        cwd=CWD).communicate()
    if stderr:
        raise Exception(stderr)
    elif stdout:
        if first:
            return stdout.splitlines()[0]
        else:
            return stdout.splitlines()

def get_catkin_path(cat):
    cmd = r"catkin config|grep '%s'|awk '{print $4}'" % cat
    path = run_shell_cmd(cmd, True)
    if os.path.isdir(path):
        return path


class capture_screen():

    def __init__(self, filename, duration):
        self.filename = filename
        if not self.filename.endswith('.avi'):
            self.filename = '%s.avi' % self.filename
        dirname = os.path.dirname(self.filename) or '.'
        if not os.path.isdir(dirname):
            os.makedirs(dirname)
        self.duration = duration

    @staticmethod
    def capture(filename, duration):
        capture_script = os.path.join(
            CWD, 'capture.sh')
        proc = subprocess.Popen(
            [capture_script, filename, str(duration)])
        return proc

    def __enter__(self):
        self.proc = self.capture(self.filename, self.duration)
        return self.proc

    def __exit__(self, type, value, traceback):
        self.proc.wait()

class capture_camera():

    def __init__(self, filename, duration):
        self.filename = filename
        if not self.filename.endswith('.avi'):
            self.filename = '%s.avi' % self.filename
        dirname = os.path.dirname(self.filename) or '.'
        if not os.path.isdir(dirname):
            os.makedirs(dirname)
        self.duration = duration

    def __enter__(self):
        self.job = ThreadWorker(
            target=capture_webcam_video, args=(self.filename, self.duration))
        self.job.start()
        return self.job

    def __exit__(self, type, value, traceback):
        self.job.join()

def wait_for(node, namespace=None):
    node = canonicalize_name(node)
    def is_node_up(node):
        return any([node in upnode for upnode in
                    rosnode.get_node_names(namespace)])
    if not is_node_up(node):
        while not is_node_up(node):
            time.sleep(0.1)

from roslaunch import rlutil
def wait_for_master():
    rlutil._wait_for_master()

def wait_for_master_gone():
    def is_running():
        cmd = r"ps -ef|grep roscore|grep -v grep|grep -v defunct|awk '{print $2}'"
        core = run_shell_cmd(cmd, True)
        return True if core else False
    while is_running():
        time.sleep(0.1)
    time.sleep(3)


def capture_webcam_video(video_filename, sec=None):
    cap = cv2.VideoCapture(0)
    fps = 20

    # Define the codec and create VideoWriter object
    fourcc = cv2.cv.CV_FOURCC(*'XVID')
    out = cv2.VideoWriter(video_filename,fourcc, fps, (640,480))
    stime = time.time()

    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret:
            out.write(frame)

            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if sec and (time.time() - stime) > sec:
                break
        else:
            break

    out.release()
    cap.release()
    cv2.destroyAllWindows()

def video2rosbag(ifile, ofile, topic='/camera/image_raw'):
    cap = cv2.VideoCapture(ifile)
    fps = 20
    wait = 1.0/fps
    bridge = CvBridge()
    bag = rosbag.Bag(ofile, 'w')
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret:
            msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            bag.write(topic, msg)
            time.sleep(wait)
        else:
            break
    cap.release()
    bag.close()

def play_rosbag(argv):
    from rosbag import rosbag_main
    if isinstance(argv, basestring):
        argv = [argv]
    job = Thread(target=rosbag_main.play_cmd, args=(argv,))
    job.start()
    return job

def rosbag_msg_generator(bag_file, topics=None):
    import rosbag
    if not rospy.core.is_initialized():
        rospy.init_node('rosbag_msg_generator', anonymous=True)
    bag = rosbag.Bag(bag_file)
    types_topics_tuple = bag.get_type_and_topic_info()
    if topics is None:
        topics = types_topics_tuple.topics.keys()

    pubs = {}
    for topic, message, timestamp in bag.read_messages(topics=topics, raw=True):
        #yield topic, message, timestamp
        msg_type = message[4]
        data = message[1]
        if topic in pubs:
            pub = pubs[topic]
        else:
            pub = rospy.Publisher(topic, msg_type, latch=True, queue_size=30)
            pubs[topic] = pub
        msg = msg_type()
        msg.deserialize(data)
        pub.publish(msg)
        yield topic, msg
    bag.close()

def wait_for_message(topic, topic_class, timeout):
    msg = None
    try:
        msg = rospy.wait_for_message(topic, topic_class, timeout)
    except rospy.exceptions.ROSException as e:
        rospy.loginfo(e)
        msg = None
    return msg


class ThreadWorker(Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        Thread.__init__(
            self, group, target, name, args, kwargs, verbose)
        self.retval = None

    def run(self):
        try:
            if self._Thread__target:
                self.retval = self._Thread__target(
                    *self._Thread__args, **self._Thread__kwargs)
        finally:
            del self._Thread__target, self._Thread__args, self._Thread__kwargs

    def join(self, timeout=None):
        super(ThreadWorker, self).join(timeout)
        return self.retval

def wait_for_messages(topics, topic_classes, timeout):
    jobs = [ThreadWorker(target=wait_for_message,
                        args=(topic, topic_class, timeout))
            for topic, topic_class in zip(topics, topic_classes)]
    [job.start() for job in jobs]
    retvalues = [job.join() for job in jobs]
    return retvalues

def has_subscriber(topic, sub):
    master = rosgraph.Master('/rostopic')
    try:
        pubs, subs, srvs = master.getSystemState()
        for x in subs:
            if x[0] == topic:
                return sub in x[1]
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")
        return False
    return False

def wait_for_subscriber(topic, sub, timeout=None):
    if timeout is not None:
        time_t = time.time() + timeout
        while not has_subscriber(topic, sub):
            if time.time() >= time_t:
                break
            time.sleep(0.01)
    else:
        while not has_subscriber(topic, sub):
            time.sleep(0.01)

def create_msg_listener(topic, topic_class, timeout):
    return ThreadWorker(target=wait_for_message,
                        args=(topic, topic_class, timeout))

def get_xvfb_pid_file(display):
    return os.path.join(
        tempfile.gettempdir(), 'xvfb_%s.pid' % display.replace(':', ''))


def startxvfb(display, res='1024x768x24'):
    proc = subprocess.Popen(
        ['Xvfb', display, '-screen', '0', res, '-ac'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    pid_file = get_xvfb_pid_file(str(display))
    with open(pid_file, 'w') as f:
        f.write(str(proc.pid))


def stopxvfb(display):
    pid_file = get_xvfb_pid_file(str(display))
    if os.path.isfile(pid_file):
        with open(pid_file) as f:
            pid = int(f.read())
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                logger.info("Killing Xvfb process failed")
        os.remove(pid_file)

def get_data_path():
    rospack = rospkg.RosPack()
    return os.path.join(rospack.get_path('testing_tools'), 'data')

def get_rosbag_file(rosbag_name):
    filename = os.path.join(get_data_path(), rosbag_name)
    if os.path.isfile('%s.bag' % filename):
        return '%s.bag' % filename
    elif os.path.isfile('%s.avi' % filename):
        video2rosbag('%s.avi' % filename, '%s.bag' % filename)
        return '%s.bag' % filename

def backup(filename):
    backup_filename = ''.join([os.path.splitext(filename)[0],
                '.orig', os.path.splitext(filename)[1]])
    shutil.copy2(filename, backup_filename)

def add_text_to_video(filename, text=None):
    if text is None:
        text = os.path.splitext(os.path.basename(filename))[0]
    tmp_file = '/tmp/tmp.avi'
    cmd = "ffmpeg -y -i %s -vf drawtext=\"text=\'%s\': "\
        "fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf: "\
        "fontcolor=white: fontsize=28\" %s" % (filename, text, tmp_file)
    try:
        os.system(cmd)
        shutil.move(tmp_file, filename)
    finally:
        if os.path.isfile(tmp_file):
            os.remove(tmp_file)


def avi2mpg(filename):
    assert filename.endswith('.avi')
    ofile = '%s.mpg' % os.path.splitext(filename)[0]
    os.system('ffmpeg -y -i %s -qscale:v 1 %s' % (filename, ofile))
    return ofile

def mpg2avi(filename):
    assert filename.endswith('.mpg')
    ofile = '%s.avi' % os.path.splitext(filename)[0]
    os.system('ffmpeg -y -i %s -qscale:v 2 %s' % (filename, ofile))
    return ofile

def concatenate_videos(filenames, ofile, delete):
    tmp_file = '/tmp/all.mpg'
    ofiles = [avi2mpg(f) for f in filenames if os.path.isfile(f)]
    ofiles = [f for f in ofiles if f is not None]
    os.system('cat %s > %s' % (' '.join(ofiles), tmp_file))
    mpg2avi(tmp_file)
    shutil.move('/tmp/all.avi', ofile)
    [os.remove(f) for f in ofiles]
    os.remove(tmp_file)
    if delete:
        [os.remove(f) for f in filenames]

class MessageQueue():
    def __init__(self):
        self.queue = Queue()

    def _cb(self, msg):
        if msg is not None:
            self.queue.put(msg)

    def subscribe(self, topic, topic_class):
        return rospy.Subscriber(topic, topic_class, self._cb)

    def get(self):
        return self.queue.get()


class PololuSerialReader(object):
    CMD_DICT = {'135': 'speed', '137': 'accelaration', '132': 'position'}

    def __init__(self, device):
        import serial
        self.ser = serial.Serial(device, baudrate=115200,
                                bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                timeout=5, writeTimeout=None)

    def read(self):
        try:
            num = self.ser.read(size=4)
            id = ord(num[1])
            cmd = self.CMD_DICT[str(ord(num[0]))]
            value = (ord(num[3])<<7) + ord(num[2])
        except serial.SerialException as e:
            raise e
        except TypeError as e:
            id, cmd, value = 0, '', 0
        return (id, cmd, value)


if __name__ == '__main__':
    name = 'face_in'
    capture_webcam_video('%s.avi' % name, 5)
    video2rosbag('%s.avi' % name, '%s.bag' % name)

