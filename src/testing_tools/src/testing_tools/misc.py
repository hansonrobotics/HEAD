import subprocess
import os
import logging
import time
import rospy
import rosnode
import cv2
import rosbag
from threading import Thread
import signal
import shutil
from Queue import Queue
import serial

logger = logging.getLogger('testing_tools.misc')

CWD = os.path.abspath(os.path.dirname(__file__))

def run_shell_cmd(cmd, first=False):
    """
    Execute command in shell and get the outputs.

    @param cmd: command that is going to run in shell
    @type  cmd: str

    @param first: If it's True, return only the first line of outputs.
    @type  first: bool
    """

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

class capture_screen():
    """
    Capture cast screen and save to .avi file.

    Example
    -------
    >>> with capture_screen(filename.avi, duration):
    >>>     do something
    """

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
            [capture_script, filename, str(duration)], stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        return proc

    def __enter__(self):
        self.proc = self.capture(self.filename, self.duration)
        return self.proc

    def __exit__(self, type, value, traceback):
        self.proc.wait()

class capture_camera():
    """
    Capture webcam video and save to file.

    Example
    -------
    >>> with capture_camera(filename.avi, duration):
    >>>     do something

    """

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
    """
    Wait for ROS node up.

    @param node: node name
    @type  node: str
    """

    from rospy.names import canonicalize_name
    node = canonicalize_name(node)
    def is_node_up(node):
        try:
            node_up = any([node in upnode for upnode in
                        rosnode.get_node_names(namespace)])
            return node_up
        except Exception:
            return False
    if not is_node_up(node):
        while not is_node_up(node):
            time.sleep(0.1)

def capture_webcam_video(video_filename, sec=None):
    """
    Use OpenCV to capture video from webcam and save the video to file

    @param video_filename: video file name
    @type  video_filename: str

    @param sec: video duration in seconds
    @type  sec: double
    """

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
    """
    Read video file and convert to rosbag file.
    The default ROS topic is /camera/image_raw.

    @param ifile: Video file name
    @type  ifile: str

    @param ofile: ROS bag file name
    @type  ofile: str
    """

    from cv_bridge import CvBridge
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
    """
    New a thread and play rosbag.

    Example
    -------
    Play rosbag foo.bag
    >>> job = play_rosbag(['foo.bag'])
    >>> do something ...
    >>> job.join()

    @param argv: options and arguments that `rosbag play` supports
    @type  argv: [str]
    """

    from rosbag import rosbag_main
    if isinstance(argv, basestring):
        argv = [argv]
    job = Thread(target=rosbag_main.play_cmd, args=(argv,))
    job.start()
    return job

def rosbag_msg_generator(bag_file, topics=None):
    """
    Generate ROS message based on rosbag file.
    It sends one ROS message each time.

    For example, given rosbag file foo.bag
        >>> for topic, msg, timestamp in rosbag_msg_generator(foo.bag):
        >>>     do something

    @param bag_file: rosbag file name
    @type  bag_file: str

    @param topics: topics included in the bag file that you want to send
    @type  topics: [str]
    """

    import rosbag
    from datetime import datetime
    if not rospy.core.is_initialized():
        rospy.init_node('rosbag_msg_generator', anonymous=True)
    bag = rosbag.Bag(bag_file)
    types_topics_tuple = bag.get_type_and_topic_info()
    if topics is None:
        topics = types_topics_tuple.topics.keys()

    pubs = {}
    for topic, message, timestamp in bag.read_messages(topics=topics, raw=True):
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
        timestamp = datetime.fromtimestamp(timestamp.to_sec())
        yield topic, msg, timestamp
    bag.close()

def wait_for_message(topic, topic_class, timeout):
    """
    Wait a given seconds for a specific message

    @param timeout: timeout time in seconds
    @type  timeout: double
    """
    msg = None
    try:
        msg = rospy.wait_for_message(topic, topic_class, timeout)
    except rospy.exceptions.ROSException as e:
        rospy.loginfo(e)
        msg = None
    return msg


class ThreadWorker(Thread):
    """
    Thread with return values.

    Example
    -------
    >>> job = ThreadWorker(target=foo)
    >>> ret = job.join()
    """
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
    """
    Get the messages from specific topics.

    @param topics: topics that it subscribes
    @type  topics: [str]

    @param topic_classes: corresponding topic classes
    @type  topic_classes: [class]

    @param timeout: timeout time in seconds
    @type  timeout: double
    """
    jobs = [ThreadWorker(target=wait_for_message,
                        args=(topic, topic_class, timeout))
            for topic, topic_class in zip(topics, topic_classes)]
    [job.start() for job in jobs]
    retvalues = [job.join() for job in jobs]
    return retvalues

def get_xvfb_pid_file(display):
    """
    Make a temporary xvfb pid file

    @param display: display identifier
    @type  display: str
    """

    import tempfile
    return os.path.join(
        tempfile.gettempdir(), 'xvfb_%s.pid' % display.replace(':', ''))

def startxvfb(display, res='1024x768x24'):
    """
    Start Xvfb with the specific display identifier

    @param display: display identifier
    @type  display: str

    @param res: screen resolution
    @type  res: str
    """

    proc = subprocess.Popen(
        ['Xvfb', display, '-screen', '0', res, '-ac'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    pid_file = get_xvfb_pid_file(str(display))
    with open(pid_file, 'w') as f:
        f.write(str(proc.pid))


def stopxvfb(display):
    """
    Stop Xvfb with the specific display identifier

    @param display: display identifier
    @type  display: str
    """

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
    """
    Return the absolute test data path.
    """

    import rospkg
    rospack = rospkg.RosPack()
    return os.path.join(rospack.get_path('testing_tools'), 'data')

def get_rosbag_file(rosbag_name):
    """
    Find and return the rosbag file path in test data directory. If the
    ros bag file is found, returns the path of the file. Otherwise trys
    to file the video file that matches the file name, and convert it to
    ros bag file, and then return the path of this ros bag file.

    @param rosbag_name: file name of the rosbag without extension
    @type  rosbag_name: str
    """

    filename = os.path.join(get_data_path(), rosbag_name)
    if os.path.isfile('%s.bag' % filename):
        return '%s.bag' % filename
    elif os.path.isfile('%s.avi' % filename):
        video2rosbag('%s.avi' % filename, '%s.bag' % filename)
        return '%s.bag' % filename

def add_text_to_video(filename, text=None):
    """
    Add text to video

    @param filename: video file name
    @type  filename: str

    @param text: text to be added
    @type  text: str
    """
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
    """
    Convert avi file to mpg file

    @param filename: avi video file name
    @type  filename: str
    """
    assert filename.endswith('.avi')
    ofile = '%s.mpg' % os.path.splitext(filename)[0]
    os.system('ffmpeg -y -i %s -qscale:v 1 %s' % (filename, ofile))
    return ofile

def mpg2avi(filename):
    """
    Convert mgp file to avi file

    @param filename: mpg video file name
    @type  filename: str
    """
    assert filename.endswith('.mpg')
    ofile = '%s.avi' % os.path.splitext(filename)[0]
    os.system('ffmpeg -y -i %s -qscale:v 2 %s' % (filename, ofile))
    return ofile

def concatenate_videos(filenames, ofile, delete):
    """
    Concatenate videos.

    @param filenames: list of file names
    @type  filenames: [str]

    @param ofile: output file name
    @type  ofile: str

    @param delete: whether or not to delete the input files
    @type  delete: bool
    """
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
    """
    MessageQueue is a class that listens to the topic it subscribes and
    collect the ROS message from those topics.

    Example
    -------
    >>> queue = MessageQueue()
    >>> queue.subscribe(foo_topic, topic_class)
    >>> msg = queue.get()
    """

    def __init__(self):
        self.queue = Queue()

    def _cb(self, msg):
        if msg is not None:
            self.queue.put(msg)

    def subscribe(self, topic, topic_class):
        return rospy.Subscriber(topic, topic_class, self._cb)

    def get(self, timeout=None):
        return self.queue.get(timeout=timeout)

    def clear(self):
        while not self.queue.empty():
            self.queue.get(1)

    def tolist(self):
        data = []
        while not self.queue.empty():
            data.append(self.queue.get(1))
        return data


class PololuSerialReader(object):
    """
    Read specified serial port, parse the data it reads and return
    Pololu command.

    Example
    -------
    >>> reader = PololuSerialReader('/dev/tty0')
    >>> motor_id, cmd, value = reader.read()
    """

    CMD_DICT = {'135': 'speed', '137': 'accelaration', '132': 'position'}

    def __init__(self, device, timeout=5):
        self.ser = serial.Serial(device, baudrate=115200,
                                bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                timeout=timeout, writeTimeout=None)

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


def check_if_ffmpeg_satisfied():
    """
    Check if ffmpeg is installed and satisfied for screencasting.
    """

    try:
        configuration = run_shell_cmd('ffmpeg -version|grep configuration', True)
    except Exception:
        return False
    configuration = [i.strip() for i in configuration.split(':')[1].split('--')]
    requires = ['enable-libx264', 'enable-libfreetype']
    return all([i in configuration for i in requires])

def check_if_sound_card_exists():
    """
    Check if sound card exists.
    """

    try:
        snd_cards = run_shell_cmd('cat /proc/asound/cards')
    except Exception:
        return False
    return not 'no soundcards' in '\n'.join(snd_cards)

class SerialPortRecorder(object):
    """
    Record data transmitted to the serial port device
    """

    def __init__(self, device, ofile=None):
        """
        @param devide: serial port device
        @type  device: str
        @param ofile: output file (optional)
        @type  ofile: str
        """
        self.ser = serial.Serial(
            device, baudrate=115200, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=None, writeTimeout=None)
        self.data = []
        self.running = False
        self.job = None
        self.ofile = ofile
        signal.signal(signal.SIGINT, self._interrupt)
        self.start_time = None
        self.stop_time = None

    def start(self):
        self.job = Thread(target=self._record)
        self.job.daemon = True
        self.running = True
        self.job.start()

    def _record(self):
        self.running = True
        while self.running:
            try:
                num = self.ser.read(1)
            except serial.SerialException as e:
                self.stop()
                raise e
            self.data.append(num)
            if self.start_time is None and len(self.data) == 1:
                self.start_time = time.time()

    def _interrupt(self, signum, frame):
        self.stop()

    def stop(self):
        self.running = False
        if self.stop_time is None:
            self.stop_time = time.time()
        self.ser.close()
        if self.ofile is not None:
            with open(self.ofile, 'w') as f:
                for i in self.data:
                    f.write(i)
            print "write to file %s" % self.ofile

    def __repr__(self):
        return "<SerialPortRecorder start_time:%s, stop_time:%s, data:%s>" % (
            self.start_time, self.stop_time, ''.join(self.data[:80]))
