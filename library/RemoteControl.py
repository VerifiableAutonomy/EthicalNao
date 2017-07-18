from Tkinter import *

sys.path.append('pynaoqi-python2.7-2.1.2.17-linux64')
from naoqi import ALProxy
import vision_definitions
import Utilities
import time
from PIL import Image, ImageTk


class NaoController:
    def __init__(self, ip_address):
        self.logger = Utilities.Logger('NaoController.log')
        self.snapshot = None
        self.loop_counter = 0
        self.current_subscriber = 0

        self.robot_ip = ip_address

        self.root = Tk()
        self.root.title("Nao Controller")

        self.forward = Button(self.root, text='Foward')
        self.back = Button(self.root, text='Backward')
        self.R_left = Button(self.root, text='R Left')
        self.R_right = Button(self.root, text='R Right')
        self.G_left = Button(self.root, text='Left')
        self.G_right = Button(self.root, text='Right')
        self.talk = Button(self.root, text='Speak')
        self.battery_level = Label(self.root)

        self.image = Canvas(self.root, width=160, height=120)
        self.image.config(bg='red')

        self.text = Text(self.root)

        button_stick = S + E + N + W

        self.image.grid(row=0, column=0, columnspan=1, rowspan=4)

        self.text.grid(row=0, column=1, columnspan=3, rowspan=1)

        self.forward.grid(row=1, column=2, sticky=button_stick)
        self.back.grid(row=3, column=2, sticky=button_stick)
        self.R_left.grid(row=2, column=1, sticky=button_stick)
        self.R_right.grid(row=2, column=3, sticky=button_stick)
        self.G_left.grid(row=1, column=1, sticky=button_stick)
        self.G_right.grid(row=1, column=3, sticky=button_stick)
        self.talk.grid(row=2, column=2, sticky=button_stick)
        self.battery_level.grid(row=3, column=3, sticky=button_stick)

        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        self.root.grid_rowconfigure(1, minsize=100)
        self.root.grid_rowconfigure(2, minsize=100)
        self.root.grid_rowconfigure(3, minsize=100)

        self.root.grid_columnconfigure(1, minsize=100)
        self.root.grid_columnconfigure(2, minsize=100)
        self.root.grid_columnconfigure(3, minsize=100)

        self.logger.write('CONNECTING TO ROBOT')
        self.motion = ALProxy("ALMotion", ip_address, 9559)
        self.motion.setStiffnesses("Body", 1.0)
        self.voice = ALProxy("ALAnimatedSpeech", ip_address, 9559)
        self.battery = ALProxy("ALBattery", ip_address, 9559)
        self.life = ALProxy("ALAutonomousLife", ip_address, 9559)
        self.awareness = ALProxy("ALBasicAwareness", ip_address, 9559)
        self.posture = ALProxy("ALRobotPosture", ip_address, 9559)

        self.logger.write('CONNECTING TO CAMERA')
        self.camera = ALProxy("ALVideoDevice", ip_address, 9559)
        subscribers = self.camera.getSubscribers()
        for subscriber in subscribers:
            if subscriber.startswith('python_GVM'):
                self.logger.write('Unsubscribing ' + subscriber)
                self.camera.unsubscribe(subscriber)
        resolution = vision_definitions.kQQQVGA
        color_space = vision_definitions.kYuvColorSpace

        frame_rate = 50
        self.camera.subscribe("python_GVM", resolution, color_space, frame_rate)
        self.camera.subscribe("python_GVM", resolution, color_space, frame_rate)
        self.camera.subscribe("python_GVM", resolution, color_space, frame_rate)

        self.logger.write('SWTICH OF AUTO LIFE')
        self.life.setState('solitary')
        self.awareness.stopAwareness()

        ## Start getting image and head normalization
        self.set_canvas_image()
        self.set_battery_level()

        ## BUTTON BINDINGS GO HERE
        self.forward.bind("<Button-1>", self.move_forward)
        self.forward.bind("<ButtonRelease-1>", self.stop_robot)

        self.back.bind("<Button-1>", self.move_back)
        self.back.bind("<ButtonRelease-1>", self.stop_robot)

        self.R_left.bind("<Button-1>", self.rotate_left)
        self.R_left.bind("<ButtonRelease-1>", self.stop_robot)

        self.R_right.bind("<Button-1>", self.rotate_right)
        self.R_right.bind("<ButtonRelease-1>", self.stop_robot)

        self.G_left.bind("<Button-1>", self.go_left)
        self.G_left.bind("<ButtonRelease-1>", self.stop_robot)

        self.G_right.bind("<Button-1>", self.go_right)
        self.G_right.bind("<ButtonRelease-1>", self.stop_robot)

        self.talk.bind("<Button-1>", self.speak_text)

        self.set_window_size(1000, 400)
        self.image.config(width=399, height=399)

        self.root.mainloop()

    def set_battery_level(self):
        charge = self.battery.getBatteryCharge()
        colour = 'green'
        if charge < 50: colour = 'orange'
        if charge < 20: colour = 'red'
        self.battery_level.config(fg=colour, text=charge, font=("Helvetica", 40))
        self.root.after(5000, self.set_battery_level)

    def set_window_size(self, width, height):
        self.root.geometry('{}x{}'.format(width, height))

    def get_snapshot(self):
        start = time.time()
        if self.current_subscriber > 2: self.current_subscriber = 0
        if self.current_subscriber == 0: id = 'python_GVM_0'
        if self.current_subscriber == 1: id = 'python_GVM_1'
        if self.current_subscriber == 2: id = 'python_GVM_2'
        nao_image = self.camera.getImageRemote(id)
        stop = time.time()
        x = nao_image[0]
        y = nao_image[1]
        array = nao_image[6]
        # snapshot = Image.frombytes("RGB", (x, y), array)
        snapshot = Image.frombytes("L", (x, y), array)
        self.current_subscriber += 1
        delay = stop - start
        # print delay
        return snapshot

    def set_canvas_image(self):
        # get snapshot
        snapshot = self.get_snapshot()
        # rescale
        size = snapshot.size
        scale = 5
        new_width = size[0] * scale
        new_height = size[0] * scale
        snapshot = snapshot.resize((new_width, new_height))
        offset_x = int(new_width / 2.0)
        offset_y = int(new_height / 2.0)
        # put on canvas
        self.snapshot = ImageTk.PhotoImage(snapshot)
        self.image.create_image(offset_x, offset_y, image=self.snapshot)
        delay = int(1000 / 25)
        self.root.after(delay, self.set_canvas_image)

    def move_forward(self, event):
        self.motion.post.moveTo(10, 0, 0)

    def move_back(self, event):
        self.motion.post.moveTo(-10, 0, 0)

    def rotate_left(self, event):
        self.motion.post.moveTo(0, 0, +90)

    def rotate_right(self, event):
        self.motion.post.moveTo(0, 0, -90)

    def go_right(self, event):
        self.motion.post.moveTo(0, -10, 0)

    def go_left(self, event):
        self.motion.post.moveTo(0, 10, 0)

    def stop_robot(self, event):
        self.motion.stopMove()

    def speak_text(self, event):
        text = self.text.get("1.0", 'end-1c')
        text = text.replace('\n', ' ')
        text = text.replace('\r', ' ')
        text = str(text)
        motion_configuration = {"bodyLanguageMode": "contextual"}
        self.voice.say(text, motion_configuration)


ip_address = '192.168.20.223'
NaoController(ip_address)