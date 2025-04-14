import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals
import time
import math
import serial
from threading import Lock
from std_msgs.msg import Int32
from rclpy.executors import MultiThreadedExecutor



class MotorDriver(Node):

    angle = 0

    def __init__(self):
        super().__init__('motor_driver')
        self.angle = 0
        

        self.servo_sub = self.create_subscription(
            Int32,
            'servo_angle',
            self.servo_callback,
            10
        )


        # Setup parameters

        self.declare_parameter('encoder_cpr', value=0)
        if (self.get_parameter('encoder_cpr').value == 0):
            print("WARNING! ENCODER CPR SET TO 0!!")


        self.declare_parameter('loop_rate', value=0)
        if (self.get_parameter('loop_rate').value == 0):
            print("WARNING! LOOP RATE SET TO 0!!")


        self.declare_parameter('serial_port', value="/dev/ttyACM0")
        self.serial_port = self.get_parameter('serial_port').value


        self.declare_parameter('baud_rate', value=9600)
        self.baud_rate = self.get_parameter('baud_rate').value


        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if self.debug_serial_cmds:
            print("Serial debug enabled")



        # Setup topics & services

        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

        self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)

        self.encoder_pub = self.create_publisher(EncoderVals, 'encoder_vals', 10)
        

        # Member Variables

        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.m1_spd = 0.0
        self.m2_spd = 0.0

        self.mutex = Lock()


        # Open serial comms

        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")
        

        


    # Raw serial commands
    
    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        self.send_command(f"o {int(mot_1_pwm)} {int(mot_2_pwm)}")

    def send_feedback_motor_command(self, mot_1_ct_per_loop, mot_2_ct_per_loop):
        self.send_command(f"m {int(mot_1_ct_per_loop)} {int(mot_2_ct_per_loop)}")

    def send_encoder_read_command(self):
        resp = self.send_command("e")
        if resp:
            print(f"Raw response from Arduino: {resp}")
            # Vérifier si la réponse contient uniquement des nombres
            if all(part.isdigit() for part in resp.split()):
                return [int(raw_enc) for raw_enc in resp.split()]
            else:
                self.get_logger().warn(f"Ignored invalid response: {resp}")
        return []


    # More user-friendly functions

    def motor_command_callback(self, motor_command):
        if (motor_command.is_pwm):
            self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        else:
            # counts per loop = req rads/sec X revs/rad X counts/rev X secs/loop 
            scaler = (1 / (2*math.pi)) * self.get_parameter('encoder_cpr').value * (1 / self.get_parameter('loop_rate').value)
            mot1_ct_per_loop = motor_command.mot_1_req_rad_sec * scaler
            mot2_ct_per_loop = motor_command.mot_2_req_rad_sec * scaler
            self.send_feedback_motor_command(mot1_ct_per_loop, mot2_ct_per_loop)

    def check_encoders(self):
        resp = self.send_encoder_read_command()
        if (resp):

            new_time = time.time()
            time_diff = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            m1_diff = resp[0] - self.last_m1_enc
            self.last_m1_enc = resp[0]
            m2_diff = resp[1] - self.last_m2_enc
            self.last_m2_enc = resp[1]

            rads_per_ct = 2*math.pi/self.get_parameter('encoder_cpr').value
            self.m1_spd = m1_diff*rads_per_ct/time_diff
            self.m2_spd = m2_diff*rads_per_ct/time_diff

            spd_msg = MotorVels()
            spd_msg.mot_1_rad_sec = self.m1_spd
            spd_msg.mot_2_rad_sec = self.m2_spd
            self.speed_pub.publish(spd_msg)

            enc_msg = EncoderVals()
            enc_msg.mot_1_enc_val = self.last_m1_enc
            enc_msg.mot_2_enc_val = self.last_m2_enc
            self.encoder_pub.publish(enc_msg)

    # Utility functions

    def send_command(self, cmd_string):
        
        self.mutex.acquire()
        
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))

            ## Adapted from original
            c = ''
            value = ''
            while c != '\r':
                c = self.conn.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\r')

            if (self.debug_serial_cmds):
                print("Received: " + value)
            return value
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()

    
    def send_servo_command(self, servo_index, angle):
        """Envoie une commande pour contrôler le servo."""
        if 0 <= angle <= 180:
            command = f"s {servo_index} {angle}"
            self.send_command(command)
            self.get_logger().info(f"Sent servo command: {command}")
        else:
            self.get_logger().warn("Invalid angle. Use a value between 0 and 180.")

    def servo_callback(self, msg):
        """callback pour déplacer le servo en fonction de l'angle reçu."""
        angle = msg.data
        self.send_servo_command(0, angle)  # envoi commande principale

        # forcer une mise à jour en envoyant une petite variation
        time.sleep(0.1)  # court délai
        self.send_servo_command(0, angle + 1)  # envoyer un angle légèrement différent
        time.sleep(0.1)
        self.send_servo_command(0, angle)

    def servo_keyboard(self):
        led = input('message: ')
        if led =="i" and self.angle > 0 :
            print("BACKWARD")
            self.angle -= 10
        if led =="k" and self.angle < 180:
            print("FORWARD")
            self.angle += 10

        self.send_servo_command(self.angle,0)  # envoi commande principale

        # forcer une mise à jour en envoyant une petite variation
        time.sleep(0.1)  
        self.send_servo_command(self.angle + 1, 0)  
        time.sleep(0.1)
        self.send_servo_command(self.angle, 0)



def main(args=None):
    rclpy.init(args=args)

    # Initialiser le nœud MotorDriver
    motor_driver = MotorDriver()

    # Utiliser un exécuteur multithread pour gérer les callbacks et les tâches simultanément
   # executor = MultiThreadedExecutor()
   # executor.add_node(motor_driver)

    try:
        # Lancer l'exécuteur en continu
        while rclpy.ok():
            rclpy.spin_once(motor_driver, timeout_sec=0.01)
            motor_driver.servo_keyboard()
            motor_driver.check_encoders()
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver.close_conn()
        motor_driver.destroy_node()
        rclpy.shutdown()
