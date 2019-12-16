import socket
import threading
import time
import numpy as np
# import libh264decoder
import signal
from . import logger
from . import event
from . import dispatcher
from . utils import *
from . protocol import *

log = logger.Logger('Tello')
log.info("tellopy - Arkady's Version")

class TelloSDK:

    """Wrapper class to interact with the Tello drone."""
    EVENT_CONNECTED = event.Event('connected')
    EVENT_WIFI = event.Event('wifi')
    EVENT_LIGHT = event.Event('light')
    EVENT_FLIGHT_DATA = event.Event('fligt_data')
    EVENT_LOG_HEADER = event.Event('log_header')
    EVENT_LOG = EVENT_LOG_HEADER
    EVENT_LOG_RAWDATA = event.Event('log_rawdata')
    EVENT_LOG_DATA = event.Event('log_data')
    EVENT_LOG_CONFIG = event.Event('log_config')
    EVENT_TIME = event.Event('time')
    EVENT_VIDEO_FRAME = event.Event('video frame')
    EVENT_VIDEO_DATA = event.Event('video data')
    EVENT_DISCONNECTED = event.Event('disconnected')
    EVENT_FILE_RECEIVED = event.Event('file received')


    def __init__(self, local_ip='', local_port=8889, network_interface='', imperial=False, command_timeout=.3, tello_ip='192.168.10.1',
                 tello_port=8889, stream_=True, local_video_port=11111):
        """
        Binds to the local IP/port and puts the Tello into command mode.

        :param local_ip (str): Local IP address to bind.
        :param local_port (int): Local port to bind.
        :param imperial (bool): If True, speed is MPH and distance is feet.
                             If False, speed is KPH and distance is meters.
        :param command_timeout (int|float): Number of seconds to wait for a response to a command.
        :param tello_ip (str): Tello IP.
        :param tello_port (int): Tello port.
        """
   


        self.abort_flag = False
        # self.decoder = libh264decoder.H264Decoder()
        self.command_timeout = command_timeout
        self.imperial = imperial
        self.response = None  
        self.frame = None  # numpy array BGR -- current camera output frame
        self.is_freeze = False  # freeze current camera output
        self.last_frame = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd

        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for receiving video stream
        self.tello_address = (tello_ip, tello_port)
        self.local_video_port = local_video_port
        # self.local_video_port = 11111  # port for receiving video stream
        # self.local_video_port = 6038

        self.connected_once = False

        self.tello_addr = (tello_ip, tello_port)

        self.last_height = 0
        self.quit_flag = False
        self.local_ip = local_ip
        self.local_port = local_port
        self.socket.bind((self.local_ip, self.local_port))
        self.socket.settimeout(2)

        self.socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for receiving state
        self.state_port = 8890
        self.state_ip = ''
        self.socket_state.bind((self.state_ip, self.state_port))
        self.socket_state.settimeout(2)

        self.flight_data = FlightData([])

        self.state_int_func = [float]*6 + [int]*5 + [float]*5

        self.high_speed_mode = False

        log.info('local_ip={}, local_port={}, tello_ip={}, tello_port={}'.format(local_ip, local_port, tello_ip, tello_port))
        log.info('self.socket sends to {}'.format(self.tello_address))
        log.info('self.socket receives from {}'.format((self.local_ip, self.local_port)))
        log.info('self.joystick_socket sends to {}'.format((self.tello_addr)))
        log.info('self.socket_state receives from {}'.format((self.state_ip, self.state_port)))
        log.info('self.socket_video receives from {}'.format((self.local_ip, self.local_video_port)))  

        signal.signal(signal.SIGINT, self.quit)
        signal.signal(signal.SIGTERM, self.quit)


        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0

        dispatcher.connect(self.__state_machine, dispatcher.signal.All)


        

        # to receive video -- send cmd: command, streamon
        # self.socket.sendto(b'command', self.tello_address)
        # print ('sent: command')

        self.command_flag = False

        if 'ok' in self.send_command('command'):
            self.command_flag = True
            print('Connected, successfully sent command')

        # thread for receiving cmd ack
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        self.state_thread = threading.Thread(target=self._state_thread)
        self.state_thread.daemon = True
        self.state_thread.start()

        self.joystick_timeout = 1/30.0
        self.joystick_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.joystick_thread = threading.Thread(target=self._send_joystick_thread)
        self.joystick_thread.daemon = True
        self.joystick_thread.start()

        if stream_:
            # self.socket.sendto(b'streamon', self.tello_address)
            self.stream_flag = (self.send_command('streamon') == 'ok')
            # print ('sent: streamon')

            self.socket_video.bind((self.local_ip, self.local_video_port))
            self.socket_video.settimeout(0.5)

            # thread for receiving video
            self.receive_video_thread = threading.Thread(target=self._receive_video_thread)
            self.receive_video_thread.daemon = True

            self.receive_video_thread.start()

    def __state_machine(self, event, sender, data, **args):
        pass

    def quit(self, *args):
        self.__del__()

    def __del__(self):
        """Closes the local socket."""
        self.quit_flag = True
        print('starting quit')
        # self.socket.close()
        # self.socket_video.close()

    def connect(self):
        """Connect is used to send the initial connection request to the drone."""
        self.send_command('command')

    def wait_for_connection(self, timeout=0):
        """Wait_for_connection will block until the connection is established."""
        t = time.time()
        while time.time() - t < timeout:
            if self.send_command('command') == 'ok':
                log.info("Connected!")
                self.connected_once = True
                return
            time.sleep(0.3)
        log.error("Could Not Connect!")

    def record_log_data(self):
        pass

    def start_video(self):
        self.set_stream(True)
    
    def read(self):
        """Return the last frame from camera."""
        if self.is_freeze:
            return self.last_frame
        else:
            return self.frame

    def video_freeze(self, is_freeze=True):
        """Pause video output -- set is_freeze to True"""
        self.is_freeze = is_freeze
        if is_freeze:
            self.last_frame = self.frame

    def subscribe(self, signal, handler):
        """Subscribe a event such as EVENT_CONNECTED, EVENT_FLIGHT_DATA, EVENT_VIDEO_FRAME and so on."""
        dispatcher.connect(handler, signal)

    def __publish(self, event, data=None, **args):
        args.update({'data': data})
        if 'signal' in args:
            del args['signal']
        if 'sender' in args:
            del args['sender']
        log.debug('publish signal=%s, args=%s' % (event, args))
        dispatcher.send(event, sender=self, **args)

    

    def parse_state(self, state_string):
        lines = state_string.strip().replace('\n', '').replace('\r','').split(';')
        state_dict = {}
        # print(state_string)
        for i, line in enumerate(lines):
            if not ':' in line:
                continue
            key, val = line.split(':')
            if val.replace('-','').isdigit():
                state_dict[key] = int(val)
            if val.replace('.', '').replace('-','').isdigit():
                state_dict[key] = float(val)
            else:
                state_dict[key] = val
            # try:
            #     if '.' in val:
            #         state_dict[key] = float(val)  
            #     else:
            #         state_dict[key] = int(val)  
            # except:
            #     state_dict[key] = val
        # print(state_dict)
        self.flight_data.battery_percentage = state_dict['bat']
        self.flight_data.pressure = state_dict['baro']
        self.flight_data.fly_time = state_dict['time']
        self.flight_data.east_speed = state_dict['vgy']
        self.flight_data.north_speed = state_dict['vgx']
        self.flight_data.ground_speed = state_dict['vgz']
        # self.flight_data.height  = '%1.1f' % float(state_dict['h']/10.)
        self.flight_data.height  = float(state_dict['h'])/10.
        self.flight_data.pitch  = state_dict['pitch']
        self.flight_data.roll  = state_dict['roll']
        self.flight_data.yaw  = state_dict['yaw']
        self.flight_data.templ  = state_dict['templ']
        self.flight_data.temph  = state_dict['temph']
        self.flight_data.tof  = state_dict['tof']
        self.flight_data.agx  = state_dict['agx']/100.
        self.flight_data.agy  = state_dict['agy']/100.
        self.flight_data.agy  = state_dict['agz']/100.
        self.__publish(event=self.EVENT_FLIGHT_DATA, data=self.flight_data)

    def _state_thread(self):
        """Listen to state from the Tello.

        Runs as a thread, sets self.response to whatever the Tello last returned.

        """
        try:
            log.info('start state thread')
            while True:
                try:
                    if self.quit_flag:
                        raise KeyboardInterrupt
                    response, ip = self.socket_state.recvfrom(1024)
                    if response == 'ok':
                        continue
                    self.socket.sendto('streamon'.encode('utf-8'), self.tello_address)
                    try:
                        self.parse_state(response)
                    except ValueError as ex:
                        show_exception(ex)

                    # out = response.replace(';', ';\n')
                    # out = 'Tello State:\n' + out
                    # print(out)

                    #print(self.response)
                except socket.error as exc:
                    log.debug("State Thread Caught exception socket.error : %s" % exc)
        except KeyboardInterrupt as ex:
            log.info('exit from the state thread.') 
            # log.error('state thread: %s' % str(ex))
            # show_exception(ex)
            # raise(ex)
        finally:
            self.socket_state.close()
            self.__del__()
              

    def _send_joystick_thread(self):
        try:
            log.info('start joystick thread with timeout of {}'.format(self.joystick_timeout))
            while True:
                try:
                    if self.quit_flag:
                        raise KeyboardInterrupt

                    # self.abort_joystick_flag = False
                    # timer = threading.Timer(self.joystick_timeout, self.set_abort_joystick_flag)

                    # timer.start()
                    # while self.set_abort_joystick_flag is False:
                        # pass
                    # timer.cancel()

                    time.sleep(self.joystick_timeout)
                    self.__send_stick_command()
                    # self.send_command('command')
                    self.socket.sendto('command'.encode('utf-8'), self.tello_address)

                    # out = response.replace(';', ';\n')
                    # out = 'Tello State:\n' + out
                    # print(out)

                    #print(self.response)
                except socket.error as exc:
                    log.debug("joystick Thread Caught exception socket.error : %s" % exc)
        except KeyboardInterrupt as ex:
            log.info('exit from the joystick thread.') 
            # log.error('joystick thread: %s' % str(ex))
            # show_exception(ex)
            # raise(ex)
        finally:
            self.socket_state.close()
            self.__del__()
            

    def __send_stick_command(self):
        pkt = Packet(STICK_CMD, 0x60)

        axis1 = int(1024 + 660.0 * self.right_x) & 0x7ff
        axis2 = int(1024 + 660.0 * self.right_y) & 0x7ff
        axis3 = int(1024 + 660.0 * self.left_y) & 0x7ff
        axis4 = int(1024 + 660.0 * self.left_x) & 0x7ff
        axis5 = int(self.high_speed_mode * 0x7ff) & 0x7ff
        '''
        11 bits (-1024 ~ +1023) x 4 axis = 44 bits
        44 bits will be packed in to 6 bytes (48 bits)

                    axis4      axis3      axis2      axis1
             |          |          |          |          |
                 4         3         2         1         0
        98765432109876543210987654321098765432109876543210
         |       |       |       |       |       |       |
             byte5   byte4   byte3   byte2   byte1   byte0
        '''
        log.debug("stick command: yaw=%4d thr=%4d pit=%4d rol=%4d high_speed_mode=%4d" %
                  (axis4, axis3, axis2, axis1, axis5))
        log.debug("stick command: yaw=%04x thr=%04x pit=%04x rol=%04x high_speed_mode=%4d" %
                  (axis4, axis3, axis2, axis1, axis5))
        # pkt.add_byte(((axis2 << 11 | axis1) >> 0) & 0xff)
        # pkt.add_byte(((axis2 << 11 | axis1) >> 8) & 0xff)
        # pkt.add_byte(((axis3 << 11 | axis2) >> 5) & 0xff)
        # pkt.add_byte(((axis4 << 11 | axis3) >> 2) & 0xff)
        # pkt.add_byte(((axis4 << 11 | axis3) >> 10) & 0xff)
        # pkt.add_byte(((axis4 << 11 | axis3) >> 18) & 0xff)


        packedAxis = (axis1 & 0x7FF) | ((axis2 & 0x7FF) << 11) | ((0x7FF & axis3) <<22) | ((0x7FF&axis4)<< 33)| ((0x7FF&axis5) << 44)
        pkt.add_byte((packedAxis >> 0) & 0x00000000ff)
        pkt.add_byte((packedAxis >> 8) & 0x00000000ff)
        pkt.add_byte((packedAxis >> 16) & 0x00000000ff)

        pkt.add_byte((packedAxis >> 24) & 0x00000000ff)
        pkt.add_byte((packedAxis >> 32) & 0x00000000ff)
        pkt.add_byte((packedAxis >> 40) & 0x00000000ff)


        pkt.add_time()
        pkt.fixup()
        log.debug("stick command: %s" % byte_to_hexstring(pkt.get_buffer()))
        return self.send_packet(pkt)

    def send_packet(self, pkt):
        """Send_packet is used to send a command packet to the drone."""
        try:
            cmd = pkt.get_buffer()
            self.joystick_socket.sendto(cmd, self.tello_addr)
            log.debug("send_packet: %s" % byte_to_hexstring(cmd))
        except socket.error as err:
            # if self.state == self.STATE_CONNECTED:
            #     log.error("send_packet: %s" % str(err))
            # else:
            #     log.info("send_packet: %s" % str(err))
            return False

        return True

    def __send_ack_log(self, id):
        pkt = Packet(LOG_HEADER_MSG, 0x50)
        pkt.add_byte(0x00)
        b0, b1 = le16(id)
        pkt.add_byte(b0)
        pkt.add_byte(b1)
        pkt.fixup()
        return self.send_packet(pkt)


    def _receive_thread(self):
        """Listen to responses from the Tello.

        Runs as a thread, sets self.response to whatever the Tello last returned.

        """
        try:
            log.info('start receive thread')
            while True:
                try:
                    if self.quit_flag:
                        raise KeyboardInterrupt
                    # self.send_remote_ctrl()
                    self.response, ip = self.socket.recvfrom(3000)
                    #print(self.response)
                except socket.error as exc:
                    log.debug("Receive Thread Caught exception socket.error : %s" % exc)
        except KeyboardInterrupt as ex:
            log.info('exit from the recv thread.')  
            # log.error('recv thread: %s' % str(ex))
            # show_exception(ex)
            # raise(ex)
        finally:
            for i in range(10):
                self.land()
            time.sleep(1)
            self.socket.close()
            self.__del__()
                

    def _receive_video_thread(self):
        """
        Listens for video streaming (raw h264) from the Tello.

        Runs as a thread, sets self.frame to the most recent frame Tello captured.

        """
        try:
            log.info('start video thread')
            # packet_data = ""

            last_time_sent_command = time.time()

            while True:
                try:
                    if self.quit_flag:
                        raise KeyboardInterrupt

                    if not self.stream_flag:
                        self.stream_flag = self.send_command('streamon') == 'ok'
                        continue

                    if not self.command_flag:
                        self.command_flag = self.send_command('command') == 'ok'
                        continue

                    if time.time() - last_time_sent_command < 2:
                        # self.socket.sendto('command'.encode('utf-8'), self.tello_address)
                        self.socket.sendto('streamon'.encode('utf-8'), self.tello_address)
                        last_time_sent_command = time.time()
                    # self.start_video()
                    # self.connect()
                    res_string, ip = self.socket_video.recvfrom(2048)
                    # print("res_string_len = {}".format(len(res_string)))
                    self.__publish(event=self.EVENT_VIDEO_FRAME, data=res_string)
                    # packet_data += res_string
                    # end of frame
                    # if len(res_string) != 1460:
                    #     for frame in self._h264_decode(packet_data):
                    #         self.frame = frame

                        # packet_data = ""

                except socket.error as exc:
                    log.debug("Video Thread Caught exception socket.error : %s" % exc)
        except KeyboardInterrupt as ex:
            log.info('exit from the video thread.') 
            # log.error('video thread: %s' % str(ex))
            # sho_wexception(ex)
            # raise(ex)
        finally:
            self.socket_video.close()
            self.__del__()
              
    
    # def _h264_decode(self, packet_data):
    #     """
    #     decode raw h264 format data from Tello
        
    #     :param packet_data: raw h264 data array
       
    #     :return: a list of decoded frame
    #     """
    #     res_frame_list = []
    #     frames = self.decoder.decode(packet_data)
    #     for framedata in frames:
    #         (frame, w, h, ls) = framedata
    #         if frame is not None:
    #             # print 'frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls)

    #             frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
    #             frame = (frame.reshape((h, ls / 3, 3)))
    #             frame = frame[:, :w, :]
    #             res_frame_list.append(frame)

    #     return res_frame_list

    def __fix_range(self, val, min=-1.0, max=1.0):
        if val < min:
            val = min
        elif val > max:
            val = max
        return val

    def set_throttle(self, throttle):
        """
        Set_throttle controls the vertical up and down motion of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value means upward)
        """
        # if self.left_y != self.__fix_range(throttle):
            # log.info('set_throttle(val=%4.2f)' % throttle)
        self.left_y = self.__fix_range(throttle)
        # self.send_remote_ctrl()

    def set_yaw(self, yaw):
        """
        Set_yaw controls the left and right rotation of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value will make the drone turn to the right)
        """
        # if self.left_x != self.__fix_range(yaw):
            # log.info('set_yaw(val=%4.2f)' % yaw)
        self.left_x = self.__fix_range(yaw)
        # self.send_remote_ctrl()

    def set_pitch(self, pitch):
        """
        Set_pitch controls the forward and backward tilt of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value will make the drone move forward)
        """
        # if self.right_y != self.__fix_range(pitch):
            # log.info('set_pitch(val=%4.2f)' % pitch)
        self.right_y = self.__fix_range(pitch)
        # self.send_remote_ctrl()

    def set_roll(self, roll):
        """
        Set_roll controls the the side to side tilt of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value will make the drone move to the right)
        """
        # if self.right_x != self.__fix_range(roll):
            # log.info('set_roll(val=%4.2f)' % roll)
        self.right_x = self.__fix_range(roll)
        # self.send_remote_ctrl()

    def send_command(self, command):
        """
        Send a command to the Tello and wait for a response.

        :param command: Command to send.
        :return (str): Response from Tello.

        """
        if self.quit_flag:
            self.response = None
            response = 'none_response'
            return response

        log.debug(">> send cmd: {}".format(command))
        self.abort_flag = False
        timer = threading.Timer(self.command_timeout, self.set_abort_flag)

        try:
            self.socket.sendto(command.encode('utf-8'), self.tello_address)
        except socket.error as e:
            if self.connected_once:
                show_exception(e)


        timer.start()
        while self.response is None:
            if self.abort_flag is True:
                break
        timer.cancel()
        
        if self.response is None:
            response = 'none_response'
        else:
            try:
                response = self.response.decode('utf-8')
            except UnicodeDecodeError as e:
                show_exception(e)
                response = 'none_response'

        self.response = None

        return response

    def set_stream(self, flag): #arkadyzi
        if (flag):
            self.send_command('streamon')
        else:
            self.send_command('streamoff')

    def set_abort_joystick_flag(self):
        """
        Sets self.abort_flag to True.

        Used by the timer in Tello.send_command() to indicate to that a response
        
        timeout has occurred.

        """

        self.abort_joystick_flag = True
    
    def set_abort_flag(self):
        """
        Sets self.abort_flag to True.

        Used by the timer in Tello.send_command() to indicate to that a response
        
        timeout has occurred.

        """

        self.abort_flag = True

    def takeoff(self):
        """
        Initiates take-off.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """

        return self.send_command('takeoff')

    def set_high_speed_mode(self, high_speed_mode):
        if high_speed_mode:
            self.high_speed_mode = True
        else:   
            self.high_speed_mode = False

    def set_speed(self, speed):
        """
        Sets speed.

        This method expects KPH or MPH. The Tello API expects speeds from
        1 to 100 centimeters/second.

        Metric: .1 to 3.6 KPH
        Imperial: .1 to 2.2 MPH

        Args:
            speed (int|float): Speed.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """

        speed = float(speed)

        if self.imperial is True:
            speed = int(round(speed * 44.704))
        else:
            speed = int(round(speed * 27.7778))

        return self.send_command('speed %s' % speed)

    def send_remote_ctrl(self):
        a = round(self.right_x * 100)
        b = round(self.right_y * 100)
        c = round(self.left_y * 100)
        d = round(self.left_x * 100)
        cmd = 'rc %s %s %s %s' % (a, b, c, d)
        self.socket.sendto(cmd.encode('utf-8'), self.tello_address)
        return self.send_command(cmd)

    def rotate_cw(self, degrees):
        """
        Rotates clockwise.

        Args:
            degrees (int): Degrees to rotate, 1 to 360.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """

        return self.send_command('cw %s' % degrees)

    def rotate_ccw(self, degrees):
        """
        Rotates counter-clockwise.

        Args:
            degrees (int): Degrees to rotate, 1 to 360.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """
        return self.send_command('ccw %s' % degrees)

    def flip(self, direction):
        """
        Flips.

        Args:
            direction (str): Direction to flip, 'l', 'r', 'f', 'b'.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """

        return self.send_command('flip %s' % direction)

    def get_response(self):
        """
        Returns response of tello.

        Returns:
            int: response of tello.

        """
        response = self.response
        return response

    def get_height(self):
        """Returns height(dm) of tello.

        Returns:
            int: Height(dm) of tello.

        """
        height = self.send_command('height?')
        height = str(height)
        height = filter(str.isdigit, height)
        try:
            height = int(height)
            self.last_height = height
        except Exception as e:
            print e, "received height = {}".format(height)
            height = self.last_height
            pass
        return height

    def get_battery(self):
        """Returns percent battery life remaining.

        Returns:
            int: Percent battery life remaining.

        """
        
        battery = self.send_command('battery?')

        try:
            battery = int(battery)
        except Exception as e:
            print e, "received battery = {}".format(battery)
            pass

        return battery

    def get_flight_time(self):
        """Returns the number of seconds elapsed during flight.

        Returns:
            int: Seconds elapsed during flight.

        """

        flight_time = self.send_command('time?')

        try:
            flight_time = int(flight_time)
        except Exception as e:
            print e, "received flight_time = {}".format(flight_time)
            pass

        return flight_time

    def get_acceleration(self): # arkadyzi
        """Returns the IMU angular acceleration data(0.0001g).

        Returns:
            int: Seconds elapsed during flight.

        """

        res = self.send_command('acceleration?')

        #try:
        #    flight_time = int(flight_time)
        #except Exception as e:
            # print e, "received acceleration = {}".format(acceleration)
        #    pass

        return res

    def get_temperature(self): # arkadyzi
        """Returns the IMU angular acceleration data(0.0001g).

        Returns:
            int: Seconds elapsed during flight.

        """

        res = self.send_command('temp?')

        #try:
        #    flight_time = int(flight_time)
        #except Exception as e:
            # print e, "received temperature = {}".format(res)
        #    pass

        return res

    def get_attitude(self): # arkadyzi
        """Returns the IMU angular acceleration data(0.0001g).

        Returns:
            int: Seconds elapsed during flight.

        """

        res = self.send_command('attitude?')

        #try:
        #    flight_time = int(flight_time)
        #except Exception as e:
            # print e, "received temperature = {}".format(res)
        #    pass

        return res

    def get_barometer(self): # arkadyzi
        """Returns the TBD.

        Returns:
            int: Seconds elapsed during flight.

        """

        res = self.send_command('baro?')

        #try:
        #    flight_time = int(flight_time)
        #except Exception as e:
            # print e, "received barometer = {}".format(res)
        #    pass

        return res

    def get_tof(self): # arkadyzi
        """Returns the TBD.

        Returns:
            int: Seconds elapsed during flight.

        """

        res = self.send_command('tof?')

        #try:
        #    flight_time = int(flight_time)
        #except Exception as e:
            # print e, "received barometer = {}".format(res)
        #    pass

        return res

    def get_wifi_snr(self): # arkadyzi
        """Returns the TBD.

        Returns:
            int: Seconds elapsed during flight.

        """

        res = self.send_command('wifi?')

        #try:
        #    flight_time = int(flight_time)
        #except Exception as e:
            # print e, "received barometer = {}".format(res)
        #    pass

        return res

    def get_speed(self):
        """Returns the current speed.

        Returns:
            int: Current speed in KPH or MPH.

        """

        speed = self.send_command('speed?')

        try:
            speed = float(speed)

            if self.imperial is True:
                speed = round((speed / 44.704), 1)
            else:
                speed = round((speed / 27.7778), 1)
        except Exception as e:
            print e, "speed received = {}".format(speed)
            pass

        return speed


    def land(self):
        """Initiates landing.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """

        return self.send_command('land')




    def move(self, direction, distance):
        """Moves in a direction for a distance.

        This method expects meters or feet. The Tello API expects distances
        from 20 to 500 centimeters.

        Metric: .02 to 5 meters
        Imperial: .7 to 16.4 feet

        Args:
            direction (str): Direction to move, 'forward', 'back', 'right' or 'left'.
            distance (int|float): Distance to move.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """

        distance = float(distance)

        if self.imperial is True:
            distance = int(round(distance * 30.48))
        else:
            distance = int(round(distance * 100))

        return self.send_command('%s %s' % (direction, distance))

    def move_backward(self, distance):
        """Moves backward for a distance.

        See comments for Tello.move().

        Args:
            distance (int): Distance to move.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """

        return self.move('back', distance)

    def move_down(self, distance):
        """Moves down for a distance.

        See comments for Tello.move().

        Args:
            distance (int): Distance to move.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """

        return self.move('down', distance)

    def move_forward(self, distance):
        """Moves forward for a distance.

        See comments for Tello.move().

        Args:
            distance (int): Distance to move.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """
        return self.move('forward', distance)

    def move_left(self, distance):
        """Moves left for a distance.

        See comments for Tello.move().

        Args:
            distance (int): Distance to move.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """
        return self.move('left', distance)

    def move_right(self, distance):
        """Moves right for a distance.

        See comments for Tello.move().

        Args:
            distance (int): Distance to move.

        """
        return self.move('right', distance)

    def move_up(self, distance):
        """Moves up for a distance.

        See comments for Tello.move().

        Args:
            distance (int): Distance to move.

        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.

        """

        return self.move('up', distance)

if __name__ == '__main__':
    drone = Tello(local_ip='', local_port=8889, imperial=False, command_timeout=.3, 
        tello_ip='192.168.10.1', tello_port=8889, stream_=False)