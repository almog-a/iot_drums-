import time
import rtmidi
import serial
import time


class play_midi:
    def __init__(self, is_midi=False):
        self.is_midi = is_midi
        self.midiout = rtmidi.MidiOut()
        self.time_sleep=0.05
        self.def_velocity=80
        available_ports = self.midiout.get_ports()
        self.current_velocity=80
        self.is_arduino_connected=False
        self.port = []
        self.isDepthOn=False
        # here we're printing the ports to check that we see the one that loopMidi created.
        # In the list we should see a port called "loopMIDI port".
        print(available_ports)

         #Attempt to open the port
        if self.is_midi:
            self.open_port()
            # if available_ports:
            #     self.midiout.open_port(2)
            # else:
            #     self.midiout.open_virtual_port("My virtual output")

    def set_isDepthOn(self,flag):
        self.isDepthOn = flag

    def get_isDepthOn(self):
        return self.isDepthOn

    def open_port(self):
        available_ports = self.midiout.get_ports()
        if available_ports:
            self.port = self.midiout.open_port(2)
        else:
            self.port = self.midiout.open_virtual_port("My virtual output")

    def close_port(self):
        self.port.close_port()


    def arduino_config(self, flag):
        self.is_arduino_connected = flag
        if (self.is_arduino_connected):
            self.s1 = serial.Serial('COM3', 9600)
            time.sleep(3)
            return self.s1

    def close_arduino(self):
        self.is_arduino_connected = False
        self.s1.close()

    def get_is_arduino_connected(self):
        return self.is_arduino_connected

    def get_s1(self):
        return self.s1

    def set_midi_flag(self,flag):
        self.is_midi=flag
    def get_midi_flag(self):
        return self.is_midi

    def set_current_velocity(self,current_val):
        self.current_velocity=min(current_val,127)
        if(self.current_velocity<8):
            self.current_velocity=self.current_velocity*2


    def play_hihate(self,velocity=100):
        if(velocity==0): return
        print("current velocity")
        print(self.current_velocity)
        note_on = [0x90, 42, velocity]
        note_off = [0x80, 42, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_kick(self,velocity=100):
        if (velocity == 0): return
        print("current velocity")
        print(self.current_velocity)
        note_on = [0x90, 36, velocity]
        note_off = [0x80, 36, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_floor(self,velocity=100):
        if (velocity == 0): return
        print("current velocity")
        print(self.current_velocity)
        note_on = [0x90, 41, velocity]
        note_off = [0x80, 41, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_tom(self,velocity=100):
        if (velocity == 0): return
        print("current velocity")
        print(self.current_velocity)
        note_on = [0x90, 45, velocity]
        note_off = [0x80, 45, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_snare(self,velocity=100):
        if (velocity == 0): return
        velocity=max(velocity-40,25)
        print("current velocity")
        print(self.current_velocity-20)
        note_on = [0x90, 38, velocity]
        note_off = [0x80, 38, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_ride(self,velocity=100):
        if (velocity == 0): return
        print("current velocity")
        print(self.current_velocity)
        note_on = [0x90, 51, velocity]
        note_off = [0x80, 51, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)
    #
    # def convert_velocity(self,old_velocity=80):
    #     print(old_velocity)
    #     #get velocity from 0 to 10000
    #     #converts to 0 to 127
    #     velocity=(old_velocity/10000)*127
    #     print(velocity)
    #     return velocity


if __name__== "__main__":

    pm = play_midi()
    while(1):
        pm.play_hihate()
        pm.play_kick()
        pm.play_floor()
        pm.play_tom()
        pm.play_ride()
        pm.play_snare()

        #  I tried running the script without having to invoke the sleep function but it doesn't work.
        # If someone could enlighten me as to why this is, I'd be more than grateful.


    #del midiout