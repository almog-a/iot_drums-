import time
import rtmidi



class play_midi:
    def __init__(self):
        self.midiout = rtmidi.MidiOut()
        self.time_sleep=0.05
        available_ports = self.midiout.get_ports()

        # here we're printing the ports to check that we see the one that loopMidi created.
        # In the list we should see a port called "loopMIDI port".
        print(available_ports)

        # Attempt to open the port
        if available_ports:
            self.midiout.open_port(2)
        else:
            self.midiout.open_virtual_port("My virtual output")

    def play_hihate(self,velocity=100):
        note_on = [0x90, 60, velocity]
        note_off = [0x80, 60, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_kick(self,velocity=100):
        note_on = [0x90, 36, velocity]
        note_off = [0x80, 36, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_floor(self,velocity=100):
        note_on = [0x90, 41, velocity]
        note_off = [0x80, 41, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_tom(self,velocity=100):
        note_on = [0x90, 45, velocity]
        note_off = [0x80, 45, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_snare(self,velocity=100):
        note_on = [0x90, 38, velocity]
        note_off = [0x80, 38, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

    def play_ride(self,velocity=100):
        note_on = [0x90, 51, velocity]
        note_off = [0x80, 51, 0]
        self.midiout.send_message(note_on)
        time.sleep(self.time_sleep)
        self.midiout.send_message(note_off)

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