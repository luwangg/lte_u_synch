#!/usr/bin/env python3
import zmq
import numpy as np

__author__ = "Piotr Gawlowicz"
__copyright__ = "Copyright (c) 2017, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "gawlowicz@tkn.tu-berlin.de"


class LtFiEnbManager(object):
    lteFrameTime = 10  # ms
    lteSubFrameTime = 1  # ms
    lteSlotTime = 0.5  # ms

    mappingM1 = {'0': 0, '1': 1}
    mappingM2 = {'00': 0, '01': 1, '10': 2, '11': 3}
    mappingM3 = {'000': 0, '001': 1, '010': 2, '011': 3,
                 '100': 4, '101': 5, '110': 6, '111': 7}

    def __init__(self, address="tcp://127.0.0.1:5555"):
        self.eNbAddress = address
        self.cycleDuration = 40  # ms
        self.maxDutyCycle = 25  # %
        self.onTimeDuration = 10  # ms

        self.preambleSymbols = [1, 5, 7, 8]
        self.generate_preamble()
        self.generate_symbols()

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)

        self.rfFreq = 2442
        self.rfAmp = 0.8
        self.rfGain = 30
        self.lteMcs = 0

        self.connect_to_srsLte()

    def connect_to_srsLte(self):
        self.socket.connect(self.eNbAddress)
        print("Connected to srsLte : ", self.eNbAddress)

    def disconnect_from_srsLte(self):
        print("Close connection towards srsLte...")
        self.socket.close()
        self.context.term()

    def set_enb_ip(self, value):
        self.eNbAddress = value

    def get_enb_ip(self):
        return self.eNbAddress

    def set_cycle_duration(self, value):
        if value != self.cycleDuration:
            self.cycleDuration = value
            self.generate_preamble()
            self.generate_symbols()

    def get_cycle_duration(self):
        return self.cycleDuration

    def set_max_duty_cycle(self, value):
        if value != self.maxDutyCycle:
            self.maxDutyCycle = value
            self.generate_preamble()
            self.generate_symbols()

    def get_max_duty_cycle(self):
        return self.maxDutyCycle

    def set_ontime_duration(self, value):
        if value != self.onTimeDuration:
            self.onTimeDuration = value
            self.generate_preamble()
            self.generate_symbols()

    def get_ontime_duration(self):
        return self.onTimeDuration

    def generate_preamble(self):
        lteuCycle = np.zeros(self.cycleDuration, dtype=np.uint)
        rangeStart = np.uint(0)
        punctureNum = 1
        rangeEnd = np.uint(rangeStart + self.onTimeDuration + punctureNum)
        lteuCycle[rangeStart:rangeEnd] = np.uint(1)

        self.preambleSignal = []

        for s in self.preambleSymbols:
            puncturePosition = s + 1
            puncturedCycle = np.copy(lteuCycle)
            puncturedCycle[puncturePosition] = np.uint(0)
            if not len(self.preambleSignal):
                self.preambleSignal = puncturedCycle
            else:
                self.preambleSignal = np.concatenate((self.preambleSignal, puncturedCycle), axis=0)

    def generate_symbols(self):
        lteuCycle = np.zeros(self.cycleDuration, dtype=np.uint)
        rangeStart = np.uint(0)
        punctureNum = 1
        rangeEnd = np.uint(rangeStart + self.onTimeDuration + punctureNum)
        lteuCycle[rangeStart:rangeEnd] = np.uint(1)

        self.symbolSignals = np.empty([0, self.cycleDuration], dtype=np.uint)
        symNum = 2**3

        for i in range(0, symNum):
            puncturePosition = i + 1
            puncturedCycle = np.copy(lteuCycle)
            puncturedCycle[puncturePosition] = np.uint(0)
            self.symbolSignals = np.concatenate((self.symbolSignals, np.uint([puncturedCycle])), axis=0)

    def symbol_to_lte_cycle_mapping(self, symbols):
        schedule = np.array([], dtype=np.uint)
        for s in symbols:
            schedule = np.append(schedule, self.symbolSignals[s])
        return schedule

    def set_rf_freq(self, freq):
        self.rfFreq = freq
        msgType = '0'
        message = str(freq)
        self.socket.send_multipart([msgType.encode('utf-8'), message.encode('utf-8')])

        response = self.socket.recv()
        response = response.decode('utf-8')
        return response

    def get_rf_freq(self, freq):
        return self.rfFreq

    def set_rf_amp(self, amp):
        self.rfAmp = amp
        msgType = '1'
        message = str(amp)
        self.socket.send_multipart([msgType.encode('utf-8'), message.encode('utf-8')])

        response = self.socket.recv()
        response = response.decode('utf-8')
        return response

    def get_rf_amp(self):
        return self.rfAmp

    def set_rf_gain(self, gain):
        self.rfGain = gain
        msgType = '2'
        message = str(gain)
        self.socket.send_multipart([msgType.encode('utf-8'), message.encode('utf-8')])

        response = self.socket.recv()
        response = response.decode('utf-8')
        return response

    def get_rf_gain(self):
        return self.rfGain

    def set_lte_mcs(self, index):
        self.lteMcs = index
        msgType = '3'
        message = str(index)
        self.socket.send_multipart([msgType.encode('utf-8'), message.encode('utf-8')])

        response = self.socket.recv()
        response = response.decode('utf-8')
        return response

    def get_lte_mcs(self):
        return self.lteMcs

    def send_schedule(self, schedule):
        msgType = '4'
        message = schedule
        self.socket.send_multipart([msgType.encode('utf-8'), message.encode('utf-8')])

        response = self.socket.recv()
        response = response.decode('utf-8')
        return response

    def send_sync_signal(self):
        preamble = ''.join(str(x) for x in self.preambleSignal)

        randomPunctures = [4, 4, 3, 2, 7, 3]
        randomPunctures = self.symbol_to_lte_cycle_mapping(randomPunctures)
        randomPuncturesSchedule = ''.join(str(x) for x in randomPunctures)

        schedule = preamble + randomPuncturesSchedule
        return self.send_schedule(schedule)

    def start_tx(self):
        preamble = ''.join(str(x) for x in self.preambleSignal)

        randomPunctures = [4, 4, 3, 2, 7, 3]
        randomPunctures = self.symbol_to_lte_cycle_mapping(randomPunctures)
        randomPuncturesSchedule = ''.join(str(x) for x in randomPunctures)

        schedule = preamble + randomPuncturesSchedule
        return self.send_schedule(schedule)
