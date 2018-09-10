import sh
import datetime
import logging
import numpy as np
import queue
import zmq
import threading
import peakutils

__author__ = "Piotr Gawlowicz"
__copyright__ = "Copyright (c) 2017, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "gawlowicz@tkn.tu-berlin.de"


class LteuSynchronizer(threading.Thread):

    def __init__(self, samplingInt=0.25, cycleLength=40, onTime=11,
                 punctureTime=1, thresholdAdaptation=True, winterface="wlan0"):
        threading.Thread.__init__(self)
        self.log = logging.getLogger("{module}.{name}".format(
            module=self.__class__.__module__, name=self.__class__.__name__))

        self.signalQueue = queue.Queue()

        # RX chain parameters
        self.ccaThreshold = -62
        # Decoder parameters
        self.samplingInterval = samplingInt
        self.Fs = (1 / self.samplingInterval) * 1000
        self.thresholdAdaptation = thresholdAdaptation

        self.samplesToCollect = np.uint(1.0 * self.Fs)
        self.detectionStats = {}

        self.punctureTime = punctureTime  # ms

        self.cycleLength = cycleLength  # ms
        self.onTime = onTime  # ms

        # LTE-U cycle length detection
        N = np.uint(0.5 * self.Fs)
        self.cycleLengthDetectBuffer = np.zeros(N)
        self.cycleLenghtBufferSampleNum = 0

        self.generate_symbols()

        # publish received msg over zmq
        self.port = 8987
        self.context = zmq.Context()
        self.pubSocket = self.context.socket(zmq.PUB)
        self.pubSocket.bind("tcp://*:%s" % self.port)

        # to setup variables
        self.clean_variables()

        # channel switching
        self.winterface = winterface
        self.currentChannel = 0
        self.channels = [1, 6, 11]
        self.freq = [2412, 2437, 2462]
        #self.freq = [2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462]

        shLogger = logging.getLogger('sh')
        shLogger.setLevel(logging.CRITICAL)

        self.switch_channel(self.channels[0])

    def get_signal_queue(self):
        return self.signalQueue

    def generate_symbols(self):
        self.offTime = self.cycleLength - self.onTime
        self.samplesPerSymbol = np.uint64(self.punctureTime / self.samplingInterval)
        self.samplesPerCycleLength = np.uint64(self.cycleLength / self.samplingInterval)

        # Generate perfect LTE cycle
        entireLteCycle = np.zeros(self.samplesPerCycleLength, dtype=np.float)
        onSampleNum = np.uint64(self.onTime / self.samplingInterval)

        # rangeStart = np.uint64(self.samplesPerCycleLength / 2 - onSampleNum / 2)
        # synchronize at the beginning of LTE-U cycle
        rangeStart = np.uint64(0)
        rangeEnd = np.uint64(rangeStart + onSampleNum)
        entireLteCycle[rangeStart:rangeEnd] = np.float(1)
        refenenceSignal = np.empty([0, self.samplesPerCycleLength])

        for i in range(1, 10):
            puncurePosition = i
            punctureStartTime = np.uint64(rangeStart+puncurePosition*self.samplesPerSymbol)
            punctureStopTime = np.uint64(rangeStart+(puncurePosition+1)*self.samplesPerSymbol)
            puncturedCycle = np.copy(entireLteCycle)
            puncturedCycle[punctureStartTime:punctureStopTime] = np.float(-10)
            puncturedCycle = puncturedCycle # - 0.5
            refenenceSignal = np.concatenate((refenenceSignal, [puncturedCycle]), axis=0)

        entireLteCycle = entireLteCycle #- 0.5
        self.refenenceSignal = refenenceSignal

        # Generate preamble
        self.preambleSymbols = np.array([1, 5, 7, 8])
        self.preamble = np.concatenate((np.copy(refenenceSignal[1]),np.copy(refenenceSignal[5]), np.copy(refenenceSignal[7]), np.copy(refenenceSignal[8])) , axis=0)
        #self.preamble = self.preamble - np.min(self.preamble)
        rxPreamble = np.copy(self.preamble)
        rxPreamble[rxPreamble<-1] = -1
        rxPreamble = 0.5 * rxPreamble

        self.maxCorr = np.correlate(self.preamble, rxPreamble, mode="valid")
        self.detectionThreshold = 0.75 * self.maxCorr
        self.preambleLength = len(self.preambleSymbols)
        self.log.info("Preamble Length [symbols] {}".format(self.preambleLength))
        self.log.info("Max Preamble CC: {}".format(self.maxCorr))
        self.log.info("Preamble Detection Threshold: {}".format(self.detectionThreshold))

    def autocorr(self, x):
        result = np.correlate(x, x, mode='full')
        return result

    def smooth(self, y, box_pts):
        box = np.ones(box_pts) / box_pts
        y_smooth = np.convolve(y, box, mode='same')
        return y_smooth

    def my_round(self, x, d):
        return np.uint(np.round(x / d) * d)

    def estimate_cycle_lenght(self, buff):
        autoCorr = self.autocorr(buff)
        autoCorr = self.smooth(autoCorr, 1)
        # normalize
        autoCorr = autoCorr / np.max(autoCorr)

        detectedPeaksNum = 0
        threshold = 0.9
        indexes = []

        lteuDetected = False
        ontime = 0
        dc = 0
        cycleLength = 0

        while detectedPeaksNum < 3:
            indexes = peakutils.indexes(autoCorr, thres=threshold, min_dist=40)
            detectedPeaksNum = len(indexes)
            threshold = threshold - 0.1
            if threshold < 0.3:
                # no pattern discovered
                # print("---No LTE-U detected")
                return [lteuDetected, cycleLength, ontime, dc]

        diff = np.diff(indexes, axis=0)
        avg = np.mean(diff)
        ravg = self.my_round(avg, 10)
        cycleLength = np.uint(ravg * self.samplingInterval)
        lteuDetected = True

        # check different DC to find max corr
        singleSampleNum = cycleLength / self.samplingInterval
        # 4 cycles
        sampleNum = 4 * singleSampleNum
        lteuCycle = np.zeros(np.int(sampleNum), dtype=np.float)
        maxCorr = 0.0

        samples = buff[1000:].astype(np.float)

        for i in range(np.int(cycleLength + 1)):
            lteuCycle.fill(-0.5)

            ss = np.int(0)
            ee = np.int(ss + i / self.samplingInterval)
            lteuCycle[ss:ee] = 0.5

            ss = np.int(1 * cycleLength / self.samplingInterval)
            ee = np.int(ss + i / self.samplingInterval)
            lteuCycle[ss:ee] = 0.5

            ss = np.int(2 * cycleLength / self.samplingInterval)
            ee = np.int(ss + i / self.samplingInterval)
            lteuCycle[ss:ee] = 0.5

            ss = np.int(3 * cycleLength / self.samplingInterval)
            ee = np.int(ss + i / self.samplingInterval)
            lteuCycle[ss:ee] = 0.5

            corr = np.correlate(lteuCycle, samples, mode='same')
            myMaxCorr = np.max(corr)

            if myMaxCorr > maxCorr:
                maxCorr = myMaxCorr
                ontime = i
                dc = ontime / cycleLength

        if ontime <= 3:
            # print("---No LTE-U detected")
            lteuDetected = False
            cycleLength = 0
            ontime = 0
            dc = 0
            return [lteuDetected, cycleLength, ontime, dc]

        if dc <= 0.05:
            # print("---No LTE-U detected")
            lteuDetected = False
            cycleLength = 0
            ontime = 0
            dc = 0
            return [lteuDetected, cycleLength, ontime, dc]

        lteuCycle.fill(-0.5)
        ss = np.int(0)
        ee = np.int(ss + ontime / self.samplingInterval)
        lteuCycle[ss:ee] = 0.5

        ss = np.int(1 * cycleLength / self.samplingInterval)
        ee = np.int(ss + ontime / self.samplingInterval)
        lteuCycle[ss:ee] = 0.5

        ss = np.int(2 * cycleLength / self.samplingInterval)
        ee = np.int(ss + ontime / self.samplingInterval)
        lteuCycle[ss:ee] = 0.5

        ss = np.int(3 * cycleLength / self.samplingInterval)
        ee = np.int(ss + ontime / self.samplingInterval)
        lteuCycle[ss:ee] = 0.5

        maxPossibleCorr = np.correlate(lteuCycle, lteuCycle, mode='same')
        maxPossibleCorr = np.max(maxPossibleCorr)

        maxCorr = np.correlate(lteuCycle, samples, mode='same')
        maxCorr = np.max(maxCorr)

        if maxCorr <= 0.5 * maxPossibleCorr:
            # print("---No LTE-U detected")
            lteuDetected = False
            cycleLength = 0
            ontime = 0
            dc = 0
            return [lteuDetected, cycleLength, ontime, dc]

        # print("---Detected LTE-U Cycle Length [ms]: ", cycleLength)
        # print("---Detected on-time: ", ontime)
        # print("---Detected DC: ", dc)

        return [lteuDetected, cycleLength, ontime, dc]

    def clean_regmon_sample(self, regMonSample):
        # singal cleaning
        cleanSignal1 = True
        tau1 = 0.9
        cleanSignal2 = True
        tau2 = 0.25
        tau3 = 0.5

        [timestamp, intfSample, txSample, rxSample, idleSample, channel] = regMonSample

        if cleanSignal1:
            if intfSample > tau1:
                intfSample = 1.0

        if cleanSignal2:
            if txSample > tau2:
                intfSample = 0.0
            if rxSample > tau2:
                intfSample = 0.0
            if idleSample > tau2:
                intfSample = 0.0

            RxTxIdle = txSample + rxSample + idleSample
            if RxTxIdle > tau3:
                intfSample = 0.0

        return [timestamp, intfSample, txSample, rxSample, idleSample, channel]

    def clean_variables(self):
        self.syncTimestamp = 0
        self.synchronized = False
        self.synchronizedWithCC = 0
        self.t = 0

    def reset_buffers(self):
        W = np.uint64(self.samplesPerCycleLength)
        N = np.uint64(self.preambleLength * W)
        ccHistoryLen = np.uint64(80 * W)

        self.preambleBuffer = np.zeros(N)
        self.ccBuffer = np.zeros(ccHistoryLen)

    def reset_sample_buffer(self):
        self.sampleBuffers = {}
        self.collectedSampleNum = {}
        self.busySampleBuffers = {}
        self.idleSampleBuffers = {}
        self.wifiSampleBuffers = {}

    def get_sample_num(self, channel):
        return self.collectedSampleNum.get(channel, 0)

    def add_sample(self, channel, sample):
        N = self.samplesToCollect
        buff = self.sampleBuffers.get(channel, np.zeros(N, dtype=np.float))
        sampleNum = self.collectedSampleNum.get(channel, 0)

        buff[:-1] = buff[1:]
        buff[-1] = sample
        sampleNum = sampleNum + 1

        self.sampleBuffers[channel] = buff
        self.collectedSampleNum[channel] = sampleNum

    def add_busy_sample(self, channel, sample):
        N = self.samplesToCollect
        buff = self.busySampleBuffers.get(channel, np.zeros(N, dtype=np.float))

        buff[:-1] = buff[1:]
        buff[-1] = sample

        self.busySampleBuffers[channel] = buff

    def add_idle_sample(self, channel, sample):
        N = self.samplesToCollect
        buff = self.idleSampleBuffers.get(channel, np.zeros(N, dtype=np.float))

        buff[:-1] = buff[1:]
        buff[-1] = sample

        self.idleSampleBuffers[channel] = buff

    def add_wifi_sample(self, channel, sample):
        N = self.samplesToCollect
        buff = self.wifiSampleBuffers.get(channel, np.zeros(N, dtype=np.float))

        buff[:-1] = buff[1:]
        buff[-1] = sample

        self.wifiSampleBuffers[channel] = buff

    def stop(self):
        pass

    def switch_channel(self, channel):
        print("Switch channel to ", channel)
        sh.iw(self.winterface, "set", "channel", channel)

    def get_detection_stats(self):
        return self.detectionStats

    def compute_cc(self):
        print("***Compute CC for collected samples***")

        for f in self.freq:
            print("+Channel", f)
            buff = self.sampleBuffers.get(f)

            busyBuff = self.busySampleBuffers.get(f)
            busyRatio = np.sum(busyBuff) / busyBuff.shape[0]

            idleBuff = self.idleSampleBuffers.get(f)
            idleRatio = np.sum(idleBuff) / idleBuff.shape[0]
            idleRatio = np.round(idleRatio, 2)
            busyRatio = 1.0 - idleRatio
            busyRatio = np.round(busyRatio, 2)

            wifiBuff = self.wifiSampleBuffers.get(f)
            wifiRatio = np.sum(wifiBuff) / wifiBuff.shape[0]
            wifiRatio = np.round(wifiRatio, 2)

            # try to find LTE-U cycle and DC
            [lteuDetected, cycleLength, ontime, dc] = self.estimate_cycle_lenght(buff)

            # compare with the known LTE-U schedule
            corr = np.correlate(buff, self.preamble, mode="valid")
            maxCorr = np.max(corr)
            if maxCorr > self.detectionThreshold:
                lteuDetected = True
                cycleLength = 40
                ontime = 10
                dc = ontime / cycleLength

            if lteuDetected:
                print("---Detected LTE-U Cycle Length [ms]: ", cycleLength)
                print("---Detected on-time: ", ontime)
                print("---Detected DC: ", dc)
            else:
                print("---No LTE-U detected")

            print("---Channel Busy Ratio", busyRatio)
            print("---Channel Idle Ratio", idleRatio)
            print("---Channel WiFi Ratio", wifiRatio)

            self.detectionStats[f] = {"LTE-U": lteuDetected,
                                      "cycleTime": cycleLength,
                                      "onTime": ontime,
                                      "DC": dc,
                                      "idle": idleRatio,
                                      "busy": busyRatio,
                                      "wifi": wifiRatio}

    def run(self):
        self.thresholdAdaptation = False
        self.t = 0
        self.reset_buffers()
        self.clean_variables()
        self.reset_sample_buffer()

        while True:
            self.t = self.t + 1
            regMonSample = self.signalQueue.get()
            regMonSample = self.clean_regmon_sample(regMonSample)

            [timestamp, intfSample, txSample, rxSample, idleSample, channel] = regMonSample
            timestamp = np.uint64(timestamp)
            busy = intfSample + txSample + rxSample
            wifiSample = txSample + rxSample
            intfSample = intfSample - 0.5

            self.add_sample(channel, intfSample)
            self.add_busy_sample(channel, busy)
            self.add_idle_sample(channel, idleSample)
            self.add_wifi_sample(channel, wifiSample)
            sampleNum = self.get_sample_num(self.freq[self.currentChannel])

            if sampleNum > self.samplesToCollect:
                self.currentChannel = self.currentChannel + 1
                if self.currentChannel == len(self.channels):
                    self.currentChannel = 0
                    self.compute_cc()
                    self.reset_sample_buffer()

                self.switch_channel(self.channels[self.currentChannel])


class LteuDetector(LteuSynchronizer):
    def __init__(self, samplingInt=0.25, cycleLength=40, onTime=11,
                 punctureTime=1, thresholdAdaptation=True, winterface="wlan0"):

        super().__init__(samplingInt=0.25, cycleLength=40, onTime=11,
                         punctureTime=1, thresholdAdaptation=True,
                         winterface="wlan0")

        self.log = logging.getLogger("{module}.{name}".format(
            module=self.__class__.__module__, name=self.__class__.__name__))
