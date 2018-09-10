import os
import time
import datetime
import logging
import threading
import numpy as np
import pyric
import pyric.pyw as pyw

__author__ = "Piotr Gawlowicz"
__copyright__ = "Copyright (c) 2017, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "gawlowicz@tkn.tu-berlin.de"


DTYPE_REGISTER_LOG_TLV = np.dtype([
    ("host_time", np.uint64),
    ("tsf_upper", np.uint32),
    ("tsf_lower", np.uint32),
    ("mac_counter", np.uint32),
    ("tx_busy", np.uint32),
    ("rx_busy", np.uint32),
    ("ed_busy", np.uint32),
    ("tsf_lower_2", np.uint32),
    ("register_7", np.uint32),
    ("register_8", np.uint32),
    ("register_9", np.uint32),
    ("register_10", np.uint32),
    ("register_11", np.uint32),
    ("channel", np.uint16),
]).newbyteorder('>')


class RegMonReader(threading.Thread):
    def __init__(self, iface='wlp2s0', sampling_interval=10,
                 collectRawSignal=False, collectionTime=None,
                 read_interval=1.0, result_file=None,
                 outFileSubStr=None, cwd='./'):
        threading.Thread.__init__(self)
        self.log = logging.getLogger("{module}.{name}".format(
            module=self.__class__.__module__, name=self.__class__.__name__))

        # pin process to single core, otherwise can crush
        os.system("taskset -p -c 0 %d" % os.getpid())

        self.iface = iface
        self.sampling_interval = sampling_interval
        self.collectionTime = collectionTime
        self.collectRawSignal = collectRawSignal
        self.read_interval = read_interval
        self.result_file = result_file
        self.outFileSubStr = outFileSubStr
        self.cwd = cwd
        self.lastStats = None

        self.startT = time.time()
        self.endT = time.time()

        self.statsCallback = None

        self.rxSignalQueue = None

        self.phyName = self.get_device_phyid(iface)
        if not self.phyName:
            self.log.error("Wrong interface name: {}, exit".format(iface))
            return
        self.log.info("Phy of interface: {} is {}".format(iface, self.phyName))
        self.logFileName = '/sys/kernel/debug/ieee80211/{}/regmon/rfs_register_log0'.format(self.phyName)
        self.logFd = None

        self.ccaThreshold = 28  # -62dBm in ath9k

        self.enableReading = False

    def read_file(self, fn):
        fd = open(fn, 'r')
        dat = fd.read()
        fd.close()
        return dat

    def write_file(self, fn, msg):
        f = open(fn, 'w')
        f.write(msg)
        f.close()
        return None

    def add_register(self, reg_addr, reg_num):
        fn = '/sys/kernel/debug/ieee80211/{}/regmon/register_{}'.format(self.phyName, reg_num)
        self.write_file(fn, reg_addr)
        return None

    def ath9k_translate_cca_threshold(self, value):
        keys = [-95, -92, -78, -70, -62]
        samples = {}
        samples[-62] = 28
        samples[-70] = 20
        samples[-78] = 12
        samples[-92] = 3
        samples[-95] = 0

        ath9kValue = 28
        if value <= -95:
            ath9kValue = 0
            # print("My value: {} -> ath9k value {}".format(value, ath9kValue))
            return ath9kValue

        if value >= -62:
            diff = value + 62
            ath9kValue = 28 + diff
            # print("My value: {} -> ath9k value {}".format(value, ath9kValue))
            return ath9kValue

        lowerBound = None
        upperBound = None
        for i in range(len(keys)):
            if keys[i] <= value and value < keys[i + 1]:
                lowerBound = keys[i]
                upperBound = keys[i + 1]
                break

        ath9kValue = samples[lowerBound] + ((value-lowerBound) / (upperBound-lowerBound)) * (samples[upperBound] - samples[lowerBound])
        ath9kValue = int(ath9kValue)
        # print("My value: {} -> ath9k value {}".format(value, ath9kValue))
        return ath9kValue

    def get_cca_threshold(self):
        regIdxPath = "/sys/kernel/debug/ieee80211/{}/ath9k/regidx".format(self.phyName)
        regValPath = "/sys/kernel/debug/ieee80211/{}/ath9k/regval".format(self.phyName)
        ccaThresholdReg = "0x9864"
        ccaThresholdReg = "0x9e1c"
        ccaThresholdMask = "0x000FF000"
        shitfVal = 12
        ccaThresholdMaskInt = int(ccaThresholdMask, 16)

        self.write_file(regIdxPath, ccaThresholdReg)
        regIdx = self.read_file(regIdxPath).rstrip()
        regVal = self.read_file(regValPath).rstrip()

        regValInt = int(regVal, 16)
        regValMasked = ccaThresholdMaskInt & regValInt
        regValShifted = regValMasked >> shitfVal
        return regValShifted

    def set_cca_threshold(self, newCcaThresh):
        # cd /sys/kernel/debug/ieee80211/phy0/ath9k
        # echo "0x9864" > regidx
        # echo "0x1991cf9c" > regval
        regIdxPath = "/sys/kernel/debug/ieee80211/{}/ath9k/regidx".format(self.phyName)
        regValPath = "/sys/kernel/debug/ieee80211/{}/ath9k/regval".format(self.phyName)
        ccaThresholdReg = "0x9864"
        ccaThresholdReg = "0x9e1c"
        ccaThresholdMask = "0x000FF000"
        shitfVal = 12
        wildCardMask = "0xFFFFFFFF"

        ccaThresholdMaskInt = int(ccaThresholdMask, 16)
        wildCardMaskInt = int(wildCardMask, 16)
        ccaThresholdMaskInverted = ccaThresholdMaskInt ^ wildCardMaskInt

        self.write_file(regIdxPath, ccaThresholdReg)
        regIdx = self.read_file(regIdxPath).rstrip()
        regVal = self.read_file(regValPath).rstrip()
        regValInt = int(regVal, 16)

        # print("Register idx: {}, Register value: {}".format(regIdx, regVal))
        regValOldMasked = ccaThresholdMaskInverted & regValInt

        newCcaThresh = newCcaThresh << shitfVal
        newRegVal = regValOldMasked | newCcaThresh
        newRegVal = "0x{:08x}".format(newRegVal)
        self.write_file(regValPath, newRegVal)

    def set_sampling_interval(self, ival):
        ival_str = str(int(ival * 1e6))  # convert ms => ns
        fn = '/sys/kernel/debug/ieee80211/{}/regmon/sampling_interval'.format(self.phyName)
        self.write_file(fn, ival_str)
        return None

    def get_sampling_interval(self):
        fn = '/sys/kernel/debug/ieee80211/{}/regmon/sampling_interval'.format(self.phyName)
        ival = int(self.read_file(fn).strip())  # ns
        return ival

    def disable_ani(self):
        ival_str = str(0)
        fn = '/sys/kernel/debug/ieee80211/{}/ath9k/ani'.format(self.phyName)
        self.write_file(fn, ival_str)
        return None

    def get_data(self):
        try:
            data = self.logFd.read()
            data = np.frombuffer(data, dtype=DTYPE_REGISTER_LOG_TLV)
            #data = list(map(lambda line: list(line), data))
            #data = np.array(data)
            return data
        except Exception as e:
            self.log.error("Exception: {}".format(e))
            return None

    def clear_buffers(self):
        try:
            self.logFd.read()
        except Exception:
            pass

    def get_current_channel(self):
        w0 = pyw.getcard(self.iface)
        return pyw.chget(w0)

    def get_device_phyid(self, iface):
        ''' get phy id for this interface '''
        fn = '/sys/class/net/{}/phy80211/name'.format(iface)
        if (os.path.isfile(fn)):
            phyid = self.read_file(fn).strip()
            return phyid
        return None

    def save_data(self, data, fn=''):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        if self.outFileSubStr:
            fn = "rx-raw-signal-" + self.outFileSubStr + "-" + timestamp + '.npy'
        else:
            fn = "rx-raw-signal-" + timestamp + '.npy'
        if self.result_file:
            fn = self.result_file

        fn = self.cwd + fn
        print("Save data to : {}".format(fn))
        np.save(fn, data)

    def set_signal_receiver_queue(self, queue):
        self.rxSignalQueue = queue

    def stop(self):
        self.enableReading = False

    def run(self):
        self.log.info("Disable ANI for {}".format(self.phyName))
        self.disable_ani()

        self.log.info("Start RegMon for {}".format(self.phyName))

        # set RegMon sampling interval
        self.log.info("Set sampling interval to {} ms".format(self.sampling_interval))
        self.set_sampling_interval(self.sampling_interval)
        time.sleep(0.5)

        # add AR_ACK_FAIL (0x8090) register on position 7 of the RegMon struct
        reg_addr = '0x8090'
        reg_num = 7
        self.log.info("Add register {} to position {}.".format(str(reg_addr), reg_num))
        self.add_register(reg_addr, reg_num)

        # empty RegMon buffers to effectively apply
        # new sampling interval setting on all samples
        self.log.info("Clear RegMon buffers")

        # wait for new samples
        time.sleep(self.read_interval)

        self.log.info("Start collecting data")
        self.logFd = open(self.logFileName, 'rb')
        self.clear_buffers()
        time.sleep(0.5)

        allData = None
        first = False
        startCollection = time.time()
        self.enableReading = True

        while self.enableReading:
            self.startT = time.time()
            data = self.get_data()

            if self.collectRawSignal:
                if not first:
                    first = True
                    allData = data
                else:
                    allData = np.concatenate((allData, data), axis=0)

            if self.rxSignalQueue:
                self.rxSignalQueue.put(data)

            # wait for new samples
            self.endT = time.time()
            computationTime = self.endT - self.startT
            steepTime = self.read_interval - computationTime
            try:
                time.sleep(steepTime)
            except Exception:
                time.sleep(1)
            self.endT = time.time()

        print("Collection time: {}".format(time.time() - startCollection))
        if self.collectRawSignal:
            self.save_data(allData)
        self.log.info("STOP: Ctrl-C -> RegMon exits")
        self.logFd.close()
