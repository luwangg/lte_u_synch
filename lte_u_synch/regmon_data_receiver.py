import logging
import numpy as np
import threading
import queue
import zmq

__author__ = "Piotr Gawlowicz"
__copyright__ = "Copyright (c) 2017, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "gawlowicz@tkn.tu-berlin.de"


class RegMonDataReceiver(threading.Thread):
    def __init__(self, publishData=False):
        threading.Thread.__init__(self)
        self.log = logging.getLogger("{module}.{name}".format(
            module=self.__class__.__module__, name=self.__class__.__name__))

        self.signalQueue = queue.Queue()
        self.outQueues = []

        self.publishData = publishData
        self.pubPort = 56789
        self.context = zmq.Context()
        self.pubSocket = self.context.socket(zmq.PUB)
        self.pubSocket.bind("tcp://*:%s" % self.pubPort)

    def get_signal_queue(self):
        return self.signalQueue

    def add_signal_receiver_queue(self, queue):
        self.outQueues.append(queue)

    def prepare_signal(self, data):
        timestamp = np.copy(data[:, [0]].astype(np.uint64))
        timestamp = np.squeeze(timestamp)

        channel = np.copy(data[:, [13]].astype(np.int))
        channel = np.squeeze(channel)

        length = np.shape(data)[0]
        startTime = data[0][0]
        stopTime = data[-1][0]
        timeInterval = stopTime - startTime

        # filter overflow data
        macTimeRawData = data[:, [3]]
        index = np.argmin(macTimeRawData)
        # check if there is any overflow
        if (index > 0):
            # find all overflows in mactime
            overflows = map( lambda x : x[0] > x[1], zip(macTimeRawData, macTimeRawData[1:]))
            indexes = [i+1 for i,x in enumerate(overflows) if x == True]
            # TODO: look for overflows in the other registers as well
            splitData = np.split(data, indexes)
            splitData = list(map(lambda x: np.diff(x, axis=0), splitData))
            diff = np.concatenate(splitData, axis=0)

            timestamp = np.split(timestamp, indexes)
            timestamp = list(map(lambda x: x[0:-1], timestamp))
            timestamp = np.concatenate(timestamp, axis=0)

            channel = np.split(channel, indexes)
            channel = list(map(lambda x: x[0:-1], channel))
            channel = np.concatenate(channel, axis=0)
        else:
            diff = np.diff(data, axis=0)
            timestamp = timestamp[0:-1]
            channel = channel[0:-1]

        # take 4 registers and convert to signed int
        hostTimeVec = diff[:, [0]].astype(np.uint64)
        macTimeVec = diff[:, [3]].astype(np.uint64)
        txBusyVec = diff[:, [4]].astype(np.uint64)
        rxBusyVec = diff[:, [5]].astype(np.uint64)
        edBusyVec = diff[:, [6]].astype(np.uint64)
        macTimeVec = np.squeeze(macTimeVec)
        txBusyVec = np.squeeze(txBusyVec)
        rxBusyVec = np.squeeze(rxBusyVec)
        edBusyVec = np.squeeze(edBusyVec)

        # all registers must be positive; otherwise set to zero
        macTimeVec[macTimeVec < 0] = 0
        txBusyVec[txBusyVec < 0] = 0
        rxBusyVec[rxBusyVec < 0] = 0
        edBusyVec[edBusyVec < 0] = 0

        # macTime must be always larger than tx+rx+ed
        macTimeVec = np.maximum(macTimeVec, txBusyVec + rxBusyVec + edBusyVec)
        # edbusy must be larger than tx+rx
        edBusyVec = np.maximum(edBusyVec, txBusyVec + rxBusyVec)

        # idle time in each time bin
        idleVec = macTimeVec - edBusyVec

        # interference time in each bin
        interferenceVec = edBusyVec - txBusyVec - rxBusyVec

        # handle negative
        idleVec[idleVec < 0] = 0
        interferenceVec[interferenceVec < 0] = 0

        # relative values in each time bin
        np.seterr(divide='ignore', invalid='ignore')
        macTimeRatio = np.divide(macTimeVec, macTimeVec)
        idleRatio = np.divide(idleVec, macTimeVec)
        txBusyRatio = np.divide(txBusyVec, macTimeVec)
        rxBusyRatio = np.divide(rxBusyVec, macTimeVec)
        edBusyRatio = np.divide(edBusyVec, macTimeVec)
        interferenceRatio = np.divide(interferenceVec, macTimeVec)

        # sanity checks
        txBusyRatio[txBusyRatio > 1] = 1
        txBusyRatio[txBusyRatio < 0] = 0
        rxBusyRatio[rxBusyRatio > 1] = 1
        rxBusyRatio[rxBusyRatio < 0] = 0
        idleRatio[idleRatio > 1] = 1
        idleRatio[idleRatio < 0] = 0
        edBusyRatio[edBusyRatio > 1] = 1
        edBusyRatio[edBusyRatio < 0] = 0
        interferenceRatio[interferenceRatio > 1] = 1
        interferenceRatio[interferenceRatio < 0] = 0

        return [timestamp, txBusyRatio, rxBusyRatio, idleRatio, edBusyRatio, interferenceRatio, channel]

    def publish_numpy_array(self, A, flags=0, copy=True, track=False):
        """send a numpy array with metadata"""
        md = dict(
            dtype=str(A.dtype),
            shape=A.shape)

        topic = str("diffSamples").encode('utf-8')
        self.pubSocket.send(topic, flags | zmq.SNDMORE)
        self.pubSocket.send_json(md, flags | zmq.SNDMORE)
        return self.pubSocket.send(A, flags, copy=copy, track=track)

    def run(self):
        while True:
            data = self.signalQueue.get()
            if len(self.outQueues) == 0:
                continue
            data = list(map(lambda line: list(line), data))
            data = np.array(data)
            signal = np.array([])
            # we need to calculate difference,
            # so at least 2 rows needed
            if data.size > 2:
                [hostTimeStamp, txBusyRatio, rxBusyRatio, idleRatio, edBusyRatio, interferenceRatio, channel] = self.prepare_signal(data)
                signal = np.column_stack((hostTimeStamp, interferenceRatio, txBusyRatio, rxBusyRatio, idleRatio, channel))

            for sample in signal:
                for q in self.outQueues:
                    q.put(sample)
                if self.publishData:
                    self.publish_numpy_array(sample)
        return
