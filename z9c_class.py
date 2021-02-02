from serial import Serial
from queue import LifoQueue as Stack
import asyncio
import lz4.frame
from uuid import uuid4
import logging
from time import sleep
import pickle
import sys

def pack(data, status):
    pickled = pickle.dumps(data)
    compressed = lz4.frame.compress(pickled)
    return bytearray(f"ZZZ{str(status)}BBB{len(compressed)}C".encode()) + compressed

class z9c(Serial):

    recv_buffer = Stack(maxsize=200)
    send_buffer = Stack(maxsize=200)
    dev = ""
    baud = ""

    def __init__(self, dev, baud=115200, async_mode = False):
        self.dev, self.baud = dev, baud
        super().__init__(port=self.dev, baudrate=self.baud)
        self.logger = logging.getLogger()
        self.logger.info(f"Initialized Device {dev} @Baudrate {baud}")
        self.connection = False
        self.try_timeout = 50
        self.id = str(uuid4())
        self.logger.info(f"Created Serial Device for resource {dev} as ID:{self.id}")
        self.establish_connection()
        self.async_mode = async_mode

    def clear_device_buffer(self):
        logging.debug("Clearing Device Buffer")
        while (out := self.recv())[1] == "ACK": continue

    def establish_connection(self):
        pkt=self.id
        i = 1
        while not self.connection and i <=self.try_timeout:
            self.send(pkt, status="ACK")
            if (r := self.recv())[1] == "ACK":
                self.connection = True
                self.logger.info(f"Successfully connected to ID: {r[0]}\tpacket status {r[1]}")
                self.clear_device_buffer()
                return
            else:
                self.logger.warning(f"Try: {i} Waiting to receive ACK connection packet")
                sleep(3) #wait 3 seconds for packet
                i += 1
        raise("Connection failed")
        exit(-1)

    def send(self, data, status="CON"):
        """
        sends data to serial port
        :param data: python object
        :param status: connection status
        :return: bytes written
        """
        return super().write(pack(data, status=status))

    def _send(self, data: bytearray):
        return super().write(data)

    def to_send(self, data, status="CON"):
        self.send_buffer.put(pack(data, status))

    def recv(self):
        """
        unpack algo for serial buffers

        slow algorithm

        originally made for asyncio; pyserial doesn't support asyncio; lots of unnecessary code
        :return:
        """
        flags = {"Z": 0, "B": 0}
        meta = {"status": "", "length": ""}
        def reset():
            """
            resets if data corruption detected via mis-matches
            :return: None
            """
            for i in flags.keys(): flags[i] = 0
            for i in meta.keys(): meta[i] = ""
            return (None, "CON")
        while True:
            if not super().inWaiting() > 0: return (None, "CON")
            i = super().read(1)[0]
            if chr(i) == "Z" and flags["Z"] < 3:
                flags["Z"] += 1
                continue
            elif flags["Z"] < 3 and chr(i) != "Z": reset()  #corruption condition
            # puts everything between Z & B into status string
            if flags["Z"] == 3 and chr(i) != "B" and len(meta["status"]) < 3:
                meta["status"] += chr(i)
                continue
            # cycles through B's until length
            if chr(i) == "B" and flags["B"] < 3:
                flags["B"] += 1
                continue
            elif flags["B"] < 3 and chr(i) != "B": reset() #corruption condition
            if flags["B"] == 3 and chr(i) != "C":
                meta["length"] += chr(i)
                continue
            if flags["B"] == 3 and chr(i) == "C":
                # return tuple (py object, status)
                #super().read(1) #kick "C" out of the serial buffer
                self.logger.debug(f"Attempting to load packet of size {meta['length']}")
                packet = (
                    pickle.loads(lz4.frame.decompress(super().read(int(meta["length"])))),
                    meta["status"]
                )
                self.logger.debug(f"Received Packet of size {sys.getsizeof(packet[0])} Bytes with Network Status {packet[1]}")
                if packet[1] == "FIN":
                    self.logger.warning("Lost Connection, looking for devices")
                    self.connection = False
                elif packet[1] == "ACK" and self.connection:
                    #clear buffer of residual ACK packets
                    return self.recv()
                return packet

    async def auto_run(self):
        while 1:
            if (r := self.recv())[1] != "ACK" and r != (None, "CON"):
                self.recv_buffer.put(r)
            if not self.send_buffer.empty():
                self._send(self.send_buffer.get_nowait())
            await asyncio.sleep(0.15)

    def from_recv(self):
        #manual recv
        if self.recv_buffer.empty():
            return (None, "CON")
        else:
            return self.recv_buffer.get_nowait()

    def close_with_FIN(self):
        self.send("BYE!",status= "FIN")
        super().close()
