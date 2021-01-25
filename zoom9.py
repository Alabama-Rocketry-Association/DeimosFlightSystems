from serial import Serial
import pickle
import timeit
import datetime
import pandas
import logging
from enum import Enum
from uuid import uuid4
from time import sleep
import time
from queue import LifoQueue as Stack
import asyncio
import sys
import argparse
global device
logging.basicConfig(
        level=logging.INFO,
        #format="%(asctime)s [%(levelname)s] %(messages)s"
    )

#logging.getLogger().addHandler(logging.FileHandler(f"output-{datetime.datetime.now().strftime('%X')}.log"))
logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))

class PacketStatus(Enum):
    ACK = "ACK"
    CON = "CON"
    FIN = "FIN"

def pack(data, status:PacketStatus):
    pickled = pickle.dumps(data)
    return bytearray(f"ZZZ{str(status)}BBB{len(pickled)}C".encode()) + pickled

def unpack(packet):
    """
    test suite for unpacking algorithm
    :param packet:
    :return:
    """
    flags = {"Z":0, "B":0}
    meta = {"status":"", "length": ""}
    def reset():
        for i in flags.keys(): flags[i] = 0
        for i in meta.keys(): meta[i] = ""
    for cnt, i in enumerate(packet):
        # cycles through all Z's until status
        if chr(i) == "Z" and flags["Z"] < 3:
            flags["Z"] += 1
            continue
        # puts everything between Z & B into status string
        if flags["Z"] == 3 and chr(i) != "B" and len(meta["status"]) < 3:
            meta["status"] += chr(i)
            continue
        #cycles through B's until length
        if chr(i) == "B" and flags["B"] < 3:
            flags["B"] += 1
            continue
        if flags["B"] == 3 and chr(i) != "C":
            meta["length"] += chr(i)
            continue
        if flags["B"] == 3 and chr(i) == "C":
            #return tuple (py object, status)
            #cnt + 1 because we are at C not the first datum of serialized pickle
            return (pickle.loads(packet[cnt+1:cnt+1+int(meta["length"])]), meta["status"])


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
                    pickle.loads(super().read(int(meta["length"]))),
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

from os import system


    """
    second last to be called 
    
    then:
    
        loop = asyncio.get_event_loop()
        loop.run_until_complete(init_corountine_methods(methods : list))
    """

def term(dev, baud):
    """
    opens telnet client for configuring radio hardware settings on iot radio device
    :param dev:
    :param baud:
    :return:  none
    """
    system(f"python3 -m serial.tools.miniterm {dev} {baud}")
    print(f"Reload {dev} - bug when in miniterm causes connection to stay open past process")
    exit(0)

from textwrap import dedent
def ping_pong(dev, baud):
    device = z9c(dev, baud)
    while device.connection:
        msg = input("\n\n[PingPongTerm]?>")
        if msg == '':
            if (r := device.recv())[0] != None: print(r)
            continue
        print("\n")
        if msg.upper() == "HELP":
            print(dedent(
                """
                Ping Pong z9c help menu:
                ===========================
                help -      help menu
                exit -      exit
                """
            ))
        elif msg.upper() == "EXIT":
            device.close_with_FIN()
            exit(0)
        written = device.send(msg)
        print(f"[Wrote msg to serial buffer! SZ:{written} Bytes]\r")
        #print(device.recv(), end="\r")
    print("Connection Closed")
    device.close_with_FIN()
    #look for connections again
    ping_pong(dev, baud)

def mainCLI():

    """
    Test Code
    """

    msg2 = dedent("""
    
    
 ________    ______      ______   ___      ___   _______    
("      "\  /    " \    /    " \ |"  \    /"  | /" _   "\   
 \___/   :)// ____  \  // ____  \ \   \  //   |(: (_/  :|   
   /  ___//  /    ) :)/  /    ) :)/\\  \/.    | \____/ |)   
  //  \__(: (____/ //(: (____/ //|: \.        |    _\  '|   
 (:   / "\\        /  \        / |.  \    /:  |   /" \__|\  
  \_______)\"_____/    \"_____/  |___|\__/|___|  (________) 
                                                            

    Jonathan Martini @2021 Alabama Rocketry Association
    
    
    """)
    print(msg2)

    parser = argparse.ArgumentParser(
        description= "z9c software test suite and debug tools",
        prog="z9c test suite"
    )
    parser.add_argument("Port", metavar="P", type=str, help="Serial Device Port")
    parser.add_argument('Baud', metavar="B", type=int, help="Serial Device Baudrate")
    parser.add_argument("-T", "--Terminal", action="store_true", help="Serial Device Telnet Terminal")
    parser.add_argument("-PP", "--PingPong", action="store_true", help="Ping Pong over z9c mesh net")
    args = parser.parse_args()
    if args.Terminal:
        term(args.Port, args.Baud)
    elif args.PingPong:
        ping_pong(args.Port, args.Baud)

if __name__ == "__main__":
    mainCLI()
