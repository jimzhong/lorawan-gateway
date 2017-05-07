import socket
import struct
import base64
import requests
import binascii
from pprint import pprint

class LRUCache(object):

    def __init__(self, cap):
        self.l = []
        self.d = {}
        self.capacity = cap

    def __contains__(self, key):
        '''
        Test the existance of a key in cache
        '''
        if key in self.d:
            return True
        else:
            return False

    def __getitem__(self, key):
        '''
        Retrive cached value from LRU
        '''
        if key in self.d:
            value = self.d[key]
            self.l.remove(key)
            self.l.insert(0,key)
        else:
            value = None
        return value

    def __len__(self):
        return len(self.d)

    def __setitem__(self, key, value):
        '''
        Put K-V into LRU
        '''
        if key in self.d:
            self.l.remove(key)
        elif len(self.d) == self.capacity:
            # cache is full
            # remove oldest key
            oldest_key = self.l.pop()
            self.d.pop(oldest_key)

        self.d[key] = value
        self.l.insert(0, key)

class RxPacket(object):

    fmt = "<BBHLhhLLB256s";

    fmt_size = struct.calcsize(fmt)

    mac_fmt = "<BB4sBB";

    mac_fmt_size = struct.calcsize(mac_fmt);

    def __init__(self, data):
        if (len(data) < self.fmt_size):
            raise ValueError("Data too short")
        self.sf, self.cr, self.bw, self.freq, self.snr, self.rssi, \
            self.second, self.nanosecond, phy_len, phy_payload \
            = struct.unpack(self.fmt, data[:self.fmt_size])
        self.mhdr, self.control, devaddr_bytes, counter_hi, counter_lo \
            = struct.unpack(self.mac_fmt, phy_payload[:self.mac_fmt_size])
        self.devaddr = binascii.hexlify(devaddr_bytes).decode().upper()
        self.counter = (counter_hi << 8) | counter_lo;
        self.data = phy_payload[self.mac_fmt_size:phy_len]
        self.len = len(self.data)

    # def __str__(self):
    #     return "Packet of {} bytes. service_id {}.".format(self.len, self.service_id)


# if __name__ == "__main__":
#     encode_data = "Bi76AIcLHRoHAAgA/AwLWQAAAAAUAQI0EnhWAwAFaGVsbG8AAAAAAACknZP+fwAARyIHPMl/AAAmsGJlAAAAAP////8AAAAAAAAAAAAAAACAwp6T/n8AAMBmTTzJfwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAopp2T/n8AAAAAAAAAAAAAAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP+y8AAAAAAAwgAAAAAAAAAPpZ2T/n8AAA6lnZP+fwAAtTH5O8l/AAABAAAAAAAAAP0IQAAAAAAAAAAAAAAAAAAAAAAAAAAAALAIQAAAAAAAkARAAAAAAAAQpp2T/n8AAA=="
#     bin_data = base64.b64decode(encode_data)
#     # print(bin_data)
#     print("LEN = ", len(bin_data))
#
#     rxdata = RxData(bin_data)
#     pprint (vars(rxdata))

class Server(object):

    def __init__(self, ipaddr, port, uri, cache_size = 1000):
        self.addr = (ipaddr, port)
        self.uri = uri
        self.lru = LRUCache(cache_size)

    def verify(self, packet):
        '''
        Check if this packet is a duplicate
        Return true if not
        '''
        key = (packet.devaddr, packet.counter)
        if key in self.lru:
            return False
        else:
            self.lru[key] = packet;
            return True


    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.sock.bind(self.addr)
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                print("addr = ", addr)
                packet = RxPacket(data)
                print("devaddr = {} counter = {} len = {}".format(packet.devaddr, packet.counter, packet.len))
                if (self.verify(packet)):
                    self.send_to_app_server(packet)
                else:
                    print("Duplicate packet.")
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(e)
        self.sock.close()

    def send_to_app_server(self, packet):
        # pprint (vars(packet))
        try:
            data = {
                "devaddr": packet.devaddr,
                "data": base64.b64encode(packet.data)
                }
            # pprint(data)
            resp = requests.post(self.uri, data)
            print("Response: ", resp.text)
        except requests.exceptions.RequestException as e:
            print(e)

if __name__ == "__main__":
    server = Server("0.0.0.0", 22222, "http://127.0.0.1:8080/api/update.php")
    server.run()
