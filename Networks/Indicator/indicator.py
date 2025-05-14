import datetime
import pcapy
import threading
from matplotlib import pyplot
from matplotlib import animation

packet_count = 0
data_count = 0
last_timebin = 0

class PcapThread(threading.Thread):
    def __init__(self, cap):
        threading.Thread.__init__(self)
        self.cap = cap

    def run(self):
        global packet_count, data_count
        while True:
            something, packet = self.cap.next()
            packet_count += 1
            data_count += len(packet)

def plot_cont():
    start_time = datetime.datetime.now()
    packets = [0]
    data = [0]
    fig = pyplot.figure()
    plot_packets = fig.add_subplot(1, 2, 1)
    plot_data = fig.add_subplot(1, 2, 2)
    fig.canvas.set_window_title("Network indicator")

    def update(i):
        global packet_count, data_count, last_timebin
        time_p = datetime.datetime.now()
        if (time_p - start_time).seconds > last_timebin:
            last_timebin = (time_p - start_time).seconds
            packets.append(packet_count)
            data.append(data_count)
            data_count = 0
            packet_count = 0
        if len(packets) > 60:
            del packets[0]
            del data[0]
        x = range(len(packets))

        plot_packets.clear()
        plot_packets.set_title("Packets")
        plot_packets.plot(x, packets)

        plot_data.clear()
        plot_data.set_title("Data")
        plot_data.plot(x, data)

    a = animation.FuncAnimation(fig, update, repeat = False)
    pyplot.show()

cap = pcapy.open_live("en1", 65536, 1, 50)
thread1 = PcapThread(cap)
thread1.start()
plot_cont()
