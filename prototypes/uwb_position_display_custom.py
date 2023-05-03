import time
import turtle
import cmath
import socket
import json
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

hostname = socket.gethostname()
#UDP_IP = socket.gethostbyname(hostname)
#UDP_IP = "192.168.1.202"
UDP_IP = "192.168.1.4" 
print("***Local ip:" + str(UDP_IP) + "***")
UDP_PORT = 80
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((UDP_IP, UDP_PORT))
sock.listen(1)
data, addr = sock.accept()
distance_a1_a2 = 2.0
meter2pixel = 100
range_offset = 0.9



def read_data():
    print("Reading data")
    line = data.recv(1024).decode('UTF-8')

    uwb_list = []

    try:
        uwb_data = json.loads(line)
        print(uwb_data)

        uwb_list = uwb_data["links"]
        for uwb_archor in uwb_list:
            print(uwb_archor)

    except:
        print(line)
    print("")

    return uwb_list


def tag_pos(r1, r2): 
    h = 2
    h_t = 0.178
    l = 3.20
    dh = h-h_t
    r1+=0.0
    r2+=0.0
    r1_p = cmath.sqrt(r1*r1 - dh*dh)
    r2_p = cmath.sqrt(r2*r2 - dh*dh)
    alpha = cmath.acos(((r1_p*r1_p) + (l*l) - (r2_p*r2_p))/(2.0*r1_p*l))
    x = r1_p*cmath.cos(alpha)
    y = r1_p*cmath.sin(alpha)
    print(f"r1: {r1} r2: {r2} r1p: {r1_p} r2p: {r2_p} alpha: {alpha}")

    return round(x.real, 2), round(y.real, 2)


def main():

    a1_range = 0.0
    a2_range = 0.0
    x_history = []
    y_history = []
    fig, ax = plt.subplots()
    while True:
        node_count = 0
        list = read_data()
        print(list)
        for one in list:
            if one["A"] == "84":
                #a1_range = uwb_range_offset(float(one["R"]))
                a1_range = float(one["R"])
                node_count += 1

            if one["A"] == "18F0":
                #a2_range = uwb_range_offset(float(one["R"]))
                a2_range = float(one["R"])
                node_count += 1

        if node_count == 2:
            #x, y = tag_pos(a2_range, a1_range, distance_a1_a2)
            x, y = tag_pos(a1_range, a2_range)
            print(x, y)
        else:
            continue
        x_history.append(x)
        y_history.append(y)
        colors = cm.rainbow(np.linspace(0,1,len(x_history)))

        ax.scatter(x_history,y_history,color=colors)
        ax.plot(0,0,'rx')
        ax.plot(3.2,0,'bx')
        ax.grid()
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        fig.canvas.draw()
        plt.show(block=False)
        #plt.colorbar()
        time.sleep(0.03)
        ax.cla()

if __name__ == '__main__':
    main()
