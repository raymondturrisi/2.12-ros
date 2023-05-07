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
UDP_IP = "192.168.1.6"

print("***Local ip:" + str(UDP_IP) + "***")
UDP_PORT = 9001 
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((UDP_IP, UDP_PORT))
sock.listen(1)
data, addr = sock.accept()

h = 0.6985 
h_t = 0.46355
l = 2.591

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
    dh = h-h_t
    r1+=0.21
    r2+=0.05
    r1_p = cmath.sqrt(r1*r1 - dh*dh)
    r2_p = cmath.sqrt(r2*r2 - dh*dh)
    alpha = cmath.acos(((r1_p*r1_p) + (l*l) - (r2_p*r2_p))/(2.0*r1_p*l))
    x = r1_p*cmath.cos(alpha)
    y = r1_p*cmath.sin(alpha)
    print(f"r1: {r1} r2: {r2} r1p: {r1_p} r2p: {r2_p} alpha: {alpha}")

    return round(x.real, 2), round(y.real, 2)


def main():

    x_history = []
    y_history = []
    fig, ax = plt.subplots(figsize=(8,6))
    show_history = False
    vehicle_start = [3.2, 1.8]
    aed = [3.12, 0]
    mannequin = [0.24, 2.08]
    a1_range = 0.0
    a2_range = 0.0

    while True:
        
        node_count = 0
        list = read_data()
        print(list)
        for one in list:
            if one["A"] == "84":
                a1_range = float(one["R"])
                node_count += 1

            if one["A"] == "18F0":
                a2_range = float(one["R"])
                node_count += 1

        if node_count == 2:
            x, y = tag_pos(a1_range, a2_range)
            print(x, y)
        else:
            continue

        if show_history:
            x_history.append(x)
            y_history.append(y)
            colors = cm.rainbow(np.linspace(0,1,len(x_history)))
            ax.scatter(x_history,y_history,color=colors)
        
        x_span = [-0.5, 4.5]
        y_span = [-0.5, 3.5]
        
        ax.plot(x,y,'ro')
        ax.text(x,y,f"p = ({x},{y})", style='italic')

        #Plot the location of the tags
        ax.plot(0,0,'rx')
        ax.plot(l,0,'bx')
        
        #Plot the location of the markers
        #Vehicle start
        ax.plot(vehicle_start[0],vehicle_start[1],'mo')
        ax.text(vehicle_start[0],vehicle_start[1],f"Vehicle starting position", style='italic')

        #AED Stand
        ax.plot(aed[0],aed[1],'mo')
        ax.text(aed[0],aed[1],f"AED", style='italic')

        #Mannequin
        ax.plot(mannequin[0],mannequin[1],'mo')
        ax.text(mannequin[0],mannequin[1],f"Patient", style='italic')

        ax.set_xticks(np.linspace(x_span[0], x_span[-1], 10))
        ax.set_yticks(np.linspace(y_span[0], y_span[-1], 10))
        ax.set_xlim(x_span)
        ax.set_ylim(y_span)
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

