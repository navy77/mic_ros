import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe
from time import *
host = '192.168.1.101'
port = 1883
topic = '/ros_mqtt'
client = mqtt.Client()
client.connect(host,port)
status = "ready"

def mqtt_data():
    global msg_payload
    msg = subscribe.simple(topics=str(topic), hostname=host,port=port)
    msg_payload = msg.payload.decode("utf-8", "strict")
    return msg_payload

def move_straight(distance):
     enc_tick = distance * 10
     for x in range(enc_tick):
        print(x)
        sleep(1)

def program1():
    move_straight(2)
    
def a():
    x=0
    while(x<5):
        print(x)
        x= x+1
        sleep(0.5)

def select():
    if data_input=="a":
        a()

def cal(distance):
    vel_x=1
    # print(abs(distance))
    a = distance/abs(distance)*vel_x
    print(a)

def main():
    # global data_input
    cal(100)
    # while(1):
    #     data_input = input("select program")
    #     select()
    # while(1):
    #         program1()
        # if str(status) == "ready":
        #     print(status)
        #     mqtt_data()
        #     print(msg_payload)
        #     status = str(msg_payload)
        #     print(status)
        #     sleep(5)
        #     status = "ready"
        #     print(status)
        #     print(msg_payload)

        #     for x in range(10):
        #         print(x)
        # else:
        #     print("NG")

if __name__ == '__main__':
    try:
        main()
    except InterruptedError or KeyboardInterrupt:
        print("Error!!!")