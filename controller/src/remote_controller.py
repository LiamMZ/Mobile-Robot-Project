import socket



if __name__=="__main__":
    # UDP_IP = "10.0.0.152" #ip on Schloss
    UDP_IP = "192.168.43.9" #ip on razer phone
    UDP_PORT = 1998
    message = b"s"

    

    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    
    while True:
        message = input("Input f/b/r/l/s: ").encode()
        sock.sendto(message, (UDP_IP, UDP_PORT))
        
        print("UDP target IP: %s" % UDP_IP)
        print("UDP target port: %s" % UDP_PORT)
        print("message: %s" % message)

