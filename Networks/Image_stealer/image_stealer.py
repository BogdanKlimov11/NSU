from scapy import sendrecv
import time

# steals PNG, GIF and JPEG image if it is not fragmented
def small_stealer(packet):
    packet_bytes = bytes(packet)
    begin = packet_bytes.find('\x89PNG')
    ### PNG ###
    if begin > -1: # if found PNG beginning
        print 'Found \'\x89PNG\' at', begin
        end = packet_bytes.find('IEND')
        if end > -1: # if found PNG end
            print'---Found \'IEND\' at', end
            file_name = str(time.time()) + '.png'
            with open(file_name, 'wb') as image_file:
                print 'File', file_name, 'created\n'
                image_file.write(packet_bytes[begin:])
                image_file.close()
    ### GIF ###
    begin = packet_bytes.find('GIF8')
    if begin > -1: # if found GIF beginning
        print 'Found \'GIF8\' at', begin
        end = packet_bytes.find(';')
        if end > -1: # if found GIF end
            print '---Found \';\' at', end
            file_name = str(time.time()) + '.gif'
            with open(file_name, 'wb') as image_file:
                print 'File', file_name, 'created\n'
                image_file.write(packet_bytes[begin:])
                image_file.close()
    ### JPEG ###
    begin = packet_bytes.find(chr(0xFF) + chr(0xD8))
    if begin > -1: # if found JPEG beginning
        print 'Found 0xFFD8 at', begin
        end = packet_bytes.find(chr(0xFF) + chr(0xD9))
        if end > -1: # if found JPEG end
            print '---Found 0xFFD9 at', end
            file_name = str(time.time()) + '.jpg'
            with open(file_name, 'wb') as image_file:
                print 'File', file_name, 'created\n'
                image_file.write(packet_bytes[begin:])
                image_file.close()

print 'Started sniffing...'
sendrecv.sniff(prn = small_stealer)
