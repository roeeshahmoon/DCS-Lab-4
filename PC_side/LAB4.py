import serial as ser
import time

def main():
    l = "Hello World"
    menu_num_list = ['1', '2', '3', '4', '5', '6', '7', '8','9']
    menu = """-------------------Menu---------------------
1. Blink RGB LED, color by color with delay of X[ms]
2. Count up onto LCD screen with delay of X[ms]
3. Circular tone series via Buzzer with delay of X[ms]
4. Get delay time X[ms]:
5. LDR 3-digit value [v] onto LCD
6. Clear LCD screen
7. On each PB1 or PB2 pressed, Send a Massage “Hello world”
8. Show menu
9. Sleep
"""


    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)  # timeout of 1 sec so that the read and write operations are blocking,
    # after the timeout the program continues
    enableTX = True
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()
    k = 0
    print(menu)
    while (1):


        while (s.in_waiting > 0):  # while the input buffer isn't empty
            enableTX = True
            buffer = s.read_all()
            if '7' == menuNum:
                line = s.read_all()
                a = buffer.decode("ascii")
                while(a != '\n'):
                    line = s.read_all()  # read  from the buffer until the terminator is received,
                    # readline() can also be used if the terminator is '\n'
                    if(buffer < bytes(0x80)):
                        a = buffer.decode("ascii")

                    if(a in l and a != '' ):
                        print(buffer.decode("ascii"))
                        if (s.in_waiting == 0):
                            enableTX = True
                    buffer = s.read_all()


            elif bytes('8', 'ascii') == buffer:
                print(menu)
            elif bytes('4', 'ascii') == buffer:
                delay_time = input("Enter Delay Time in ms:")
                bytesDelay = bytes(delay_time + '\n', 'ascii')
                s.write(bytesDelay)

            time.sleep(0.25)
            if (s.in_waiting == 0):
                enableTX = True
        # s.reset_input_buffer()
        while (s.out_waiting > 0 or enableTX):  # while the output buffer isn't empty
            menuNum = input("Enter menu number:")
            bytesMenuChar = bytes(menuNum, 'ascii')
            s.write(bytesMenuChar)
            if s.out_waiting == 0 and menuNum in menu_num_list :
                enableTX = False


if __name__ == '__main__':
    main()

