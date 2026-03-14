import board
import busio
from genie import Genie, GENIE_OBJ_SLIDER

uart = busio.UART(board.TX, board.RX, baudrate=115200)

genie = Genie(uart)
genie.begin()


def my_handler(event):

    print("Event:")
    print("object:", event.obj)
    print("index:", event.index)
    print("value:", event.value)


genie.attach_event_handler(my_handler)


while True:

    genie.do_events()

    # Example: update slider widget
    genie.write_object(GENIE_OBJ_SLIDER, 0, 50)


