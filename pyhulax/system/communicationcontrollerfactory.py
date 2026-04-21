from . import communicationcontroller
from . import networkcontroller
from . import serialcontroller

from .communicationcontroller import ControllerType

controller_list = [
    networkcontroller.NetworkController,
    serialcontroller.SerialController,
]


class CommunicationControllerFactory:
    @staticmethod
    def get_controller(controller_type):
        global controller_list
        for controller in controller_list:
            if controller.controller_type == controller_type:
                return controller()
        return None


if __name__ == "__main__":
    sc = CommunicationControllerFactory.get_controller()
    buffer = sc.get_buffer()
    buffer.set_timeout(2)
    sc.serial_init(39)
    sc.send_buf(b"hello world")
    # print(buffer.get_data(5))
    print(bytearray(buffer.get_data(5)))
    print(bytearray(buffer.get_data(5)))
    print(bytearray(buffer.get_data(5)))
