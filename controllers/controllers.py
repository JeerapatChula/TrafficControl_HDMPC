import sys
from .Fixed.controller import Controller as Fixed
from .Fuzzy.controller import Controller as Fuzzy
from .HDC.controller import Controller as HDC
from .HDMPC1.controller import Controller as HDMPC1
from .HDMPC2.controller import Controller as HDMPC2
from .HDMPC3.controller import Controller as HDMPC3
from .HDMPC4.controller import Controller as HDMPC4 # Thesis version
from .HDMPC5.controller import Controller as HDMPC5 # Journal version
from .HDMPC6.controller import Controller as HDMPC6 # Journal version
from .HDMPC_PROOF.controller import Controller as HDMPC_PROOF


class Controllers():

    @staticmethod
    def load_controller(intermediary, controller_configs, network_config):

        controller = controller_configs["controller"]

        if controller == "Fixed":
            return Fixed()
        elif controller == "Fuzzy":
            return Fuzzy(intermediary, network_config)
        elif controller == "HDC":
            return HDC(intermediary, controller_configs, network_config)
        elif controller == "HDMPC1":
            return HDMPC1(intermediary, controller_configs, network_config)
        elif controller == "HDMPC2":
            return HDMPC2(intermediary, controller_configs, network_config)
        elif controller == "HDMPC3":
            return HDMPC3(intermediary, controller_configs, network_config)
        elif controller == "HDMPC4":
            return HDMPC4(intermediary, controller_configs, network_config)
        elif controller == "HDMPC5":
            return HDMPC5(intermediary, controller_configs, network_config)
        elif controller == "HDMPC6":
            return HDMPC6(intermediary, controller_configs, network_config)
        elif controller == "HDMPC_PROOF":
            return HDMPC_PROOF(intermediary, controller_configs, network_config)
        else:
            sys.exit("The requiring controller doesn't exist in the controller list.")
