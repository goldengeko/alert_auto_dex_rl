from robosuite.models.objects import MujocoXMLObject

class CustomOmniBoard(MujocoXMLObject):
    def __init__(self, name="omniboard"):
        super().__init__(
            fname="/home/guts/alert_auto_dex_rl/omni-board.xml",
            name=name,
            joints="default",  # Ensures a free joint is added if not defined in XML
        )

omniboard = CustomOmniBoard()
