from ghost_bridge.netcat import netcat


class ActionFeedbackCtrl:
    """
        The ActionFeedbackCtrl class is used to send feedback to Ghost about the state of the actions
        that are being executed on the robot.
    """

    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port

    def say_started(self):
        """ Notify Ghost that the say command has started

        :return: None
        """

        content = '(say-started)\n'
        netcat(self.hostname, self.port, content)

    def say_finished(self):
        """ Notify Ghost that the say command has finished

        :return: None
        """

        content = '(say-finished)\n'
        netcat(self.hostname, self.port, content)
