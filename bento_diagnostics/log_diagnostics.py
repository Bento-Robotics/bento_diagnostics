import diagnostic_msgs
from diagnostic_msgs.msg._diagnostic_status import Metaclass_DiagnosticStatus
from rcl_interfaces.msg import Log


class ErrorMessage:
    """Class for storing log snippets to look for and how to process them"""

    def __init__(self):
        self.index_pointer = -1
        self.names: list[str] = []
        self.keywords: list[str] = []
        self.messages: list[str] = []
        self.levels: list[Metaclass_DiagnosticStatus] = []
        self.priorities: list[int] = []


class LogDiagnostics:

    def __init__(self, configuration, updater):
        self.diagnostic_components: dict[str, ErrorMessage] = {}

        # Parse YAML:
        # "log_diagnostics" top-level list of diagnostic thingies
        for diagnostic in configuration["log_diagnostics"]:
            # save the stuff from YAML into a ErrorMessage entry
            message = ErrorMessage()
            # "log_messages" list of individual log keywords to look for and react to
            for error_message in diagnostic["log_messages"]:
                message.names.append(error_message["name"])
                message.keywords.append(error_message["keyword"])
                message.messages.append(error_message["message"]),
                # convert text to python code - just using numbers 0..3 didn't work :(
                message.levels.append(eval("diagnostic_msgs.msg.DiagnosticStatus." + error_message["level"]))
                try:  # this lets users leave out the priority in YAML and have 0 be default
                    message.priorities.append(error_message["priority"])
                except KeyError:
                    message.priorities.append(0)

            # and save that entry to diagnostic_components
            self.diagnostic_components[diagnostic["node_name"]] = message
            # add a diagnostic_updater hook for it too
            updater.add(diagnostic["node_name"], self.update_diagnostics(diagnostic["node_name"]))


    def update_diagnostics(self, component_name):
        component = self.diagnostic_components[component_name]
        def defined_checker(stat):
            if component.index_pointer == -1:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE, 'Not updated yet')
            else:
                stat.summary(component.levels[component.index_pointer], component.messages[component.index_pointer])
                # stat.add('Henlo', 'frens')
            return stat

        return defined_checker


    def log_callback(self, msg: Log):
        for errormessage in self.diagnostic_components.values():
            for idx, keyword in enumerate(errormessage.keywords):
                if (keyword in msg.msg and
                        (errormessage.index_pointer == -1 or
                         errormessage.priorities[idx] >= errormessage.priorities[errormessage.index_pointer])):
                    errormessage.index_pointer = idx
                    # print(msg.msg)  # parrot all key log messages
