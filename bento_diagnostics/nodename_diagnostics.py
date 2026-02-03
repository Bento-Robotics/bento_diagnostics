import diagnostic_msgs
from diagnostic_msgs.msg._diagnostic_status import Metaclass_DiagnosticStatus


class NodenameDiagnostic:
    """Class for storing log snippets to look for and how to process them"""

    def __init__(self):
        self.is_visible: int = -1
        self.node_name: str = ""
        self.message: str = ""
        self.level: Metaclass_DiagnosticStatus = diagnostic_msgs.msg.DiagnosticStatus


class NodenameDiagnostics:

    def __init__(self, configuration, updater):
        self.diagnostic_components: dict[str, NodenameDiagnostic] = {}

        # Parse YAML:
        # "nodename_diagnostics" top-level list of diagnostic thingies
        for diagnostic in configuration["nodename_diagnostics"]:
            # save the stuff from YAML into a NodenameDiagnostic entry
            message = NodenameDiagnostic()
            message.node_name = diagnostic["node_name"]
            message.message = diagnostic["message"]
            # convert text to python code - just using numbers 0..3 didn't work :(
            message.level = eval("diagnostic_msgs.msg.DiagnosticStatus." + diagnostic["level"])

            # and save that entry to diagnostic_components
            self.diagnostic_components[diagnostic["node_name"]] = message
            # add a diagnostic_updater hook for it too
            updater.add(diagnostic["node_name"], self.update_diagnostics(diagnostic["node_name"]))

    def update_diagnostics(self, component_name):

        component = self.diagnostic_components[component_name]

        def defined_checker(stat):
            if component.is_visible == -1:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE, 'Not updated yet')
            elif component.is_visible == 1:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Node online.')
            else:  # component.is_visible == 0
                stat.summary(component.level, component.message)
                # stat.add('Henlo', 'frens')
            return stat

        return defined_checker

    def nodename_callback(self, nodenames: list[tuple[str, str]]):
        nodes: list[str] = []
        for node in nodenames:
            # remove fist '/' char of namespace and append add node name
            nodes.append(node[1][1:] + '.' + node[0])

        for node in self.diagnostic_components.values():
            if node.is_visible != -1: node.is_visible = 0
            for nodename in nodes:
                if nodename in node.node_name:
                    node.is_visible = 1
                # print(nodename)  # parrot all visiblie node names
