import diagnostic_updater
import diagnostic_msgs
from std_msgs.msg import Int64


class Demo_Diagnostics(diagnostic_updater.DiagnosticTask):

    def __init__(self, parent_node, parent_updater):
        diagnostic_updater.DiagnosticTask.__init__(
            self,
            'Test Diagnostic')
        parent_node.get_logger().info('Diagnostics demo online!')

        # Some diagnostic tasks are very common, such as checking the rate
        # at which a topic is publishing, or checking that timestamps are
        # sufficiently recent. FrequencyStatus and TimestampStatus can do these
        # checks for you.
        #
        # Usually you would instantiate them via a HeaderlessTopicDiagnostic
        # (FrequencyStatus only, for topics that do not contain a header) or a
        # TopicDiagnostic (FrequencyStatus and TimestampStatus, for topics that
        # do contain a header).
        self.pub1_freq = diagnostic_updater.HeaderlessTopicDiagnostic('topic1', parent_updater,
                                                                      diagnostic_updater.FrequencyStatusParam(
                                                                          {'min': 0.5, 'max': 2},
                                                                          0.1, 10))
        # callbacks need to accept a message argument
        def callback(msg: Int64):
            # Count and keep the statistics up to date.
            self.pub1_freq.tick()

        parent_node.create_subscription(Int64, (parent_node.get_namespace() + '/diagnostics_demo'),
                                        callback, 10)



    def run(self, stat):
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                     'This is a silly updater.')
        stat.add('Silliness of this updater', '1000')
        return stat
