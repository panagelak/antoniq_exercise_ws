from rqt_gui_py.plugin import Plugin

from antoniq_mission_status_rqt.mission_status_widget import MissionStatusWidget


class MissionStatusPlugin(Plugin):

    def __init__(self, context):
        super(MissionStatusPlugin, self).__init__(context)
        self.setObjectName('MissionStatusPlugin')

        assert hasattr(context, 'node'), 'Context does not have a node.'
        self._widget = MissionStatusWidget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_widget()
