from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QFormLayout,
    QLabel,
    QProgressBar,
    QVBoxLayout,
    QWidget,
)

from antoniq_interfaces.msg import MissionStatus

# If no MissionStatus message has arrived in this long, the display is treated as stale (the
# mission node may have exited, crashed, or never started) rather than just "a bit slow".
_STALE_AFTER_SEC = 3.0
_REFRESH_PERIOD_MS = 200


class MissionStatusWidget(QWidget):
    """Displays antoniq_interfaces/MissionStatus, published by antoniq_mission_node."""

    def __init__(self, node, topic='mission_status'):
        super(MissionStatusWidget, self).__init__()
        self._node = node
        self._last_message = None
        self._last_message_stamp = None

        self.setWindowTitle('Antoniq Mission Status')

        self._description_label = QLabel('Waiting for %s...' % topic)
        self._description_label.setWordWrap(True)
        self._description_label.setStyleSheet('font-size: 13pt; font-weight: bold;')

        self._progress_bar = QProgressBar()
        self._progress_bar.setRange(0, 1)
        self._progress_bar.setValue(0)
        self._progress_bar.setTextVisible(True)
        self._progress_bar.setFormat('no data')

        self._row_label = QLabel('-')
        self._waypoint_label = QLabel('-')
        self._frame_label = QLabel('-')
        self._age_label = QLabel('-')

        form = QFormLayout()
        form.addRow('Row:', self._row_label)
        form.addRow('Waypoint:', self._waypoint_label)
        form.addRow('Target frame:', self._frame_label)
        form.addRow('Last update:', self._age_label)

        layout = QVBoxLayout()
        layout.addWidget(self._description_label)
        layout.addWidget(self._progress_bar)
        layout.addLayout(form)
        layout.addStretch(1)
        self.setLayout(layout)

        # The subscription callback runs on rclpy's spin thread; it only ever stores the latest
        # message (a plain attribute write, safe under the GIL). Everything Qt-facing happens on
        # this QTimer tick, on the GUI thread, same as rqt_topic's refresh pattern.
        self._subscription = self._node.create_subscription(MissionStatus, topic,
                                                            self._message_callback, 10)
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh)
        self._refresh_timer.start(_REFRESH_PERIOD_MS)

    def _message_callback(self, msg):
        self._last_message = msg
        self._last_message_stamp = self._node.get_clock().now()

    def _refresh(self):
        if self._last_message is None:
            return

        msg = self._last_message
        age_sec = (self._node.get_clock().now() - self._last_message_stamp).nanoseconds / 1e9
        stale = age_sec > _STALE_AFTER_SEC

        self._description_label.setText(msg.status_description or '(no description)')
        self._description_label.setEnabled(not stale)
        self._row_label.setText('%d / %d' % (msg.row + 1, msg.row_count))
        self._waypoint_label.setText('%d / %d' % (msg.waypoint_index, msg.waypoint_count))
        self._frame_label.setText(msg.waypoint_frame)
        self._age_label.setText('%.1fs ago%s' % (age_sec, ' -- STALE' if stale else ''))

        if msg.waypoint_count > 0:
            self._progress_bar.setRange(0, msg.waypoint_count)
            self._progress_bar.setValue(msg.waypoint_index)
            self._progress_bar.setFormat('waypoint %v / %m')
        self._progress_bar.setEnabled(not stale)

    def shutdown_widget(self):
        self._refresh_timer.stop()
        self._node.destroy_subscription(self._subscription)
