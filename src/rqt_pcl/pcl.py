#!/usr/bin/env python

from qt_gui.plugin import Plugin

from .pcl_widget import PclWidget


class RosPcl(Plugin):
    def __init__(self, context):
        super(RosPcl, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        self._context = context

        # Create QWidget
        topic = '/rosarnl_node/sim_lms1xx_1_pointcloud'
        self._widget = PclWidget(topic)

        # Add widget to the user interface
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget.vtkWidget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
