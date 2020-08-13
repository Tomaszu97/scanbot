import kivy
from kivy.graphics import *
from kivy.app import App
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.clock import Clock
from kivy.config import Config
from kivy.lang import Builder
from threading import Thread
from kivy.uix.widget import Widget
from global_uix import *
from config import *
import serial.tools.list_ports
from math import floor, ceil

kivy.require("1.11.1")


##import kv files##
Builder.load_file("kv/initial_screen.kv")
Builder.load_file("kv/main_screen.kv")

##kivy configuration##
Config.set("kivy", "window_icon", "res/icon.png")
Config.set("graphics", "multisamples", 0)  # disable antialiasing
Config.set("graphics", "width", "1280")
Config.set("graphics", "height", "720")
Config.set("input", "mouse", "mouse,disable_multitouch")
Config.write()


class DrawBox(Widget):
    def __init__(self, **kwargs):
        super(DrawBox, self).__init__(**kwargs)
        self.size_hint_y = 2


class MyScreen(Screen):
    def __init__(self):
        Screen.__init__(self)

    def display_popup(
        self,
        _title="Popup title",
        _text="Popup text",
        _auto_dismiss=True,
        _size=(300, 150),
    ):
        popup = Popup(
            title_font="res/classic-robot-font/Classic_Robot_Bold.ttf",
            title=_title,
            content=Label(text=_text, halign="left"),
            size_hint=(None, None),
            size=_size,
            auto_dismiss=_auto_dismiss,
        )
        popup.open()
        return popup


class InitialScreen(MyScreen):
    def refresh_button_handler(self):
        self.ids.serial_list.clear_widgets()
        for comport in serial.tools.list_ports.comports():
            device = comport.device
            btn = Button(text=device, id=device, size_hint_y=None)
            btn.bind(on_release=self.serial_port_button_handler)
            self.ids.serial_list.add_widget(btn)

    def serial_port_button_handler(self, instance=None):
        SERIAL_PORT = instance.id
        self.parent.transition.direction = "left"
        self.parent.transition.duration = 0.4
        self.parent.current = "main_screen"
        print(SERIAL_PORT)


class MainScreen(MyScreen):
    def __init__(self, **kwargs):
        super(MainScreen, self).__init__(**kwargs)
        self.dbox = DrawBox(size_hint_y=None)
        self.ids.main_vertical_layout.add_widget(self.dbox)

    def proceed_button_handler(self, instance=None):
        # self.parent.transition.direction = "right"
        # self.parent.transition.duration = 0.4
        # self.parent.current = "initial_screen"

        self.dbox.canvas.clear()
        with self.dbox.canvas.before:
            w = self.dbox.width
            h = self.dbox.height
            Color(0, 0, 0)
            Rectangle(size=(w, h))
            Color(0, 0.7, 1.0)

            hlist = []
            for i in range(ceil(h / 100) + 1):
                hlist.append(i * 100)
            wlist = []
            for i in range(ceil(w / 100) + 1):
                wlist.append(i * 100)

            for h in hlist:
                Line(points=[(0, h), (w, h)], width=1)
            for w in wlist:
                Line(points=[(w, 0), (w, h)], width=1)


class MyApp(App):
    def __init__(self):
        App.__init__(self)
        self.title = "ScanBot"
        self.icon = "res/favicon.png"

    def build(self):
        screen_manager = ScreenManager()
        screen_manager.add_widget(InitialScreen())
        screen_manager.add_widget(MainScreen())
        screen_manager.current = "main_screen"
        return screen_manager

    def on_start(self):
        return super().on_start()

    def on_stop(self):
        return super().on_stop()


if __name__ == "__main__":
    MyApp().run()
