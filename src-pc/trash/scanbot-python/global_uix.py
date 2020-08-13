import kivy

kivy.require("1.10.1")
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.spinner import Spinner
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.slider import Slider
from kivy.uix.popup import Popup
from kivy.uix.tabbedpanel import TabbedPanel
from kivy.uix.scrollview import ScrollView
from kivy.properties import NumericProperty, ObjectProperty, StringProperty
from kivy.lang import Builder


class LabelWithHint(Button):
    hint_text = StringProperty()

    def display_hint(self):
        popup = Popup(
            title_font="res/classic-robot-font/Classic_Robot_Bold.ttf",
            title="Dokumentacja EV6-223 - podpowiedź",
            content=Label(text=self.hint_text, halign="left"),
            size_hint=(None, None),
            size=(600, 400),
        )
        popup.open()


class ColoredLabel(Label):
    pass


class ColoredLabel2(Label):
    pass


class FilledBoxLayout(BoxLayout):
    pass


class DarkFilledBoxLayout(BoxLayout):
    pass


##my layouts##


class Page(BoxLayout):
    pass


class Column(BoxLayout):
    pass


class Section(BoxLayout):
    pass


##more complex widgets##


class Indicator(BoxLayout):
    pass


class MyTextInput(BoxLayout):
    pass


class PrecisionSlider(BoxLayout):
    def increase_value(self):
        if (self.ids.slider.value + self.ids.slider.step) <= self.max:
            self.ids.slider.value += self.ids.slider.step

    def decrease_value(self):
        if (self.ids.slider.value - self.ids.slider.step) >= self.min:
            self.ids.slider.value -= self.ids.slider.step


class AnalogValue(BoxLayout):
    # def __init__(self):
    # set gui to proper starting value
    # self.update_from_reg();

    def update_from_reg(self):
        self.ids.slider.ids.slider.value = round(self.reg_value * self.step, 1)


class DigitalValue(BoxLayout):
    def update_from_reg(self):
        self.ids.spinner.text = self.values[round(self.reg_value)]


class ModuleWidget(DarkFilledBoxLayout):
    module_name = StringProperty()
    module_desc = StringProperty()
    module_id = StringProperty()
    module_type = StringProperty()
    module_status = StringProperty()


Builder.load_file("kv/global_uix.kv")
