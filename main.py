from hcsr04 import HCSR04
from machine import Pin, PWM
from time import sleep, time, time_ns
from servo import Servo
"""
Biblioteki potrzebne do uruchomienia programu na mikrokontrolerze
time i machine dostępne domyślnie w micropythonie dla ESP32
hscr04 dostępne pod https://github.com/rsc1975/micropython-hcsr04
servo dostępne pod https://github.com/redoxcode/micropython-servo
"""


class DistanceSensor():
    """
    Klasa odpowiedzialna za sterowanie czujnikiem odległości HCSR04 wykrywającym
    pojazd

    :param trigger_pin: numer GPIO płytki do którego podłączona jest nóżna trigger
        czujnika
    :type trigger_pin: int

    :param echo_pin: numer GPIO płytki do którego podłączona jest nóżka echo
        czujnika
    :type echo_pin: int

    :param detection_distance: maksymalna odległość tramwaju od czujnika aby ten
        wykrył jego obecność, w cm
    :type detection_distance: int
    """
    def __init__(self, trigger_pin, echo_pin, detection_distance):
        """
        Metoda inicjująca, używa obiektu biblioteki HCSR04 aby ułatwić
        użycie czujnika odległości
        """
        self.sensor = HCSR04(trigger_pin, echo_pin, echo_timeout_us=10000)
        self.detection_distance = detection_distance

    def detect(self):
        """
        Metoda wykrywająca tramwaj, zwraca TRUE jeśli tramwaj znajduje się
        w dystancie wykrycia, FALSE w przeciwnym wypadku
        """
        if self.get_distance() <= self.detection_distance:
            return True
        return False

    def get_distance(self):
        """
        Metoda zwracająca odległość tramwaju od czujnika
        """
        return self.sensor.distance_cm()


class Blink():
    """
    Klasa odpowiadająca za sterowanie diodami LED lub brzęczykiem

    :param pin: numer GPIO płytki do którego podpięte są lampki lub brzęczyk
    :type pin: int

    :param blink_interval: określa jak długo włączone a potem wyłączone są
        lampki/buzzer (połowa okresu migania), w milisekundach
    :type blink_interval: int
    """
    def __init__(self, pin, blink_interval):
        """
        Metoda inicjująca
        """
        self.pin = Pin(pin, Pin.OUT)
        self.is_on = False
        self.blink = False
        self.blink_interval = blink_interval * 1000000
        self.time = 0

    def test(self):
        """
        Metoda przeprowadzająca test lampek/brzęczyka poprzez uruchomienie ich
        oraz wyłączenie 5 razy
        """
        for i in range(5):
            self.pin.on()
            sleep(0.2)
            self.pin.off()
            sleep(0.2)

    def on(self):
        """
        Metoda włączająca lampki/brzęczyk
        """
        self.pin.on()
        self.is_on = True

    def off(self):
        """
        Metoda wyłączająca lampki/brzęczyk
        """
        self.pin.off()
        self.is_on = False

    def start(self):
        """
        Metoda uruchamiająca cykl lampek/brzęczyka
        """
        if not self.blink:
            self.blink = True
            self.time = time_ns()
            self.on()

    def stop(self):
        """
        Metoda zarzymująca cykl lampek/brzęczyka
        """
        if self.blink:
            self.blink = False
            self.off()

    def switch_state(self):
        """
        Metoda przełączająca lampki/brzęczyk z wyłączonego na włączony lub
        na odwrut
        """
        if self.is_on:
            self.off()
        else:
            self.on()

    def main(self):
        """
        Metoda odpowiadająca za przeprowadzanie wszystkich operacji na
        obiekcie, uruchamiana co każdy cykl programu, pilnuje aby
        lampki/brzęczyk były włączone/wyłączone przez określony czas
        """
        if self.blink:
            if self.time + self.blink_interval < time_ns():
                self.time = time_ns()
                self.switch_state()


class Gate():
    """
    Klasa odpowiadająca za sterowanie bramką kontrolowaną przez servo SG90

    :param pin: numer GPIO płytki do którego podpięta jest nózka kontrolujaca servo
    :type pin: int

    :param close_delay: określa jak długo ma być zamknięta bramka po tym jak
        przestanie być wykrywany tramwaj, w milisekundach
    :type close_delay: int
    """
    def __init__(self, pin, close_delay):
        """
        Metoda inicjująca
        """
        self.gate = Servo(pin=pin)
        self.time = 0
        self.closed = False
        self.close_delay = close_delay * 1000000

    def test(self):
        """
        Metoda przeprowadzająca test bramki przez zamknięcie jej na sekundę
        a następnie ponowne jej otwarcie
        """
        self.gate.move(90)
        sleep(1)
        self.gate.move(0)
        sleep(1)

    def close(self):
        """
        Metoda zamykająca bramkę
        """
        self.time = time_ns()
        if not self.closed:
            self.gate.move(90)
            self.closed = True

    def open(self):
        """
        Metoda otwierająca bramkę
        """
        if self.closed:
            self.gate.move(0)
            self.closed = False

    def main(self):
        """
        Metoda odpowiadająca za przeprowadzanie wszystkich operacji na
        obiekcie, uruchamiana co każdy cykl programu, pilnuje aby
        bramka była zamknięta przez określony czas po wykryciu tramwaju
        """
        if self.closed:
            if self.time + self.close_delay < time_ns():
                self.open()


def test_systems():
    """
    Funkcja przeprowadzająca testy na wszystkich obiektach programu
    """
    led_built_in = Pin(2, Pin.OUT)
    led_built_in.on()
    print("TESTING SYSTEMS")
    lights.test()
    buzzer.test()
    gate.test()
    print("COMPLETE")
    led_built_in.off()


DETECTION_DISTANCE_CM = 10  # maksymalny dystans do wykrycia tramwaju w cm
GATE_DELAY_MS = 2000  # jak długo bramka zostanie zamknięta po przejeździe tramwaju, w ms
LIGHTS_INTERVAL_MS = 100  # połowa okresu migania lampek, w ms
BUZZER_INTERVAL_MS = 200  # połowa okresu dźwięku brzęczyka, w ms


gate = Gate(22, GATE_DELAY_MS)  # inicjacja bramki
distance_sensor = DistanceSensor(5, 18, DETECTION_DISTANCE_CM)  # inicjacja czujnika odległości
lights = Blink(19, LIGHTS_INTERVAL_MS)  # inicjacja światełek
buzzer = Blink(21, BUZZER_INTERVAL_MS)  # inicjacja brzęczyka

test_systems()  # testowanie obiektów

print("\033[91m")

while True:  # główna pętla programu
    if distance_sensor.detect():  # sprawdza czy przejeżdża tramwaj
        # jeśli wykryto tramwaj uruchamia lampki, brzęczyk i zamyka bramkę
        lights.start()
        buzzer.start()
        gate.close()
        print("UWAGA TRAMWAJ!!")
    else:
        # jeśli nie wykryto wyłącza lampki i brzęczyk, bramka zamknie się sama po określonym czasie
        lights.stop()
        buzzer.stop()
        print("")
    # wywołuje główne funkcje obiektów
    lights.main()
    buzzer.main()
    gate.main()
    sleep(0.05)  # pętla powtarzana 20 razy na sekundę
