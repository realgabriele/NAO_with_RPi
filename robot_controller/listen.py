#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Gives commands to take an object from the shelf """
import NAO_interface


def set_color(word):
    # ToDo: valori non veritieri
    threshold = 10
    papera = [255, 255, 0, threshold]
    palla = [255, 0, 0, threshold]
    macchina = [0, 0, 255, threshold]

    NAO_interface.execute_command("ALColorBlobDetectionProxy", "setColor", vars()[word.lower()])


def listen():
    words = ['Palla', 'Papera', 'Macchina']

    sub = NAO_interface.get_events()

    NAO_interface.execute_command("ALSpeechRecognitionProxy", "pause", [True])
    NAO_interface.execute_command("ALSpeechRecognitionProxy", "setLanguage", ['Italian'])
    NAO_interface.execute_command("ALSpeechRecognitionProxy", "setVocabulary", [words, False])
    NAO_interface.execute_command("ALSpeechRecognitionProxy", "subscribe", ['listen'])
    NAO_interface.execute_command("ALSpeechRecognitionProxy", "pause", [False])
    NAO_interface.execute_command("ALSpeechRecognitionProxy", "setParameter", ['speed', 80])

    # return "Papera"
    for msg in sub.listen():
        if msg['type'] == 'message':
            event = msg['data'].split(';')[0]
            value = msg['data'].split(';')[1]
            if event == 'WordRecognized':
                arr = eval(value)
                word = arr[0]
                if arr[1] > 0.4:
                    NAO_interface.execute_command("ALTextToSpeechProxy", "say", [word])
                    NAO_interface.execute_command("ALSpeechRecognitionProxy", "unsubscribe", ['listen'])
                    return word



