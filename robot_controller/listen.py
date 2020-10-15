#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" Gives commands to take an object from the shelf """
import nao_interface


def set_color(word):
    # ToDo: valori non veritieri
    threshold = 10
    papera = [255, 255, 0, threshold]
    palla = [255, 0, 0, threshold]
    macchina = [0, 0, 255, threshold]

    nao_interface.execute_command("ALColorBlobDetectionProxy", "setColor", vars()[word.lower()])


def listen():
    words = ['Palla', 'Papera', 'Macchina']

    sub = nao_interface.get_events()

    nao_interface.execute_command("ALSpeechRecognitionProxy", "pause", [True])
    nao_interface.execute_command("ALSpeechRecognitionProxy", "setLanguage", ['Italian'])
    nao_interface.execute_command("ALSpeechRecognitionProxy", "setVocabulary", [words, False])
    nao_interface.execute_command("ALSpeechRecognitionProxy", "subscribe", ['listen'])
    nao_interface.execute_command("ALSpeechRecognitionProxy", "pause", [False])
    nao_interface.execute_command("ALSpeechRecognitionProxy", "setParameter", ['speed', 80])

    # return "Papera"
    for msg in sub.listen():
        if msg['type'] == 'message':
            event = msg['data'].split(';')[0]
            value = msg['data'].split(';')[1]
            if event == 'WordRecognized':
                arr = eval(value)
                word = arr[0]
                if arr[1] > 0.4:
                    nao_interface.execute_command("ALTextToSpeechProxy", "say", [word])
                    nao_interface.execute_command("ALSpeechRecognitionProxy", "unsubscribe", ['listen'])
                    return word



